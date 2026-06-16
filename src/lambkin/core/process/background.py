# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Non-blocking process execution for lambkin benchmarks.

Wraps subprocess.Popen to launch processes in the background, allowing the
benchmark to continue while the process runs. Useful for starting long-running
services or ROS nodes that must run alongside the benchmark.
"""

from __future__ import annotations

import logging
import os
import shlex
import signal
import subprocess
import threading
from pathlib import Path
from types import TracebackType
from typing import IO, Any

from lambkin.common import defaults, exceptions, signals
from lambkin.core.process.cgroup import (
    kill_cgroup,
    make_process_cgroup,
    remove_cgroup,
    spawn_in_cgroup,
)
from lambkin.core.process.resource_monitor import ResourceMonitor
from lambkin.core.shell.proxy import CommandProxy

logger = logging.getLogger(__name__)


class BackgroundProcess:
    """Context manager that runs a command as a background process in a cgroup.

    The process is placed inside a dedicated cgroup v2 child directory under
    the iteration cgroup. On exit, the process tree is terminated with a
    graduated signal sequence: SIGTERM first, then SIGKILL for survivors.

    If the process dies before the context manager exits,
    ProcessDiedUnexpectedly is raised.
    """

    def __init__(
        self,
        argv: list[str],
        iteration_cgroup: Path,
        process_name: str,
        cwd: Path,
        dry_run: bool = False,
        env: dict[str, str] | None = None,
        stdout: IO[str] | None = None,
        stderr: IO[str] | None = None,
    ) -> None:
        """Initialize the BackgroundProcess.

        Args:
            argv (list[str]): The command to run as a list of tokens.
            iteration_cgroup (Path): The cgroup directory for this iteration.
            process_name (str): Process name shared across all output files for this
                process, including stdout, stderr and resource logs.
            cwd (Path): Working directory for the process and root for all
                output files.
            dry_run (bool): If True, log the command instead of executing it.
            env (dict | None): Environment variables for the process. If None,
                inherits from the parent.
            stdout: stdout stream for the process. If None, inherits from the parent.
            stderr: stderr stream for the process. If None, inherits from the parent.
        """
        self._argv = argv
        self._iteration_cgroup = iteration_cgroup
        self._process_name = process_name
        self._dry_run = dry_run
        self._cgroup: Path | None = None
        self._proc: subprocess.Popen[bytes] | None = None
        self._monitor: threading.Thread | None = None
        self._resource_monitor: ResourceMonitor | None = None
        self._died_unexpectedly: bool = False
        self._exiting: threading.Event = threading.Event()
        self._cwd = cwd
        self._env = env
        self._stdout = stdout
        self._stderr = stderr

    def _monitor_process(self) -> None:
        """Monitor thread that detects if the process dies unexpectedly.

        If the process dies before the context manager exits, sets
        sigusr1_pending and sends SIGUSR1 to the main thread to interrupt
        any blocking foreground process. BackgroundProcess.__exit__ is
        responsible for raising LambkinProcessDiedUnexpectedlyError.
        """
        assert self._proc is not None
        self._proc.wait()
        if not self._exiting.is_set():
            self._died_unexpectedly = True
            signals.sigusr1_pending.set()
            os.kill(os.getpid(), signal.SIGUSR1)

    def __enter__(self) -> BackgroundProcess:
        """Start the background process inside its own cgroup.

        In dry-run mode, logs the command at debug level instead of executing it.

        Returns:
            BackgroundProcess: This instance.
        """
        if self._dry_run:
            logger.debug("[DRY RUN BG] %s", shlex.join(self._argv))
            return self

        self._cgroup = make_process_cgroup(self._iteration_cgroup, self._argv)
        self._proc = spawn_in_cgroup(
            self._cgroup,
            self._argv,
            cwd=self._cwd,
            env=self._env,
            stdout=self._stdout,
            stderr=self._stderr,
        )

        self._resource_monitor = ResourceMonitor(
            process_name=self._process_name,
            cgroup=self._cgroup,
            output_path=self._cwd / f"{self._process_name}.resources.jsonl",
        )
        self._resource_monitor.start()

        self._monitor = threading.Thread(
            target=self._monitor_process,
            daemon=True,
        )
        self._monitor.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """Stop the background process and clean up its cgroup.

        Args:
            exc_type: Exception type if an exception is propagating, else None.
            exc_val: Exception value if an exception is propagating, else None.
            exc_tb: Exception traceback if an exception is propagating, else None.

        Raises:
            LambkinProcessDiedUnexpectedlyError: If the process died before this
                method was called and no other exception is already propagating.
        """
        if self._dry_run:
            return
        if self._stdout:
            self._stdout.close()
        if self._stderr:
            self._stderr.close()
        self._exiting.set()

        # Stop the resource monitor before killing the cgroup so the final
        # samples are flushed while the cgroup files are still readable.
        if self._resource_monitor is not None:
            self._resource_monitor.stop()

        assert self._cgroup is not None
        kill_cgroup(self._cgroup, grace_period=defaults.SIGTERM_GRACE_PERIOD)

        # Join after kill_cgroup — the monitor thread will have unblocked from
        # proc.wait() by the time the process is dead.
        if self._monitor is not None:
            self._monitor.join()

        remove_cgroup(self._cgroup)

        assert self._proc is not None
        if exc_type is None and self._died_unexpectedly:
            returncode = self._proc.poll()
            raise exceptions.LambkinProcessDiedUnexpectedlyError(
                self._argv, returncode or 1
            )


def background(proxy: CommandProxy, *args: Any, **kwargs: Any) -> BackgroundProcess:
    """Create a BackgroundProcess context manager from a command proxy.

    Args:
        proxy (CommandProxy): A command proxy representing the command to run.
        *args: Positional arguments to append to the command.
        **kwargs: Keyword arguments to convert to --flag value pairs.

    Returns:
        BackgroundProcess: A context manager that runs the command in the background.

    Raises:
        ValueError: If the proxy has no cgroup or working directory set.

    Example:
        with background(ctx.shell.ros2.bag.record, "-O", "output.mcap", "-a"):
        ...
    """
    per_call_log_output = kwargs.pop("log_output", None)
    argv = proxy.build_argv(*args, **kwargs)
    env = proxy.build_env()
    stdout, stderr = proxy.open_streams(per_call_log_output)
    cgroup = proxy.get_cgroup()
    if cgroup is None:
        raise ValueError("background() requires a proxy with a cgroup set.")
    cwd = proxy.get_cwd()
    if cwd is None:
        raise ValueError("background() requires a proxy with a working directory set.")
    return BackgroundProcess(
        argv=argv,
        iteration_cgroup=cgroup,
        process_name=proxy.get_process_name(),
        cwd=cwd,
        dry_run=proxy.get_dry_run(),
        env=env,
        stdout=stdout,
        stderr=stderr,
    )
