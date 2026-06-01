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

import os
import subprocess
import threading
from pathlib import Path
from typing import Any

from lambkin.common import defaults, exceptions
from lambkin.core.process.cgroup import kill_cgroup, make_process_cgroup, remove_cgroup
from lambkin.core.shell.proxy import CommandProxy


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
        cwd: Path | None = None,
        dry_run: bool = False,
        env: dict | None = None,
        stdout=None,
        stderr=None,
    ) -> None:
        """Initialize the BackgroundProcess.

        Parameters
        ----------
        argv : list[str]
            The command to run as a list of tokens.
        iteration_cgroup : Path
            The cgroup directory for this iteration.
        dry_run : bool
            If True, print the command instead of executing it.
        cwd : Path, optional
            Working directory for the process. If None, inherits from the parent.
        """
        self._argv = argv
        self._iteration_cgroup = iteration_cgroup
        self._dry_run = dry_run
        self._cgroup: Path | None = None
        self._proc: subprocess.Popen | None = None
        self._monitor: threading.Thread | None = None
        self._died_unexpectedly: bool = False
        self._exiting: threading.Event = threading.Event()
        self._cwd = cwd
        self._env = env
        self._stdout = stdout
        self._stderr = stderr

    def _enter_cgroup(self) -> None:
        """Write the current PID to cgroup.procs.

        Runs in the child process after fork() but before exec().
        """
        (self._cgroup / "cgroup.procs").write_text(str(os.getpid()))

    def _monitor_process(self) -> None:
        """Monitor thread that detects if the process dies unexpectedly."""
        self._proc.wait()
        if not self._exiting.is_set():
            self._died_unexpectedly = True

    def __enter__(self) -> BackgroundProcess:
        """Start the background process inside its own cgroup.

        Returns:
        -------
        BackgroundProcess
            This instance.
        """
        if self._dry_run:
            print(f"[DRY RUN BG] {' '.join(self._argv)}")
            return self

        self._cgroup = make_process_cgroup(self._iteration_cgroup, self._argv)
        self._proc = subprocess.Popen(
            self._argv,
            preexec_fn=self._enter_cgroup,
            cwd=self._cwd,
            env=self._env,
            stdout=self._stdout,
            stderr=self._stderr,
        )

        self._monitor = threading.Thread(
            target=self._monitor_process,
            daemon=True,
        )
        self._monitor.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Stop the background process and clean up its cgroup.

        Parameters
        ----------
        exc_type :
            Exception type if an exception is propagating, else None.
        exc_val :
            Exception value if an exception is propagating, else None.
        exc_tb :
            Exception traceback if an exception is propagating, else None.

        Raises:
        ------
        LambkinProcessDiedUnexpectedlyError
            If the process died before this method was called and no other
            exception is already propagating.
        """
        if self._dry_run:
            return
        if self._stdout:
            self._stdout.close()
        if self._stderr:
            self._stderr.close()
        self._exiting.set()

        kill_cgroup(self._cgroup, grace_period=defaults.SIGTERM_GRACE_PERIOD)

        if self._monitor is not None:
            self._monitor.join()

        remove_cgroup(self._cgroup)

        if exc_type is None and self._died_unexpectedly:
            returncode = self._proc.poll()
            raise exceptions.LambkinProcessDiedUnexpectedlyError(
                self._argv, returncode or 1
            )


def background(proxy: CommandProxy, *args: Any, **kwargs: Any) -> BackgroundProcess:
    """Create a BackgroundProcess context manager from a command proxy.

    Parameters
    ----------
    proxy : CommandProxy
        A command proxy representing the command to run.
    *args :
        Positional arguments to append to the command.
    **kwargs :
        Keyword arguments to convert to --flag value pairs.

    Returns:
    -------
    BackgroundProcess
        A context manager that runs the command in the background.

    Example:
    -------
    with background(ctx.shell.ros2.bag.record, "-O", "output.mcap", "-a"):
        ...
    """
    argv = proxy.build_argv(*args, **kwargs)
    env = proxy.build_env()
    stdout, stderr = proxy.open_streams()
    return BackgroundProcess(
        argv=argv,
        iteration_cgroup=proxy.get_cgroup(),
        cwd=proxy.get_cwd(),
        dry_run=proxy.get_dry_run(),
        env=env,
        stdout=stdout,
        stderr=stderr,
    )
