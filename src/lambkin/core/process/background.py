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
from collections.abc import Sequence
from pathlib import Path
from types import TracebackType
from typing import IO, Any

from lambkin.common import defaults, exceptions, signals
from lambkin.core.process.cgroup import kill_cgroup, make_process_cgroup, remove_cgroup
from lambkin.core.process.resources import ResourceSampler
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
        cwd: Path | None = None,
        dry_run: bool = False,
        env: dict[str, str] | None = None,
        stdout: IO[str] | None = None,
        stderr: IO[str] | None = None,
        measure: Sequence[str] = (),
        measure_interval: float | None = None,
    ) -> None:
        """Initialize the BackgroundProcess.

        Args:
            argv (list[str]): The command to run as a list of tokens.
            iteration_cgroup (Path): The cgroup directory for this iteration.
            cwd (Path | None): Working directory for the process. If None,
                inherits from the parent.
            dry_run (bool): If True, log the command instead of executing it.
            env (dict | None): Environment variables for the process. If None,
                inherits from the parent.
            stdout: stdout stream for the process. If None, inherits from the parent.
            stderr: stderr stream for the process. If None, inherits from the parent.
            measure (Sequence[str]): Names of processes to measure resource usage
                for, matched against the basename of argv[0]. Empty disables
                measurement.
            measure_interval (float | None): Seconds between resource samples.
                If None, uses defaults.MEASURE_INTERVAL.
        """
        self._argv = argv
        self._iteration_cgroup = iteration_cgroup
        self._dry_run = dry_run
        self._cgroup: Path | None = None
        self._proc: subprocess.Popen[bytes] | None = None
        self._monitor: threading.Thread | None = None
        self._died_unexpectedly: bool = False
        self._exiting: threading.Event = threading.Event()
        self._cwd = cwd
        self._env = env
        self._stdout = stdout
        self._stderr = stderr
        self._measure = tuple(measure)
        self._measure_interval = (
            defaults.MEASURE_INTERVAL if measure_interval is None else measure_interval
        )
        self._sampler: ResourceSampler | None = None

    def _enter_cgroup(self) -> None:
        """Write the current PID to cgroup.procs.

        Runs in the child process after fork() but before exec().
        """
        assert self._cgroup is not None
        (self._cgroup / "cgroup.procs").write_text(str(os.getpid()))

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
        old_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGUSR1})
        try:
            self._monitor.start()
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, old_mask)

        # Started after the death monitor, so a sampler problem can never
        # leave the process running unmonitored.
        if self._measure:
            self._sampler = ResourceSampler(
                names=self._measure,
                cgroup=self._cgroup,
                output_dir=self._cwd or Path.cwd(),
                interval=self._measure_interval,
            )
            self._sampler.start()
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

        assert self._cgroup is not None
        try:
            # While the process is still alive: once it is signalled its /proc
            # entry is gone, taking VmHWM and the CPU counters with it.
            if self._sampler is not None:
                self._sampler.stop()
        finally:
            kill_cgroup(self._cgroup, grace_period=defaults.SIGTERM_GRACE_PERIOD)

            if self._monitor is not None:
                self._monitor.join()

            remove_cgroup(self._cgroup)

        assert self._proc is not None
        if self._died_unexpectedly and exc_type in (
            None,
            exceptions.LambkinSIGUSR1Interrupt,
        ):
            returncode = self._proc.poll()
            raise exceptions.LambkinProcessDiedUnexpectedlyError(
                self._argv, returncode or 1
            )


def _normalize_measure(measure: str | Sequence[str] | None) -> tuple[str, ...]:
    """Normalize the ``measure`` argument to a tuple of process names.

    Args:
        measure: A single process name, a sequence of names, or None.

    Returns:
        The process names, empty if measurement was not requested.

    Raises:
        TypeError: If measure is neither a string nor a sequence of strings.
        ValueError: If any process name is empty or blank.
    """
    if measure is None:
        return ()
    names = (measure,) if isinstance(measure, str) else tuple(measure)
    for name in names:
        if not isinstance(name, str):
            raise TypeError(
                f"measure must be a string or a sequence of strings, got {name!r}."
            )
        if not name.strip():
            raise ValueError("measure process names must not be empty.")
    return names


def background(proxy: CommandProxy, *args: Any, **kwargs: Any) -> BackgroundProcess:
    """Create a BackgroundProcess context manager from a command proxy.

    Args:
        proxy (CommandProxy): A command proxy representing the command to run.
        *args: Positional arguments to append to the command.
        **kwargs: Keyword arguments to convert to --flag value pairs. The SDK
            consumes ``log_output``, ``measure`` and ``measure_interval``
            itself; they never reach the command. ``measure`` is validated by
            :func:`_normalize_measure`, which rejects non-string names with a
            TypeError.

    Returns:
        BackgroundProcess: A context manager that runs the command in the background.

    Raises:
        ValueError: If the proxy has no cgroup set, or a measure name is blank.

    Example:
        with background(ctx.shell.ros2.bag.record, "-O", "output.mcap", "-a"):
        ...

        with background(
            ctx.shell.ros2.launch, "pkg", "slam.launch.py", measure="slam_node"
        ):
        ...
    """
    per_call_log_output = kwargs.pop("log_output", None)
    # Popped before build_argv, or they would be forwarded to the command as
    # --measure flags.
    measure = _normalize_measure(kwargs.pop("measure", None))
    measure_interval = kwargs.pop("measure_interval", None)
    argv = proxy.build_argv(*args, **kwargs)
    env = proxy.build_env()
    stdout, stderr = proxy.open_streams(per_call_log_output)
    cgroup = proxy.get_cgroup()
    if cgroup is None:
        raise ValueError("background() requires a proxy with a cgroup set.")
    return BackgroundProcess(
        argv=argv,
        iteration_cgroup=cgroup,
        cwd=proxy.get_cwd(),
        dry_run=proxy.get_dry_run(),
        env=env,
        stdout=stdout,
        stderr=stderr,
        measure=measure,
        measure_interval=measure_interval,
    )
