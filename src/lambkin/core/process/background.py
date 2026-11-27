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
import shlex
import signal
import subprocess
import threading
from collections.abc import Sequence
from pathlib import Path
from types import TracebackType
from typing import IO, Any

from lambkin.common import defaults, exceptions, signals
from lambkin.core.process.cgroup import destroy_cgroup, spawn_in_cgroup
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
        flamegraph: bool = False,
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
            flamegraph (bool): If True, also profile the measured processes with
                perf and render a flamegraph SVG for each.
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
        self._flamegraph = flamegraph
        self._sampler: ResourceSampler | None = None

    def _monitor_process(self) -> None:
        """Monitor thread that detects if the process dies unexpectedly.

        If the process dies before the context manager exits, records the
        death and sends SIGUSR1 to the main thread to interrupt any blocking
        foreground process. BackgroundProcess.__exit__ is responsible for
        raising LambkinProcessDiedUnexpectedlyError.
        """
        assert self._proc is not None
        self._proc.wait()
        if not self._exiting.is_set():
            self._died_unexpectedly = True
            signals.notify_death(self)

    def __enter__(self) -> BackgroundProcess:
        """Start the background process inside its own cgroup.

        In dry-run mode, logs the command at debug level instead of executing it.

        Returns:
            BackgroundProcess: This instance.

        Raises:
            CommandError: If the command could not be started, for example
                because it is not installed or could not join its cgroup.
        """  # noqa: DOC502 - raised by spawn_in_cgroup
        if self._dry_run:
            logger.debug("[DRY RUN BG] %s", shlex.join(self._argv))
            return self

        try:
            self._proc, cgroup = spawn_in_cgroup(
                self._iteration_cgroup,
                self._argv,
                cwd=self._cwd,
                env=self._env,
                stdout=self._stdout,
                stderr=self._stderr,
            )
        except BaseException:
            # Nothing started, so __exit__ has nothing to stop: close the log
            # files here. spawn_in_cgroup already removed the cgroup.
            self._close_streams()
            raise
        self._cgroup = cgroup

        try:
            self._start_watchers(cgroup)
        except BaseException as e:
            # Python never calls __exit__ when __enter__ raises, so clean up
            # here. That includes the SIGUSR1 a process that died at once
            # sends while this is still running, which __exit__ turns into
            # LambkinProcessDiedUnexpectedlyError.
            self.__exit__(type(e), e, e.__traceback__)
            raise
        return self

    def _close_streams(self) -> None:
        """Close the process's log files, if any."""
        if self._stdout:
            self._stdout.close()
        if self._stderr:
            self._stderr.close()

    def _start_watchers(self, cgroup: Path) -> None:
        """Start the death monitor and, if requested, the resource sampler.

        Args:
            cgroup: The process's cgroup, which the sampler reads.
        """
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
                cgroup=cgroup,
                output_dir=self._cwd or Path.cwd(),
                interval=self._measure_interval,
                flamegraph=self._flamegraph,
            )
            self._sampler.start()

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

        Teardown runs inside ``signals.deferred()``, so another background
        process dying meanwhile does not cut it short.

        Raises:
            LambkinProcessDiedUnexpectedlyError: If the process died before this
                method was called and no other exception is already propagating.
            LambkinSIGUSR1Interrupt: If another background process died during
                teardown and no other exception is already propagating, which
                it must not replace. That process's own block reports it.
        """  # noqa: DOC502 - LambkinProcessDiedUnexpectedlyError via _report_death
        if self._dry_run:
            return
        # Another process dying meanwhile is raised only if nothing else
        # propagates. Otherwise its death stays recorded, and its own block
        # reports it against the exception propagating here.
        with signals.deferred(raise_on_exit=exc_type is None):
            err_type, err = exc_type, exc_val
            try:
                self._stop()
            except BaseException as e:
                # Report the death against the teardown error, which wins.
                err_type, err = type(e), e
                raise
            finally:
                # Even if teardown failed: a death left recorded would stop
                # every later foreground command, in later iterations too.
                signals.clear_death(self)
                if self._died_unexpectedly:
                    self._report_death(err_type, err)

    def _stop(self) -> None:
        """Stop the sampler and the process, and remove the process's cgroup."""
        self._close_streams()
        self._exiting.set()

        assert self._cgroup is not None
        try:
            try:
                # While the process is still alive: once it is signalled its
                # /proc entry is gone, taking VmHWM and the CPU counters with it.
                if self._sampler is not None:
                    self._sampler.stop()
            finally:
                destroy_cgroup(self._cgroup, grace_period=defaults.SIGTERM_GRACE_PERIOD)
        finally:
            if self._monitor is not None:
                # Once it returns, its death record, if any, is final. Bounded
                # in case the process outlived a failed teardown.
                self._monitor.join(timeout=defaults.SIGKILL_GRACE_PERIOD)

    def _report_death(
        self, exc_type: type[BaseException] | None, exc_val: BaseException | None
    ) -> None:
        """Raise or log that the process died before ``__exit__``.

        Args:
            exc_type: Type of the exception propagating through ``__exit__``.
            exc_val: The exception propagating through ``__exit__``.

        Raises:
            LambkinProcessDiedUnexpectedlyError: If no exception, or only the
                interrupt this death caused, is propagating.
        """
        assert self._proc is not None
        returncode = self._proc.poll() or 1
        if exc_type is None or exc_type is exceptions.LambkinSIGUSR1Interrupt:
            raise exceptions.LambkinProcessDiedUnexpectedlyError(self._argv, returncode)
        # Another error wins, but the death must leave a trace.
        logger.warning(
            "%s died unexpectedly (return code %d) while %s propagated.",
            shlex.join(self._argv),
            returncode,
            exc_type.__name__,
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
    flamegraph = bool(kwargs.pop("flamegraph", False))
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
        flamegraph=flamegraph,
    )
