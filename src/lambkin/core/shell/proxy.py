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

"""Shell proxy for lambkin benchmark execution.

Provides an abstraction over shell command dispatch, allowing benchmark
processes to be launched and managed through a consistent interface.
"""

from __future__ import annotations

import functools
import logging
import shlex
import subprocess
from collections.abc import Callable
from pathlib import Path
from typing import Any, Final

from lambkin.common import defaults, exceptions
from lambkin.core.process.cgroup import (
    cgroup_joiner,
    kill_cgroup,
    kill_cgroup_tree,
    make_process_cgroup,
    remove_cgroup,
)
from lambkin.core.process.resources import pids_in_cgroup, read_cmdline

logger = logging.getLogger(__name__)

# Keyword arguments the SDK consumes itself. They configure how a command is
# run, never what is passed to it, so build_argv strips them from the argv.
_SDK_KWARGS: Final[frozenset[str]] = frozenset({
    "log_output",
    "measure",
    "measure_interval",
    "flamegraph",
})


class CommandError(Exception):
    """Raised when a shell command cannot be started or exits with an error.

    Attributes:
        command:    The argv list passed to the operating system.
        returncode: Exit code of the process, or None if it never started.
    """

    def __init__(
        self,
        command: list[str],
        message: str,
        returncode: int | None = None,
    ) -> None:
        """Initialize the error with the command that failed and its cause.

        Args:
            command:    The argv list that was passed to the operating system.
            message:    Human-readable description of the failure.
            returncode: The exit code returned by the process, or None if it
                        never started.
        """
        self.command = command
        self.returncode = returncode
        super().__init__(message)


class CommandProxy:
    """Builds a shell command lazily by chaining attribute access and calls.

    Each attribute access appends a new token to the command being constructed
    and returns a new proxy. Calling the proxy finalises the command and
    dispatches it to the operating system via subprocess, or logs it in
    dry-run mode.

    This class is not meant to be instantiated directly. Use ShellProxy to
    obtain the first proxy in a chain.
    """

    _specialisations: dict[tuple[str, ...], type] = {}

    def __init__(
        self,
        parts: list[str],
        dry_run: bool = False,
        cwd: Path | None = None,
        cgroup: Path | None = None,
        benchmark_log_output: str | None = None,
        call_counts: dict[str, int] | None = None,
    ) -> None:
        """Initialize the proxy with the command tokens accumulated so far.

        Args:
            parts: The list of command tokens accumulated so far.
            dry_run: If True, commands are logged instead of executed.
            cwd: Working directory for the command when dispatched.
            cgroup: Iteration cgroup directory. Every command, foreground or
                background, runs in its own child cgroup under it.
            benchmark_log_output: Log output mode set via CLI. Overrides any
                per-call log_output argument. None means no CLI override was
                provided.
            call_counts: Shared dictionary tracking how many times each command
                has been launched in the current iteration, used to append
                numeric suffixes to log file names to avoid collisions.
        """
        self._parts = parts
        self._dry_run = dry_run
        self._cwd = cwd
        self._cgroup = cgroup
        self._benchmark_log_output = benchmark_log_output
        self._call_counts = call_counts or {}

    @classmethod
    def register(cls, parts: tuple[str, ...], proxy_cls: type) -> None:
        """Register a specialised proxy class for a given command prefix.

        Args:
            parts: The command tokens that trigger the specialisation.
            proxy_cls: The proxy class to use for that command.
        """
        cls._specialisations[parts] = proxy_cls

    def __getattr__(self, name: str) -> CommandProxy:
        """Append a new token to the command and return a new proxy.

        This allows chaining attribute access to build multi-word commands.
        For example, shell.ros2.topic.list() builds ['ros2', 'topic', 'list'].

        Args:
            name: The token to append to the command.

        Returns:
            A new proxy with the token appended.
        """
        parts = tuple(self._parts + [name])
        proxy_cls = self._specialisations.get(parts, CommandProxy)
        return proxy_cls(
            list(parts),
            self._dry_run,
            self._cwd,
            self._cgroup,
            self._benchmark_log_output,
            self._call_counts,
        )

    def _resolve_log_output(self, per_call: str | None) -> str:
        """Resolve the effective log output mode following precedence rules.

        The resolution order from highest to lowest priority is:
        benchmark-level option set via CLI, per-call override, default value.

        Args:
            per_call: Log output mode passed at the call site, or None if not provided.

        Returns:
            The resolved log output mode.
        """
        if self._benchmark_log_output is not None:
            return self._benchmark_log_output
        if per_call is not None:
            return per_call
        return defaults.LOG_OUTPUT

    def get_cgroup(self) -> Path | None:
        """Return the iteration cgroup directory."""
        return self._cgroup

    def get_cwd(self) -> Path | None:
        """Return the working directory."""
        return self._cwd

    def get_dry_run(self) -> bool:
        """Return the dry run flag."""
        return self._dry_run

    def _make_popen(
        self,
        argv: list[str],
        stdout,
        stderr,
        cgroup: Path | None = None,
    ) -> subprocess.Popen:
        """Create and return a subprocess with the given streams.

        Args:
            argv: The command to run as a list of tokens.
            stdout: stdout stream configuration passed to subprocess.Popen.
            stderr: stderr stream configuration passed to subprocess.Popen.
            cgroup: Cgroup the child joins before exec(), or None to stay in
                the caller's cgroup.

        Returns:
            The running subprocess.
        """
        preexec = None
        if cgroup is not None:
            preexec = functools.partial(self._prepare_child, cgroup_joiner(cgroup))
        return subprocess.Popen(
            argv,
            cwd=self._cwd,
            stdout=stdout,
            stderr=stderr,
            env=self.build_env(),
            preexec_fn=preexec,
        )

    def _prepare_child(self, join_cgroup: Callable[[], None]) -> None:
        """Set up a foreground child between fork() and exec().

        Runs in the child process, so it must stay async-signal-safe in
        spirit: no logging, no locks, nothing computed here that the parent
        could prepare. This is the single place to add more per-child setup
        steps.

        Args:
            join_cgroup: Moves the child into the command's own cgroup, from
                ``cgroup_joiner``.
        """
        join_cgroup()

    def _make_cgroup(self, parent: Path, argv: list[str]) -> Path:
        """Create the cgroup a foreground command runs in.

        Args:
            parent: The iteration cgroup the command's cgroup goes under.
            argv: The command about to run, used to name the cgroup.

        Returns:
            The new cgroup directory.

        Raises:
            CommandError: If the cgroup cannot be created, for example because
                the iteration cgroup is gone. Raised as its own error so it is
                not reported as a problem with the command itself.
        """
        try:
            return make_process_cgroup(parent, argv)
        except OSError as e:
            raise CommandError(
                argv,
                f"Could not create a cgroup for {argv[0]!r} under {parent}: {e}",
            ) from e

    def _release_cgroup(self, argv: list[str], cgroup: Path) -> None:
        """Kill what a finished foreground command left behind and remove its cgroup.

        Processes still in the cgroup after the command returned are orphans,
        for example something it daemonised. They are logged as a warning
        before being terminated.

        Args:
            argv: The command that ran, for the warning message.
            cgroup: The command's cgroup.
        """
        orphans = pids_in_cgroup(cgroup)
        try:
            if orphans:
                logger.warning(
                    "%s left %d process(es) running after it returned; "
                    "terminating them: %s",
                    shlex.join(argv),
                    len(orphans),
                    "; ".join(f"{pid} ({_cmdline(pid)})" for pid in orphans),
                )
                kill_cgroup(cgroup, grace_period=defaults.SIGTERM_GRACE_PERIOD)
            remove_cgroup(cgroup)
        except OSError as e:
            # Best effort: this runs in a finally block, so raising here would
            # hide the command's own result. The iteration teardown removes
            # whatever is left.
            logger.warning("Could not clean up cgroup %s: %s", cgroup, e)

    def _finish(
        self,
        argv: list[str],
        proc: subprocess.Popen[bytes] | None,
        cgroup: Path | None,
    ) -> bool:
        """Stop a foreground command if it still runs and release its cgroup.

        A background process that dies meanwhile raises
        ``LambkinSIGUSR1Interrupt`` wherever this happens to be, for example
        while an orphan gets its SIGTERM grace period. Every step can be
        repeated, so cleanup simply starts over until it completes.

        Args:
            argv: The command that ran.
            proc: The command's process, or None if it never started.
            cgroup: The command's cgroup, or None if it has none.

        Returns:
            True if a ``LambkinSIGUSR1Interrupt`` arrived during cleanup.
        """
        interrupted = False
        while True:
            try:
                if proc is not None and proc.poll() is None:
                    # Interrupted while the command still runs: kill its whole
                    # tree at once, not just the direct child.
                    if cgroup is not None:
                        kill_cgroup_tree(cgroup)
                    else:
                        proc.kill()
                    proc.wait()
                if cgroup is not None:
                    self._release_cgroup(argv, cgroup)
                return interrupted
            except exceptions.LambkinSIGUSR1Interrupt:
                interrupted = True

    def open_streams(self, log_output: str | None = None) -> tuple:
        """Open log files for stdout and stderr in the iteration directory.

        Args:
            log_output: Per-call output mode override. If provided, takes
                precedence over the default but not over the CLI flag.
                Accepted values are 'file' and 'console'.

        Returns:
            A tuple of (stdout_file, stderr_file) open for writing, or
            (None, None) if the resolved log output mode is not 'file'.

        Raises:
            CommandError: If the resolved log output mode is 'file' but no
                working directory is set.
        """
        if self._resolve_log_output(log_output) != "file":
            return None, None
        if self._cwd is None:
            raise CommandError(
                self._parts,
                "log_output='file' requires a working directory to be set.",
            )
        base = self._log_base()
        out = open(self._cwd / f"{base}.stdout.log", "w")
        err = open(self._cwd / f"{base}.stderr.log", "w")
        return out, err

    def _log_name(self) -> str:
        """Derive a log file base name from the command parts.

        Joins the command tokens accumulated so far with underscores.
        Arguments passed at call time are not included, only the tokens
        that form the command itself (e.g. 'ros2_launch').

        Returns:
            A base name string, e.g. 'ros2_launch'.
        """
        return "_".join(self._parts)

    def _log_base(self) -> str:
        """Return a unique log file base name, appending a suffix on collision.

        Calls _log_name to derive the base from the command parts, then
        increments the counter for that name and appends a numeric suffix
        if the same command has been launched more than once in this iteration.

        Returns:
            A unique base name string, e.g. 'ros2_launch' or 'ros2_launch_1'.
        """
        base = self._log_name()
        count = self._call_counts.get(base, 0)
        self._call_counts[base] = count + 1
        suffix = f"_{count}" if count > 0 else ""
        return f"{base}{suffix}"

    def build_env(self) -> dict | None:
        """Return the environment for the child process, or None to inherit."""
        return None

    def build_argv(self, *args: Any, **kwargs: Any) -> list[str]:
        """Build the final argv list from positional and keyword arguments.

        Positional arguments are appended as discrete tokens. Keyword arguments
        are converted to --flag value pairs, with underscores replaced by
        hyphens. A boolean True value produces a standalone flag. A boolean
        False value is omitted entirely.

        This method is shared by __call__ and background() to avoid duplicating
        argv construction logic.

        Args:
            *args: Positional arguments to append as tokens.
            **kwargs: Keyword arguments to convert to --flag value pairs.

        Returns:
            The complete argv list ready to pass to the operating system.
        """
        for name in _SDK_KWARGS:
            kwargs.pop(name, None)
        extra: list[str] = []
        for arg in args:
            extra.append(str(arg))
        for key, value in kwargs.items():
            flag = "--" + key.replace("_", "-")
            if value is True:
                extra.append(flag)
            elif value is not False:
                extra.extend([flag, str(value)])
        return self._parts + extra

    def __call__(self, *args: Any, **kwargs: Any) -> subprocess.CompletedProcess:
        """Finalise the command and dispatch it to the operating system.

        In dry-run mode, logs the command and returns a dummy CompletedProcess.
        Otherwise, it runs the command as a foreground process, blocking until
        it completes. The process inherits stdout and stderr from the parent, so
        its output goes directly to the terminal.

        Args:
            *args: Positional arguments appended as tokens to the command.
            **kwargs: Keyword arguments converted to --flag value pairs.

        Returns:
            The CompletedProcess instance returned by subprocess.run, or a
            dummy CompletedProcess(argv, returncode=0) in dry-run mode.

        Raises:
            CommandError: If the process exits with a non-zero return code,
                or if the executable is not found or not executable.
        """
        per_call_log_output = kwargs.pop("log_output", None)
        if "measure" in kwargs:
            logger.warning(
                "measure= is only supported by background(); ignoring it for %s.",
                self._log_name(),
            )
        argv = self.build_argv(*args, **kwargs)
        if self._dry_run:
            logger.debug("[DRY RUN] %s", shlex.join(argv))
            return subprocess.CompletedProcess(argv, returncode=0)

        stdout, stderr = None, None
        proc: subprocess.Popen[bytes] | None = None
        cgroup: Path | None = None
        interrupted = False

        try:
            stdout, stderr = self.open_streams(per_call_log_output)
            if self._cgroup is not None:
                cgroup = self._make_cgroup(self._cgroup, argv)
            proc = self._make_popen(argv, stdout, stderr, cgroup)
            proc.wait()
        except exceptions.LambkinSIGUSR1Interrupt:
            interrupted = True
        except subprocess.SubprocessError as e:
            # Raised by Popen when _prepare_child fails in the child.
            raise CommandError(
                argv,
                f"Could not prepare {argv[0]!r} to run: {e}",
            ) from e
        # FileNotFoundError and PermissionError must come before OSError,
        # as they are subclasses of it. Order matters here.
        except FileNotFoundError:
            raise CommandError(
                argv,
                f"Command not found: {argv[0]!r}. "
                f"Make sure it is installed and available on PATH.",
            ) from None
        except PermissionError:
            raise CommandError(
                argv,
                f"Permission denied: {argv[0]!r} is not executable. "
                f"Check file permissions.",
            ) from None
        except OSError as e:
            raise CommandError(
                argv,
                f"OS error while starting {argv[0]!r}: {e}.",
            ) from e
        finally:
            try:
                if self._finish(argv, proc, cgroup):
                    interrupted = True
            finally:
                if stdout:
                    stdout.close()
                if stderr:
                    stderr.close()

        if interrupted:
            # dummy return — never observed, background.__exit__ raises
            # LambkinProcessDiedUnexpectedlyError immediately after
            return subprocess.CompletedProcess(argv, returncode=-1)

        assert proc is not None
        if not interrupted and proc.returncode != 0:
            raise CommandError(
                argv,
                f"Command {shlex.join(argv)!r} failed with return code"
                f" {proc.returncode}.",
                returncode=proc.returncode,
            )
        return subprocess.CompletedProcess(argv, returncode=proc.returncode)


def _cmdline(pid: int) -> str:
    """Return a process' command line for log messages, best effort.

    Args:
        pid: The process ID.

    Returns:
        The space-joined command line, or "?" if it cannot be read.
    """
    argv = read_cmdline(pid, Path("/proc"))
    return " ".join(argv) if argv else "?"


class ShellProxy:
    """Shell proxy that dispatches commands to the operating system.

    Attribute access on this object starts building a command. Each subsequent
    attribute access appends a token. Calling the result dispatches the command.

    Example:
        shell = ShellProxy()
        shell.ros2.topic.list()          # runs: ros2 topic list
        shell.echo("hello", "world")     # runs: echo hello world
        shell.my_tool(verbose=True)      # runs: my_tool --verbose

    In dry-run mode, commands are logged instead of executed, which is useful
    for testing and for recording what a benchmark would do without running it.
    """

    def __init__(
        self,
        dry_run: bool = False,
        cwd: Path | None = None,
        cgroup: Path | None = None,
        log_output: str | None = None,
    ) -> None:
        """Initialize the ShellProxy.

        Args:
            dry_run: If True, commands are logged instead of executed.
            cwd: Working directory for all commands dispatched through this proxy.
            cgroup: Iteration cgroup directory. Every command, foreground or
                background, runs in its own child cgroup under it.
            log_output: Log output mode set via CLI. Overrides any per-call
                log_output argument. None means no CLI override was provided.
        """
        self._dry_run = dry_run
        self._cwd = cwd
        self._cgroup = cgroup
        self._log_output = log_output
        self._call_counts: dict[str, int] = {}

    def __getattr__(self, name: str) -> CommandProxy:
        """Start building a new command from the given top-level token.

        Args:
            name: The first token of the command, typically the program name.

        Returns:
            A CommandProxy with the first token set.
        """
        return CommandProxy(
            [name],
            self._dry_run,
            self._cwd,
            self._cgroup,
            self._log_output,
            self._call_counts,
        )
