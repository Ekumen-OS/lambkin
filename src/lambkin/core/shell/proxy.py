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

import shlex
import subprocess
from pathlib import Path
from typing import Any

from lambkin.common import defaults


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
    dispatches it to the operating system via subprocess, or prints it in
    dry-run mode.

    This class is not meant to be instantiated directly. Use ShellProxy to
    obtain the first proxy in a chain.
    """

    def __init__(
        self,
        parts: list[str],
        dry_run: bool = False,
        cwd: Path | None = None,
        cgroup: Path | None = None,
        benchmark_log_output: str | None = None,
    ) -> None:
        """Initialize the proxy with the command tokens accumulated so far.

        Args:
        parts: The list of command tokens accumulated so far.
        dry_run: If True, commands are printed instead of executed.
        cwd: Working directory for the command when dispatched.
        cgroup: Iteration cgroup directory for background processes.
        benchmark_log_output: Log output mode set via CLI. Overrides any
        per-call log_output argument. None means no CLI override was
        provided.
        """
        self._parts = parts
        self._dry_run = dry_run
        self._cwd = cwd
        self._cgroup = cgroup
        self._benchmark_log_output = benchmark_log_output

    def __getattr__(self, name: str) -> CommandProxy:
        """Append a new token to the command and return a new proxy.

        This allows chaining attribute access to build multi-word commands.
        For example, shell.ros2.topic.list() builds ['ros2', 'topic', 'list'].

        Args:
            name: The token to append to the command.

        Returns:
            A new proxy with the token appended.
        """
        return CommandProxy(
            self._parts + [name],
            self._dry_run,
            self._cwd,
            self._cgroup,
            self._benchmark_log_output,
        )

    def _resolve_log_output(self, per_call: str | None) -> str:
        print(f"[DEBUG] benchmark_log_output: {self._benchmark_log_output}")
        print(f"[DEBUG] per_call: {per_call}")
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
        kwargs.pop("log_output", None)
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

        In dry-run mode, prints the command and returns None. In real mode,
        runs the command as a foreground process, blocking until it completes.
        The process inherits stdout and stderr from the parent, so its output
        goes directly to the terminal.

        Args:
            *args: Positional arguments appended as tokens to the command.
            **kwargs: Keyword arguments converted to --flag value pairs.

        Returns:
            The CompletedProcess instance returned by subprocess.run, or a
            dummy CompletedProcess(argv, returncode=0) in dry-run mode.

        Raises:
            CommandError: If the process exits with a non-zero return code.
        """
        log_output = kwargs.pop("log_output", None)
        log_output = self._resolve_log_output(log_output)
        print(f"[DEBUG] log_output resuelto: {log_output}")
        argv = self._build_argv(*args, **kwargs)
        if self._dry_run:
            print(f"[DRY RUN] {shlex.join(argv)}")
            return subprocess.CompletedProcess(argv, returncode=0)
        try:
            # TODO(teresa-ortega): subprocess.run inherits stdout/stderr from
            # the parent process, so all output goes directly to the terminal
            # with no way to capture, redirect, or log it. When logging is
            # revisited, consider switching to subprocess.Popen for full control
            # over stdout/stderr streams.
            proc = subprocess.Popen(
                argv,
                cwd=self._cwd,
            )
            proc.wait()
            if proc.returncode != 0:
                raise subprocess.CalledProcessError(proc.returncode, argv)
            return subprocess.CompletedProcess(argv, returncode=proc.returncode)
        except subprocess.CalledProcessError as e:
            raise CommandError(
                argv,
                f"Command {shlex.join(argv)!r} failed with return code {e.returncode}.",
                returncode=e.returncode,
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


class ShellProxy:
    """Shell proxy that dispatches commands to the operating system.

    Attribute access on this object starts building a command. Each subsequent
    attribute access appends a token. Calling the result dispatches the command.

    Example:
        shell = ShellProxy()
        shell.ros2.topic.list()          # runs: ros2 topic list
        shell.echo("hello", "world")     # runs: echo hello world
        shell.my_tool(verbose=True)      # runs: my_tool --verbose

    In dry-run mode, commands are printed instead of executed, which is useful
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
            dry_run: If True, commands are printed instead of executed.
            cwd: Working directory for all commands dispatched through this proxy.
            cgroup: Iteration cgroup directory for background processes.
            log_output: Log output mode set via CLI. Overrides any per-call
            log_output argument. None means no CLI override was provided.
        """
        self._dry_run = dry_run
        self._cwd = cwd
        self._cgroup = cgroup
        self._log_output = log_output

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
        )
