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

"""Blocking process execution for lambkin benchmarks.

Runs a command in its own cgroup under the iteration cgroup and waits for it.
When the command returns, whatever it left behind is logged and killed, and
its cgroup is removed.
"""

from __future__ import annotations

import logging
import shlex
import subprocess
from pathlib import Path
from typing import IO

from lambkin.common import defaults, signals
from lambkin.core.process.cgroup import (
    destroy_cgroup,
    kill_cgroup_tree,
    pids_in_tree,
    spawn_in_cgroup,
    start_process,
)
from lambkin.core.process.resources import read_cmdline

logger = logging.getLogger(__name__)


def run_foreground(
    argv: list[str],
    parent_cgroup: Path | None,
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    stdout: IO[str] | None = None,
    stderr: IO[str] | None = None,
) -> int:
    """Run a command in its own cgroup and wait for it to finish.

    Only the wait can be interrupted by a background process dying. Starting
    the command and cleaning up after it always run to completion; a death
    meanwhile is raised once they are done.

    Args:
        argv: The command to run as a list of tokens.
        parent_cgroup: The iteration cgroup the command's cgroup goes under,
            or None to run in the caller's cgroup.
        cwd: Working directory for the command.
        env: Environment for the command, or None to inherit.
        stdout: stdout stream passed to subprocess.Popen.
        stderr: stderr stream passed to subprocess.Popen.

    Returns:
        The command's return code.

    Raises:
        CommandError: If the command could not be started, for example
            because it is not installed or could not join its cgroup.
        LambkinSIGUSR1Interrupt: If a background process died before or
            while the command ran. A death recorded earlier, whose interrupt
            was already used up, raises before the command starts.
    """  # noqa: DOC502 - raised by signals, start_process and spawn_in_cgroup
    signals.raise_if_died()
    with signals.deferred():
        cgroup: Path | None = None
        if parent_cgroup is None:
            proc = start_process(argv, cwd=cwd, env=env, stdout=stdout, stderr=stderr)
        else:
            proc, cgroup = spawn_in_cgroup(
                parent_cgroup, argv, cwd=cwd, env=env, stdout=stdout, stderr=stderr
            )
        try:
            with signals.interruptible():
                proc.wait()
        finally:
            _finish(argv, proc, cgroup)
    return proc.returncode


def _release_cgroup(argv: list[str], cgroup: Path) -> None:
    """Kill what a finished foreground command left behind and remove its cgroup.

    Processes still in the cgroup, or in cgroups the command created
    below it, after the command returned are orphans, for example
    something it daemonised. They are logged as a warning before being
    terminated.

    Args:
        argv: The command that ran, for the warning message.
        cgroup: The command's cgroup.
    """
    try:
        orphans = pids_in_tree(cgroup)
        if orphans:
            logger.warning(
                "%s left %d process(es) running after it returned; "
                "terminating them: %s",
                shlex.join(argv),
                len(orphans),
                "; ".join(f"{pid} ({_cmdline(pid)})" for pid in orphans),
            )
        destroy_cgroup(cgroup, grace_period=defaults.SIGTERM_GRACE_PERIOD)
    except OSError as e:
        # Best effort: this runs in a finally block, so raising here would
        # hide the command's own result. The iteration teardown removes
        # whatever is left.
        logger.warning("Could not clean up cgroup %s: %s", cgroup, e)


def _finish(
    argv: list[str],
    proc: subprocess.Popen[bytes],
    cgroup: Path | None,
) -> None:
    """Stop a foreground command if it still runs and release its cgroup.

    Args:
        argv: The command that ran.
        proc: The command's process.
        cgroup: The command's cgroup, or None if it has none.
    """
    if proc.poll() is None:
        # Interrupted while the command still runs: kill its whole tree at
        # once, not just the direct child.
        if cgroup is not None:
            kill_cgroup_tree(cgroup)
        else:
            proc.kill()
        proc.wait()
    if cgroup is not None:
        _release_cgroup(argv, cgroup)


def _cmdline(pid: int) -> str:
    """Return a process' command line for log messages, best effort.

    Args:
        pid: The process ID.

    Returns:
        The space-joined command line, or "?" if it cannot be read.
    """
    argv = read_cmdline(pid, Path("/proc"))
    return " ".join(argv) if argv else "?"
