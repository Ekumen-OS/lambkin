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
import os
import shlex
import subprocess
from pathlib import Path
from typing import IO

from lambkin.common import defaults, exceptions
from lambkin.core.process.cgroup import (
    destroy_cgroup,
    kill_cgroup_tree,
    spawn_in_cgroup,
    start_process,
)
from lambkin.core.process.resources import pids_in_cgroup, read_cmdline

logger = logging.getLogger(__name__)


def run_foreground(
    argv: list[str],
    parent_cgroup: Path | None,
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    stdout: IO[str] | None = None,
    stderr: IO[str] | None = None,
) -> int | None:
    """Run a command in its own cgroup and wait for it to finish.

    A CommandError from starting the command, for example because it is not
    installed or could not join its cgroup, propagates to the caller.

    Args:
        argv: The command to run as a list of tokens.
        parent_cgroup: The iteration cgroup the command's cgroup goes under,
            or None to run in the caller's cgroup.
        cwd: Working directory for the command.
        env: Environment for the command, or None to inherit.
        stdout: stdout stream passed to subprocess.Popen.
        stderr: stderr stream passed to subprocess.Popen.

    Returns:
        The command's return code, or None if a ``LambkinSIGUSR1Interrupt``
        arrived while it ran or while it was cleaned up.
    """
    proc: subprocess.Popen[bytes] | None = None
    cgroup: Path | None = None
    interrupted = False
    try:
        if parent_cgroup is None:
            proc = start_process(argv, cwd=cwd, env=env, stdout=stdout, stderr=stderr)
        else:
            proc, cgroup = spawn_in_cgroup(
                parent_cgroup, argv, cwd=cwd, env=env, stdout=stdout, stderr=stderr
            )
        proc.wait()
    except exceptions.LambkinSIGUSR1Interrupt:
        interrupted = True
    finally:
        interrupted = _finish(argv, proc, cgroup) or interrupted
    if interrupted:
        return None
    assert proc is not None
    return proc.returncode


def _release_cgroup(argv: list[str], cgroup: Path, *, retry: bool = False) -> None:
    """Kill what a finished foreground command left behind and remove its cgroup.

    Processes still in the cgroup, or in cgroups the command created
    below it, after the command returned are orphans, for example
    something it daemonised. They are logged as a warning before being
    terminated.

    Args:
        argv: The command that ran, for the warning message.
        cgroup: The command's cgroup.
        retry: True if an earlier call was interrupted after it had
            started. The orphans were then already reported and sent
            SIGTERM, so they are not reported again and are killed
            without a second grace period.
    """
    try:
        orphans = [
            pid for root, _, _ in os.walk(cgroup) for pid in pids_in_cgroup(Path(root))
        ]
        if orphans and not retry:
            logger.warning(
                "%s left %d process(es) running after it returned; "
                "terminating them: %s",
                shlex.join(argv),
                len(orphans),
                "; ".join(f"{pid} ({_cmdline(pid)})" for pid in orphans),
            )
        destroy_cgroup(
            cgroup,
            grace_period=0.0 if retry else defaults.SIGTERM_GRACE_PERIOD,
        )
    except OSError as e:
        # Best effort: this runs in a finally block, so raising here would
        # hide the command's own result. The iteration teardown removes
        # whatever is left.
        logger.warning("Could not clean up cgroup %s: %s", cgroup, e)


def _finish(
    argv: list[str],
    proc: subprocess.Popen[bytes] | None,
    cgroup: Path | None,
) -> bool:
    """Stop a foreground command if it still runs and release its cgroup.

    A background process that dies meanwhile raises
    ``LambkinSIGUSR1Interrupt`` wherever this happens to be, for example
    while an orphan gets its SIGTERM grace period. Every step can be
    repeated, so cleanup simply starts over until it completes; orphans
    are reported and given their SIGTERM grace period only once.

    Args:
        argv: The command that ran.
        proc: The command's process, or None if it never started.
        cgroup: The command's cgroup, or None if it has none.

    Returns:
        True if a ``LambkinSIGUSR1Interrupt`` arrived during cleanup.
    """
    interrupted = False
    release_started = False
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
                retry = release_started
                release_started = True
                _release_cgroup(argv, cgroup, retry=retry)
            return interrupted
        except exceptions.LambkinSIGUSR1Interrupt:
            interrupted = True


def _cmdline(pid: int) -> str:
    """Return a process' command line for log messages, best effort.

    Args:
        pid: The process ID.

    Returns:
        The space-joined command line, or "?" if it cannot be read.
    """
    argv = read_cmdline(pid, Path("/proc"))
    return " ".join(argv) if argv else "?"
