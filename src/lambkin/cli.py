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

"""Entry point for the lambkin CLI.

When invoked, it executes the given benchmark script inside a dedicated
cgroup v2 scope created under the user's app.slice (or the current
delegated cgroup as fallback), ensuring all child processes are tracked
and cleaned up automatically.

Typical usage:

lambkin my_benchmark.py --clock-rate 50 --dry-run
"""

import logging
import os
import signal
import subprocess
import sys
import termios
import uuid
from pathlib import Path
from types import FrameType
from typing import Any

import click
from click.formatting import HelpFormatter

from lambkin.core.process.cgroup import (
    find_delegated_cgroup,
    find_user_slice,
    kill_cgroup_tree,
    make_cgroup,
    remove_cgroup_tree,
)
from lambkin.logger import configure_logging
from lambkin.sdk_options import SDK_OPTIONS

logger = logging.getLogger(__name__)


class LambkinCommand(click.Command):
    """Custom Click command that renders SDK and custom options separately."""

    def format_help(self, ctx: click.Context, formatter: HelpFormatter) -> None:
        """Write the full help text with SDK and custom options sections."""
        formatter.write_paragraph()
        formatter.write_text(
            "Usage: lambkin [OPTIONS] SCRIPT [SDK_OPTIONS] [CUSTOM_OPTIONS]"
        )
        formatter.write_paragraph()
        formatter.write_text(
            "LAMBKIN is a benchmarking SDK for robotics applications. "
            "It runs your benchmark script inside a dedicated cgroup v2 scope, "
            "ensuring all child processes are tracked and cleaned up automatically."
        )
        formatter.write_paragraph()

        with formatter.section("Options"):
            formatter.write_dl([("--help", "Show this message and exit.")])

        with formatter.section("SDK Options (always available)"):
            formatter.write_dl([(opt.opts[0], opt.help or "") for opt in SDK_OPTIONS])

        with formatter.section("Custom Options (script-defined)"):
            formatter.write_text(
                "Options registered in your benchmark script via @lambkin.option."
            )
            formatter.write_text("Run 'lambkin SCRIPT --show-options' to list them.")


def _save_terminal_state() -> list[Any] | None:
    """Save the current terminal state for later restoration.

    Reads the terminal attributes from stdin using termios. If stdin is not
    a terminal (e.g. in CI or when stdin is redirected), returns None and
    the save is a no-op.

    Returns:
        The terminal attribute list as returned by termios.tcgetattr, or None
        if stdin is not a terminal.
    """
    if sys.stdin.isatty():
        return termios.tcgetattr(sys.stdin.fileno())
    return None


def _restore_terminal_state(state: list[Any] | None) -> None:
    """Restore stdin terminal attributes to a previously saved state.

    If state is None or stdin is no longer a terminal, this is a no-op.

    Args:
        state: Terminal attribute list to restore, or None.
    """
    if state is not None and sys.stdin.isatty():
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, state)


@click.command(
    cls=LambkinCommand,
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
    params=[next(opt for opt in SDK_OPTIONS if "--log-level" in opt.opts)],
)
@click.argument("script", type=click.Path(exists=True, path_type=Path))
@click.argument("args", nargs=-1, type=click.UNPROCESSED)
def main(script: Path, args: tuple[str, ...], log_level: str) -> None:
    """Launch a lambkin benchmark script.

    Executes ``script`` with the same Python interpreter inside a dedicated
    cgroup v2 child scope created under the current user's delegated cgroup.
    This guarantees that the full process tree can be killed cleanly on
    interruption without affecting the rest of the user session.

    The benchmark script is launched in a new session (setsid) so that
    Ctrl-C is delivered only to the CLI, which then kills the benchmark
    cgroup cleanly rather than letting SIGINT crash the script mid-run.

    On Ctrl-C, all processes in the benchmark cgroup are terminated and the
    cgroup tree is removed, and the terminal state is restored before exiting.

    Exit codes:
        0    The benchmark script completed successfully.
        130  The benchmark was interrupted via Ctrl-C (SIGINT).
        N    Any other return code is propagated from the benchmark script.
    """
    # TODO(teresa-ortega): Handle concurrent runs, interrupted benchmarks, and re-runs
    # (e.g. detect an already active scope, support partial restarts).
    # To be addressed in phase 6.
    configure_logging(log_level)

    # Prefer app.slice as the parent cgroup for a stable, predictable location
    # in the cgroup hierarchy. Fall back to the current delegated cgroup if
    # app.slice is not available (e.g. inside a container).
    parent = find_user_slice()
    if parent is None:
        logger.debug("app.slice not available, falling back to delegated cgroup")
        parent = find_delegated_cgroup()

    run_cgroup = make_cgroup(parent, f"lambkin-{script.stem}-{uuid.uuid4().hex[:8]}")
    logger.debug("run_cgroup: %s", run_cgroup)

    # Enable available controllers so all child cgroups (iteration, process)
    # inherit memory.current and cpu.stat for resource monitoring.
    # run_cgroup is empty at this point so subtree_control is writable.
    try:
        available = (run_cgroup / "cgroup.controllers").read_text().split()
        to_enable = " ".join(f"+{c}" for c in ("memory", "cpu", "io") if c in available)
        if to_enable:
            (run_cgroup / "cgroup.subtree_control").write_text(to_enable)
    except OSError as e:
        logger.warning(
            "Could not enable controllers in %s: %s. "
            "Resource metrics may be unavailable.",
            run_cgroup,
            e,
        )

    # Place the script process in a dedicated child cgroup, keeping run_cgroup
    # process-free so its subtree_control remains writable. Iteration cgroups
    # are siblings of script/ under run_cgroup, found via find_run_cgroup().
    script_cgroup = make_cgroup(run_cgroup, "script")

    # Save the terminal state before launching the benchmark script.
    # start_new_session=True detaches the script from the terminal's process
    # group, which can leave the terminal in a bad state when the script is
    # killed. We restore it after the script exits.
    terminal_state = _save_terminal_state()

    def _handle_sigint(signum: int, frame: FrameType | None) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle_sigint)

    # Launch the benchmark script in a new session so that Ctrl-C (SIGINT) is
    # delivered only to the CLI process, not to the script.
    # preexec_fn places the child process inside run_cgroup immediately after
    # fork() but before exec(), ensuring it is tracked from the start.
    proc = subprocess.Popen(
        [sys.executable, str(script), "--log-level", log_level, *args],
        start_new_session=True,
        preexec_fn=lambda: (script_cgroup / "cgroup.procs").write_text(
            str(os.getpid())
        ),
    )
    try:
        proc.wait()
    except KeyboardInterrupt:
        logger.warning("Interrupted, cleaning up benchmark processes...")
        kill_cgroup_tree(run_cgroup)
        remove_cgroup_tree(run_cgroup)
        _restore_terminal_state(terminal_state)
        sys.exit(130)

    _restore_terminal_state(terminal_state)
    remove_cgroup_tree(run_cgroup)
    sys.exit(proc.returncode)
