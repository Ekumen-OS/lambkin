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

When invoked, it executes the given benchmark script under the current
user's delegated cgroup v2 scope, ensuring all child processes are
tracked and cleaned up automatically.

Typical usage:

    lambkin my_benchmark.py --clock-rate 50 --dry-run
"""

import os
import signal
import subprocess
import sys
import uuid
from pathlib import Path

import click
from click.formatting import HelpFormatter

from lambkin.core.process.cgroup import (
    find_app_slice,
    find_delegated_cgroup,
    kill_and_remove_cgroup_tree,
    make_cgroup,
)
from lambkin.sdk_options import SDK_OPTIONS


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
            "It runs your benchmark script inside a systemd cgroup scope, "
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


@click.command(
    cls=LambkinCommand,
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
)
@click.argument("script", type=click.Path(exists=True, path_type=Path))
@click.argument("args", nargs=-1, type=click.UNPROCESSED)
def main(script: Path, args: tuple) -> None:
    """Launch a lambkin benchmark script.

    Executes ``script`` with the same Python interpreter inside a dedicated
    cgroup v2 child scope created under the current user's delegated cgroup.
    This guarantees that the full process tree can be killed cleanly on
    interruption without affecting the rest of the user session.

    On Ctrl-C, all processes in the benchmark cgroup are terminated and the
    cgroup tree is removed before exiting.

    Exit codes:
        0    The benchmark script completed successfully.
        130  The benchmark was interrupted via Ctrl-C (SIGINT).
        N    Any other return code is propagated from the benchmark script.
    """
    # TODO(teresa-ortega): Handle concurrent runs, interrupted benchmarks, and re-runs
    # (e.g. detect an already active scope, support partial restarts).
    # To be addressed in phase 6.
    parent = find_app_slice() or find_delegated_cgroup()
    run_cgroup = make_cgroup(parent, f"lambkin-{script.stem}-{uuid.uuid4().hex[:8]}")

    def _handle_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle_sigint)
    proc = subprocess.Popen(
        [sys.executable, str(script), *args],
        start_new_session=True,
        preexec_fn=lambda: (run_cgroup / "cgroup.procs").write_text(str(os.getpid())),
    )
    try:
        proc.wait()
    except KeyboardInterrupt:
        kill_and_remove_cgroup_tree(run_cgroup)
        sys.exit(130)

    sys.exit(proc.returncode)
