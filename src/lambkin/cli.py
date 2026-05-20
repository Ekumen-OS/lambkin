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

Exposes the ``lambkin`` command, registered as a console script in
``pyproject.toml``. When invoked, it re-executes the given benchmark
script under a transient systemd scope so that all child processes
are placed in a dedicated cgroup v2 hierarchy automatically.

Typical usage::

    lambkin my_benchmark.py --clock-rate 50 --dry-run
"""

import subprocess
import sys
from pathlib import Path

import click
from click.formatting import HelpFormatter

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
    """Launch a lambkin benchmark script inside a systemd cgroup scope.

    Re-executes ``script`` with the same Python interpreter, wrapped in
    ``systemd-run --scope`` so that every child process spawned during
    the benchmark (ROS 2 nodes, bag players, etc.) is placed inside a
    dedicated transient cgroup v2 scope.  This guarantees that the full
    process tree can be inspected and killed cleanly without leaving
    orphaned processes behind.

    The cgroup scope is named ``lambkin-<stem>.scope``, where ``<stem>``
    is the filename of the script without its extension.  All arguments
    that follow the script path are forwarded to the child process
    unchanged, so SDK and user-defined CLI options (e.g. ``--dry-run``,
    ``--clock-rate``) are passed through transparently.

    Exit codes:
        0    The benchmark script completed successfully.
        1    An error occurred (e.g. systemd-run not found).
        130  The benchmark was interrupted via Ctrl-C (SIGINT).
        N    Any other return code is propagated from the benchmark script.
    """
    # TODO(teresa-ortega): Handle concurrent runs, interrupted benchmarks, and re-runs
    # (e.g. detect an already active scope, support partial restarts).
    # To be addressed in phase 6.
    cgroup_scope = f"lambkin-{script.stem}.scope"

    try:
        result = subprocess.run(
            [
                "systemd-run",
                "--scope",
                f"--unit={cgroup_scope}",
                "--user",
                sys.executable,
                str(script),
                *args,
            ],
        )
    except KeyboardInterrupt:
        subprocess.run(
            ["systemctl", "--user", "stop", "--wait", cgroup_scope],
            capture_output=True,
        )
        sys.exit(130)
    except FileNotFoundError as err:
        raise click.ClickException(
            "'systemd-run' not found. lambkin requires systemd."
        ) from err

    sys.exit(result.returncode)
