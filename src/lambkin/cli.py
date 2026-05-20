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

When invoked, it re-executes the given benchmark script under a
transient systemd scope so that all child processes are placed in a
dedicated cgroup v2 hierarchy automatically.

Typical usage:

    lambkin my_benchmark.py --clock-rate 50 --dry-run
"""

import importlib.util
import signal
import subprocess
import sys
from pathlib import Path

import click
from click.formatting import HelpFormatter

from lambkin.common import exceptions
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


def _show_options(script: Path) -> None:
    """Load the benchmark script and print all registered @lambkin.option entries."""
    spec = importlib.util.spec_from_file_location("_lambkin_user_script", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    options = []
    for name in dir(module):
        obj = getattr(module, name)
        if callable(obj) and hasattr(obj, "__lambkin_options__"):
            options.extend(obj.__lambkin_options__)

    formatter = HelpFormatter()
    if not options:
        formatter.write_text("No options registered in this script.")
    else:
        with formatter.section("Custom Options"):
            formatter.write_dl(
                [
                    (
                        opt.opts[0],
                        (opt.help or "")
                        + (
                            f"  [default: {opt.default}]"
                            if opt.default is not None
                            else ""
                        ),
                    )
                    for opt in options
                ]
            )
    click.echo(formatter.getvalue(), nl=False)


def _stop_scope(cgroup_scope: str) -> None:
    """Stop a systemd cgroup scope, waiting up to 30 seconds for it to terminate.

    Parameters
    ----------
    cgroup_scope : str
        The name of the systemd scope to stop.
    """
    try:
        subprocess.run(
            ["systemctl", "--user", "stop", cgroup_scope],
            capture_output=True,
            timeout=30,
        )
    except subprocess.TimeoutExpired as err:
        raise exceptions.LambkinSystemdScopeTimeoutError(
            f"Timed out waiting for scope '{cgroup_scope}' to stop. "
            "Some processes may still be running."
        ) from err


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
    if "--show-options" in args:
        _show_options(script)
        sys.exit(0)

    cgroup_scope = f"lambkin-{script.stem}.scope"

    def _handle_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle_sigint)
    try:
        proc = subprocess.Popen(
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
        proc.wait()
    except KeyboardInterrupt:
        _stop_scope(cgroup_scope)
        sys.exit(130)
    except FileNotFoundError as err:
        raise exceptions.LambkinSystemdNotFoundError(
            "'systemd-run' not found. lambkin requires systemd."
        ) from err

    sys.exit(proc.returncode)
