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


def main() -> None:
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

    Exits with the same return code as the child process.

    Raises:
    ------
    SystemExit
        Always — propagates the child process return code.
    """
    # TODO(teresa-ortega): Handle concurrent runs, interrupted benchmarks, and re-runs
    # (e.g. detect an already active scope, support partial restarts).
    # To be addressed in phase 6.
    if len(sys.argv) < 2:
        print("Usage: lambkin <script.py> [args...]", file=sys.stderr)
        sys.exit(1)

    script = Path(sys.argv[1]).resolve()
    if not script.exists():
        print(f"Error: script not found: {script}", file=sys.stderr)
        sys.exit(1)

    args = sys.argv[2:]

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
            ["systemctl", "--user", "stop", cgroup_scope],
            capture_output=True,
        )
        sys.exit(130)
    except FileNotFoundError:
        print(
            "Error: 'systemd-run' not found. lambkin requires systemd.",
            file=sys.stderr,
        )
        sys.exit(127)

    sys.exit(result.returncode)
