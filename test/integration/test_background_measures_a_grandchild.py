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

"""Integration test: measurement targets a grandchild of the background process."""

import subprocess
import sys
import time

import yaml

from lambkin.core.process.background import background
from lambkin.core.process.cgroup import find_delegated_cgroup, make_iteration_cgroup
from lambkin.core.shell.proxy import ShellProxy

# A launcher that execs a differently named child and waits for it, standing in
# for `ros2 launch` starting a node. The child is what we want measured.
LAUNCHER = (
    "import subprocess, sys; "
    "child = subprocess.Popen([sys.argv[1], '-c', sys.argv[2]]); "
    "child.wait()"
)

# The child burns CPU and holds memory so the measurement is non-zero.
CHILD = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "blob = bytearray(8 * 1024 * 1024); "
    "end = time.monotonic() + 30\n"
    "while time.monotonic() < end: pass"
)


def test_background_measures_a_grandchild(tmp_path):
    """A named grandchild of the background process is the one measured.

    This is the shape that motivates naming a process at all: a benchmark runs
    `ros2 launch`, and the node worth measuring is a grandchild of it. The
    launcher here stands in for `ros2 launch`, and `slam_node` for the node.
    """
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    # A distinctly named interpreter, so argv[0]'s basename identifies the
    # grandchild rather than the launcher. The name is longer than the 15
    # characters /proc/<pid>/comm would keep.
    node = tmp_path / "slam_node_longname"
    node.symlink_to(sys.executable)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with background(
        shell.python3,
        "-c",
        LAUNCHER,
        str(node),
        CHILD,
        measure="slam_node_longname",
        measure_interval=0.05,
    ):
        time.sleep(1.0)

    summary = yaml.safe_load(
        (iteration_dir / "slam_node_longname.resources.yaml").read_text()
    )
    assert summary["pid_found"] is True, "the grandchild was never located"
    assert summary["cpu_total_s"] > 0
    assert summary["peak_rss_mib"] > 0

    # The measured PID is the grandchild, not the background process itself.
    launcher_pids = subprocess.run(
        ["pgrep", "-f", LAUNCHER], capture_output=True, text=True, check=False
    ).stdout.split()
    assert str(summary["pid"]) not in launcher_pids
