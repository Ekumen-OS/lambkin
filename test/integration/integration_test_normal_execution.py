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
"""Integration test: normal execution with two nested background processes."""

import shutil
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, "src")

from lambkin.core.process.background import background
from lambkin.core.process.cgroup import (
    cgroup_exists,
    find_delegated_cgroup,
    make_iteration_cgroup,
)
from lambkin.core.shell.proxy import ShellProxy

COOPERATIVE = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "time.sleep(30)"
)


def main():
    """Run integration test for normal execution with two nested background processes.

    Verifies that both background processes start correctly, the foreground
    runs to completion, and both background processes are terminated in
    reverse order with their cgroups cleaned up.
    """
    iteration_dir = Path(tempfile.mkdtemp())
    try:
        cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

        shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)

        print("Starting outer background process...")
        with background(shell.python3, "-c", COOPERATIVE) as bp1:
            print(f"Outer process started — pid={bp1._proc.pid}")
            print("Starting inner background process...")
            cgroup1 = bp1._cgroup
            with background(shell.python3, "-c", COOPERATIVE) as bp2:
                print(f"Inner process started — pid={bp2._proc.pid}")
                print("Foreground running for 1s...")
                cgroup2 = bp2._cgroup
                time.sleep(1)
                print("Foreground done.")
            assert not cgroup_exists(cgroup2), "Inner cgroup should have been removed"
            print(f"Inner process dead: {bp2._proc.poll() is not None}")
        print(f"Outer process dead: {bp1._proc.poll() is not None}")
        assert not cgroup_exists(cgroup1), "Outer cgroup should have been removed"
        print("Normal execution test passed")
    finally:
        shutil.rmtree(iteration_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
