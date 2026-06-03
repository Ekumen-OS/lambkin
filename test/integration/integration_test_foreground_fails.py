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

"""Integration test: foreground fails and background processes are cleaned up."""

import shutil
import sys
import tempfile
import time
from pathlib import Path

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
    """Run integration test for foreground failure with background processes running.

    Verifies that when the foreground raises an exception, both background
    processes are terminated cleanly and the original exception is propagated.

    Raises:
        RuntimeError: Intentionally raised to simulate a foreground failure.
    """
    iteration_dir = Path(tempfile.mkdtemp())
    try:
        cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

        shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)

        print("Starting outer background process...")
        try:
            with background(shell.python3, "-c", COOPERATIVE) as bp1:
                print(f"Outer process started — pid={bp1._proc.pid}")
                cgroup1 = bp1._cgroup
                with background(shell.python3, "-c", COOPERATIVE) as bp2:
                    print(f"Inner process started — pid={bp2._proc.pid}")
                    print("Foreground failing...")
                    cgroup2 = bp2._cgroup
                    time.sleep(0.5)
                    raise RuntimeError("Foreground failed intentionally")
        except RuntimeError as e:
            print(f"RuntimeError propagated correctly: {e}")
            print(f"Inner process dead: {bp2._proc.poll() is not None}")
            print(f"Outer process dead: {bp1._proc.poll() is not None}")
            assert not cgroup_exists(cgroup2), "Inner cgroup should have been removed"
            assert not cgroup_exists(cgroup1), "Outer cgroup should have been removed"
            print("Foreground failure test passed")
            return

        print("ERROR: RuntimeError was not propagated")
    finally:
        shutil.rmtree(iteration_dir, ignore_errors=True)
    sys.exit(1)


if __name__ == "__main__":
    main()
