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
"""Integration test: background process death interrupts a blocking foreground."""

import shutil
import signal
import sys
import tempfile
from pathlib import Path

from lambkin.common import exceptions
from lambkin.common import signals as lambkin_signals
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

DIES_QUICKLY = "import sys, time; time.sleep(0.5); sys.exit(1)"


def _make_sigusr1_handler(previous):
    def _handle_sigusr1(signum, frame):
        if lambkin_signals.sigusr1_pending.is_set():
            lambkin_signals.sigusr1_pending.clear()
            raise exceptions.LambkinProcessDiedUnexpectedlyError([], 1)
        elif callable(previous):
            previous(signum, frame)

    return _handle_sigusr1


def main():
    """Run integration test for background process dying while foreground is blocking.

    Verifies that when a background process dies unexpectedly while a foreground
    process is blocking the main thread, LambkinProcessDiedUnexpectedlyError is
    raised and all cgroups are cleaned up correctly.
    """
    previous = signal.signal(signal.SIGUSR1, signal.SIG_DFL)
    signal.signal(signal.SIGUSR1, _make_sigusr1_handler(previous))

    iteration_dir = Path(tempfile.mkdtemp())
    try:
        cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)
        shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)

        print("Starting background process that will die quickly...")
        try:
            with background(shell.python3, "-c", DIES_QUICKLY) as bp1:
                print(f"Background process started — pid={bp1._proc.pid}")
                cgroup1 = bp1._cgroup
                print("Foreground blocking for 30s...")
                shell.python3("-c", COOPERATIVE)
                print("ERROR: foreground should have been interrupted")
                sys.exit(1)
        except exceptions.LambkinProcessDiedUnexpectedlyError as e:
            print(f"LambkinProcessDiedUnexpectedlyError propagated correctly: {e}")
            print(f"Background process dead: {bp1._proc.poll() is not None}")
            assert not cgroup_exists(cgroup1), "Cgroup should have been removed"
            print("Background dies during foreground test passed")
            return

        print("ERROR: LambkinProcessDiedUnexpectedlyError was not propagated")
    finally:
        shutil.rmtree(iteration_dir, ignore_errors=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
