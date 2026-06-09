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
import tempfile
from pathlib import Path

import pytest

from lambkin.common import signals
from lambkin.common.exceptions import LambkinProcessDiedUnexpectedlyError
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


def test_background_dies_during_foreground():
    """Verify that a background process dying interrupts a blocking foreground.

    When a background process dies unexpectedly while a foreground process is
    blocking the main thread, LambkinProcessDiedUnexpectedlyError must be raised
    and all cgroups must be cleaned up correctly.
    """
    signals.setup()
    iteration_dir = Path(tempfile.mkdtemp())
    try:
        cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)
        shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
        cgroup1 = None
        with pytest.raises(LambkinProcessDiedUnexpectedlyError):
            with background(shell.python3, "-c", DIES_QUICKLY) as bp1:
                cgroup1 = bp1._cgroup
                shell.python3("-c", COOPERATIVE)
        assert not cgroup_exists(cgroup1), "Cgroup should have been removed"
    finally:
        shutil.rmtree(iteration_dir, ignore_errors=True)


def test_background_dies_during_foreground_error_message():
    """Verify that LambkinProcessDiedUnexpectedlyError.

    Checks that the error message reaches the terminal when a background process
    dies during a blocking foreground.
    """
    signals.setup()
    iteration_dir = Path(tempfile.mkdtemp())
    try:
        cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)
        shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
        with pytest.raises(LambkinProcessDiedUnexpectedlyError) as exc_info:
            with background(shell.python3, "-c", DIES_QUICKLY) as _:
                shell.python3("-c", COOPERATIVE)
        assert "python3" in str(exc_info.value)
        assert "1" in str(exc_info.value)
    finally:
        shutil.rmtree(iteration_dir, ignore_errors=True)
