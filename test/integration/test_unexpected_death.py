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

"""Integration test: inner background process dies unexpectedly."""

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

signals.setup()

COOPERATIVE = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "time.sleep(30)"
)

DIES_QUICKLY = "import time; time.sleep(0.5)"


def test_unexpected_death():
    """Verify that unexpected death of an inner background process is detected.

    When an inner background process dies before the context manager exits,
    LambkinProcessDiedUnexpectedlyError must be raised and the outer
    background process must also be terminated with its cgroup cleaned up.
    """
    iteration_dir = Path(tempfile.mkdtemp())
    try:
        cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)
        shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
        cgroup1 = None
        cgroup2 = None

        with pytest.raises(LambkinProcessDiedUnexpectedlyError):
            with background(shell.python3, "-c", COOPERATIVE) as bp1:
                cgroup1 = bp1._cgroup
                with background(shell.python3, "-c", DIES_QUICKLY) as bp2:
                    cgroup2 = bp2._cgroup
                    shell.python3("-c", COOPERATIVE)

        assert not cgroup_exists(cgroup2), "Inner cgroup should have been removed"
        assert not cgroup_exists(cgroup1), "Outer cgroup should have been removed"
    finally:
        shutil.rmtree(iteration_dir, ignore_errors=True)
