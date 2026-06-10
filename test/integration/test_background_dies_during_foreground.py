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


def test_background_dies_during_foreground(tmp_path):
    """Verify that a background process dying interrupts a blocking foreground.

    When a background process dies unexpectedly while a foreground process is
    blocking the main thread, LambkinProcessDiedUnexpectedlyError must be raised
    and all cgroups must be cleaned up correctly.
    """
    signals.setup()
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), tmp_path)
    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    cgroup1 = None
    with pytest.raises(LambkinProcessDiedUnexpectedlyError):
        with background(shell.python3, "-c", DIES_QUICKLY) as bp1:
            cgroup1 = bp1._cgroup
            shell.python3("-c", COOPERATIVE)
    assert not cgroup_exists(cgroup1), "Cgroup should have been removed"


def test_background_dies_during_foreground_error_message(tmp_path):
    """Verify that LambkinProcessDiedUnexpectedlyError.

    Checks that the error message reaches the terminal when a background process
    dies during a blocking foreground.
    """
    signals.setup()
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), tmp_path)
    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    with pytest.raises(LambkinProcessDiedUnexpectedlyError) as exc_info:
        with background(shell.python3, "-c", DIES_QUICKLY) as _:
            shell.python3("-c", COOPERATIVE)
    assert exc_info.value.argv == ["python3", "-c", DIES_QUICKLY]
    assert exc_info.value.returncode == 1
