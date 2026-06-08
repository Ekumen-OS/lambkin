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

"""Integration test: background process raises an exception intentionally."""

import shutil
import tempfile
import time
from pathlib import Path

import pytest

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

RAISES = (
    "import time; "
    "time.sleep(0.5); "
    "raise RuntimeError('intentional exception in background process')"
)


def test_exception_in_background():
    """Verify that an unhandled exception in a background process is detected.

    When a background process raises an unhandled exception and exits with a
    non-zero return code, LambkinProcessDiedUnexpectedlyError must be raised
    and all other background processes must be terminated with their cgroups
    cleaned up.
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
                with background(shell.python3, "-c", RAISES) as bp2:
                    cgroup2 = bp2._cgroup
                    time.sleep(2)

        assert not cgroup_exists(cgroup2), "Inner cgroup should have been removed"
        assert not cgroup_exists(cgroup1), "Outer cgroup should have been removed"

    finally:
        shutil.rmtree(iteration_dir, ignore_errors=True)
