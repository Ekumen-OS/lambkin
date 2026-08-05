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

import time

import pytest

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

FOREGROUND_FAILURE_MESSAGE = "Foreground failed intentionally"


def test_foreground_fails(tmp_path):
    """Verify that a foreground failure terminates all background processes.

    When the foreground raises an exception, both background processes must
    be terminated cleanly and the original exception must propagate.

    Raises:
        RuntimeError: Raised intentionally to simulate a foreground failure.
    """
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), tmp_path)
    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    cgroup1 = None
    cgroup2 = None

    with pytest.raises(RuntimeError, match=FOREGROUND_FAILURE_MESSAGE):
        with background(shell.python3, "-c", COOPERATIVE) as bp1:
            cgroup1 = bp1._cgroup
            with background(shell.python3, "-c", COOPERATIVE) as bp2:
                cgroup2 = bp2._cgroup
                time.sleep(0.5)
                raise RuntimeError(FOREGROUND_FAILURE_MESSAGE)

    assert not cgroup_exists(cgroup2), "Inner cgroup should have been removed"
    assert not cgroup_exists(cgroup1), "Outer cgroup should have been removed"
