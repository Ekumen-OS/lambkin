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

"""Unit tests for cgroup v2 lifecycle utilities."""

import errno
import os
import subprocess
import sys
import time
from pathlib import Path
from unittest.mock import patch

import pytest

from lambkin.core.process.cgroup import (
    find_delegated_cgroup,
    kill_cgroup,
    make_cgroup,
    make_iteration_cgroup,
    remove_cgroup,
)


def test_find_delegated_cgroup_returns_path():
    """find_delegated_cgroup returns a valid cgroup v2 path."""
    result = find_delegated_cgroup()
    assert result.exists()
    assert (result / "cgroup.procs").exists()


def test_find_delegated_cgroup_raises_if_no_cgroup_v2(tmp_path):
    """find_delegated_cgroup raises RuntimeError if no cgroup v2 line found."""
    fake_proc_cgroup = tmp_path / "cgroup"
    fake_proc_cgroup.write_text("1:cpu:/user.slice\n")

    with patch(
        "lambkin.core.process.cgroup.Path",
        side_effect=lambda x: fake_proc_cgroup if x == "/proc/self/cgroup" else Path(x),
    ):
        with pytest.raises(RuntimeError, match="No cgroup v2 found"):
            find_delegated_cgroup()


def test_make_cgroup_creates_directory(tmp_path):
    """make_cgroup creates a child directory under parent."""
    result = make_cgroup(tmp_path, "test-cgroup")
    assert result == tmp_path / "test-cgroup"
    assert result.exists()
    assert result.is_dir()


def test_make_cgroup_returns_path(tmp_path):
    """make_cgroup returns the path of the created directory."""
    result = make_cgroup(tmp_path, "my-cgroup")
    assert isinstance(result, Path)


def test_make_cgroup_is_idempotent(tmp_path):
    """make_cgroup does not raise if directory already exists."""
    make_cgroup(tmp_path, "test-cgroup")
    result = make_cgroup(tmp_path, "test-cgroup")
    assert result.exists()


def test_make_cgroup_nested(tmp_path):
    """make_cgroup can create nested cgroups."""
    parent = make_cgroup(tmp_path, "parent")
    child = make_cgroup(parent, "child")
    assert child == parent / "child"
    assert child.exists()


def test_remove_cgroup_removes_empty_directory(tmp_path):
    """remove_cgroup removes an empty cgroup directory."""
    cgroup = tmp_path / "test-cgroup"
    cgroup.mkdir()
    remove_cgroup(cgroup)
    assert not cgroup.exists()


def test_remove_cgroup_retries_until_empty(tmp_path):
    """remove_cgroup retries if the directory is not immediately removable."""
    cgroup = tmp_path / "test-cgroup"
    cgroup.mkdir()

    call_count = 0
    original_rmdir = Path.rmdir

    def rmdir_side_effect(self):
        nonlocal call_count
        call_count += 1
        if call_count < 3:
            raise OSError(errno.ENOTEMPTY, "Directory not empty")
        original_rmdir(self)

    with patch.object(Path, "rmdir", rmdir_side_effect):
        remove_cgroup(cgroup)

    assert not cgroup.exists()
    assert call_count == 3


def test_remove_cgroup_gives_up_after_timeout(tmp_path):
    """remove_cgroup gives up after the timeout if directory cannot be removed."""
    cgroup = tmp_path / "test-cgroup"
    cgroup.mkdir()

    with (
        patch.object(
            Path, "rmdir", side_effect=OSError(errno.ENOTEMPTY, "Directory not empty")
        ),
        patch(
            "lambkin.core.process.cgroup.time.monotonic", side_effect=[0.0, 0.0, 10.0]
        ),
    ):
        remove_cgroup(cgroup)

    assert cgroup.exists()
    cgroup.rmdir()


def test_make_iteration_cgroup_creates_directory(tmp_path):
    """make_iteration_cgroup creates a child cgroup under delegated."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    result = make_iteration_cgroup(tmp_path, iteration_dir)
    assert result.exists()
    assert result.parent == tmp_path


def test_make_iteration_cgroup_name_contains_iteration_info(tmp_path):
    """make_iteration_cgroup name contains variant and iteration folder names."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    result = make_iteration_cgroup(tmp_path, iteration_dir)
    assert "var_1" in result.name
    assert "iter_1" in result.name


def test_make_iteration_cgroup_unique_names(tmp_path):
    """make_iteration_cgroup produces unique names for the same iteration_dir."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    result1 = make_iteration_cgroup(tmp_path, iteration_dir)
    result2 = make_iteration_cgroup(tmp_path, iteration_dir)
    assert result1 != result2


def test_kill_cgroup_terminates_cooperative_process():
    """kill_cgroup sends SIGTERM and the process exits cleanly."""
    delegated = find_delegated_cgroup()
    cgroup = make_cgroup(delegated, "test-kill-cooperative")

    proc = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)"],
        preexec_fn=lambda: (cgroup / "cgroup.procs").write_text(str(os.getpid())),
    )
    time.sleep(0.3)

    kill_cgroup(cgroup, grace_period=3.0)
    proc.wait()
    assert proc.poll() is not None
    remove_cgroup(cgroup)


def test_kill_cgroup_kills_non_cooperative_process():
    """kill_cgroup falls back to SIGKILL for processes ignoring SIGTERM."""
    delegated = find_delegated_cgroup()
    cgroup = make_cgroup(delegated, "test-kill-non-cooperative")

    proc = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "import signal, time; signal.signal(signal.SIGTERM, "
            "signal.SIG_IGN); time.sleep(60)",
        ],
        preexec_fn=lambda: (cgroup / "cgroup.procs").write_text(str(os.getpid())),
    )
    time.sleep(0.3)

    kill_cgroup(cgroup, grace_period=1.0)
    proc.wait()
    assert proc.poll() is not None
    remove_cgroup(cgroup)


def test_kill_cgroup_empty_cgroup_does_nothing():
    """kill_cgroup on an empty cgroup does not raise."""
    delegated = find_delegated_cgroup()
    cgroup = make_cgroup(delegated, "test-kill-empty")

    kill_cgroup(cgroup, grace_period=1.0)
    remove_cgroup(cgroup)
    assert not cgroup.exists()
