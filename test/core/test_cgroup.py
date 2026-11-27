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
import threading
import time
from pathlib import Path
from unittest.mock import patch

import pytest

from lambkin.core.process.cgroup import (
    cgroup_joiner,
    find_delegated_cgroup,
    find_run_cgroup,
    kill_cgroup,
    kill_cgroup_tree,
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


_LATE_JOINER_PARENT = """
import signal, subprocess, sys, time

def _on_term(signum, frame):
    # Blocked until the child installed its handler, so it is not lost.
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM})
    subprocess.Popen([sys.executable, "-c", sys.argv[1], sys.argv[2]])
    signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGTERM})

signal.signal(signal.SIGTERM, _on_term)
print("ready", flush=True)
while True:
    time.sleep(0.05)
"""

_LATE_JOINER_CHILD = """
import signal, sys, time

def _on_term(signum, frame):
    with open(sys.argv[1], "w") as f:
        f.write("terminated")
    sys.exit(0)

signal.signal(signal.SIGTERM, _on_term)
signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGTERM})
while True:
    time.sleep(0.05)
"""


def test_kill_cgroup_terminates_processes_started_during_grace(tmp_path):
    """A process forked during the grace period gets SIGTERM, not just SIGKILL.

    The first member survives SIGTERM and starts a new one in response.
    """
    delegated = find_delegated_cgroup()
    cgroup = make_cgroup(delegated, "test-kill-late-joiner")
    log = tmp_path / "child.log"
    proc = subprocess.Popen(
        [sys.executable, "-c", _LATE_JOINER_PARENT, _LATE_JOINER_CHILD, str(log)],
        stdout=subprocess.PIPE,
        text=True,
        preexec_fn=lambda: (cgroup / "cgroup.procs").write_text(str(os.getpid())),
    )
    try:
        assert proc.stdout is not None
        assert proc.stdout.readline().strip() == "ready"

        kill_cgroup(cgroup, grace_period=2.0)
        proc.wait()
        assert log.read_text() == "terminated"
    finally:
        proc.kill()
        proc.wait()
        remove_cgroup(cgroup)


def test_kill_cgroup_tree_waits_for_descendants(tmp_path):
    """kill_cgroup_tree returns only once the whole tree is unpopulated.

    ``cgroup.events`` reports processes in descendants too, unlike the
    cgroup's own ``cgroup.threads``.
    """
    (tmp_path / "cgroup.kill").write_text("")
    (tmp_path / "cgroup.threads").write_text("")
    events = tmp_path / "cgroup.events"
    events.write_text("populated 1\nfrozen 0\n")
    timer = threading.Timer(0.3, events.write_text, ["populated 0\nfrozen 0\n"])
    timer.start()
    start = time.monotonic()
    try:
        assert kill_cgroup_tree(tmp_path, grace_period=5.0)
    finally:
        timer.cancel()
    assert time.monotonic() - start >= 0.3
    assert (tmp_path / "cgroup.kill").read_text() == "1"


def test_cgroup_joiner_moves_child_before_exec():
    """cgroup_joiner's function as preexec_fn places the child in the cgroup."""
    cgroup = make_cgroup(find_delegated_cgroup(), "test-join")
    try:
        out = subprocess.run(
            ["cat", "/proc/self/cgroup"],
            preexec_fn=cgroup_joiner(cgroup),
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        assert out.strip().endswith("/test-join")
    finally:
        remove_cgroup(cgroup)


def test_find_run_cgroup_uses_cli_environment(monkeypatch, tmp_path):
    """Iteration cgroups go under the run cgroup the CLI exports."""
    (tmp_path / "cgroup.procs").write_text("")
    monkeypatch.setenv("LAMBKIN_RUN_CGROUP", str(tmp_path))
    assert find_run_cgroup() == tmp_path


@pytest.mark.parametrize("make_dir", [False, True])
def test_find_run_cgroup_rejects_stale_cli_environment(monkeypatch, tmp_path, make_dir):
    """A run cgroup that is gone, or not a cgroup, names the variable."""
    stale = tmp_path / "lambkin-bench-1234"
    if make_dir:
        stale.mkdir()
    monkeypatch.setenv("LAMBKIN_RUN_CGROUP", str(stale))
    with pytest.raises(RuntimeError, match="LAMBKIN_RUN_CGROUP=.*unset it"):
        find_run_cgroup()


@pytest.mark.parametrize("value", [None, ""])
def test_find_run_cgroup_falls_back_to_delegated_cgroup(monkeypatch, tmp_path, value):
    """Without the CLI, or with the variable empty, use the delegated cgroup."""
    if value is None:
        monkeypatch.delenv("LAMBKIN_RUN_CGROUP", raising=False)
    else:
        monkeypatch.setenv("LAMBKIN_RUN_CGROUP", value)
    monkeypatch.setattr(
        "lambkin.core.process.cgroup.find_delegated_cgroup",
        lambda: tmp_path,
    )
    assert find_run_cgroup() == tmp_path
