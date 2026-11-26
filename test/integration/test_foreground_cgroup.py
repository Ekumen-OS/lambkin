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

"""Integration test: foreground commands run in, and clean up, their own cgroup."""

import errno
import logging
import os
import signal
import threading
import time
from pathlib import Path

import pytest

from lambkin.common import defaults, signals
from lambkin.common.exceptions import LambkinSIGUSR1Interrupt
from lambkin.core.process.cgroup import (
    find_delegated_cgroup,
    kill_cgroup_tree,
    make_iteration_cgroup,
    remove_cgroup_tree,
)
from lambkin.core.process.resources import pids_in_cgroup
from lambkin.core.shell import proxy
from lambkin.core.shell.proxy import CommandError, CommandProxy, ShellProxy


def _cgroup_v2_writable() -> bool:
    try:
        find_delegated_cgroup()
    except (RuntimeError, OSError):
        return False
    return True


pytestmark = pytest.mark.skipif(
    not _cgroup_v2_writable(), reason="cgroup v2 is not writable here"
)


@pytest.fixture
def iteration_cgroup(tmp_path):
    """Create an iteration cgroup and tear it down after the test."""
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), tmp_path / "iter_1")
    yield cgroup
    if cgroup.exists():
        kill_cgroup_tree(cgroup)
        remove_cgroup_tree(cgroup)


@pytest.fixture
def shell(tmp_path, iteration_cgroup):
    """Return a real ShellProxy bound to the iteration cgroup."""
    return ShellProxy(dry_run=False, cwd=tmp_path, cgroup=iteration_cgroup)


def _child_cgroups(cgroup: Path) -> list[Path]:
    return [p for p in cgroup.iterdir() if p.is_dir()]


def test_foreground_runs_in_its_own_cgroup(shell, tmp_path, iteration_cgroup):
    """A foreground command runs under iter-…/<cmd>-…/, removed after it returns."""
    shell.cat("/proc/self/cgroup")

    line = (tmp_path / "cat.stdout.log").read_text().strip()
    rel = line.removeprefix("0::")
    cmd_cgroup = Path("/sys/fs/cgroup") / rel.lstrip("/")
    assert cmd_cgroup.parent == iteration_cgroup
    assert cmd_cgroup.name.startswith("cat-")
    assert not cmd_cgroup.exists()
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_orphan_is_killed_with_warning(
    wait_gone, shell, tmp_path, iteration_cgroup, caplog, monkeypatch
):
    """A process a foreground command leaves behind is killed when it returns."""
    # configure_logging, run by other tests, stops "lambkin" from propagating
    # to the root logger that caplog listens on.
    monkeypatch.setattr(logging.getLogger("lambkin"), "propagate", True)
    pid_file = tmp_path / "orphan.pid"
    with caplog.at_level(logging.WARNING, logger="lambkin.core.shell.proxy"):
        shell.sh("-c", f"sleep 60 & echo $! > {pid_file}; exit 0")

    orphan = int(pid_file.read_text())
    assert wait_gone(orphan), f"orphan {orphan} still alive"
    assert any(
        str(orphan) in r.getMessage() and "sleep 60" in r.getMessage()
        for r in caplog.records
    ), caplog.text
    assert pids_in_cgroup(iteration_cgroup) == []
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_clean_command_logs_no_warning(
    shell, iteration_cgroup, caplog, monkeypatch
):
    """A command that leaves nothing behind logs no warning."""
    monkeypatch.setattr(logging.getLogger("lambkin"), "propagate", True)
    with caplog.at_level(logging.WARNING, logger="lambkin"):
        shell.true()
        shell.sh("-c", "sleep 0.1 & wait")
    assert caplog.records == [], caplog.text
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_orphan_in_sub_cgroup_is_killed_with_warning(
    wait_gone, shell, tmp_path, iteration_cgroup, caplog, monkeypatch
):
    """A process a command moved into a cgroup of its own is still cleaned up."""
    monkeypatch.setattr(logging.getLogger("lambkin"), "propagate", True)
    pid_file = tmp_path / "nested.pid"
    script = tmp_path / "nest.sh"
    script.write_text(
        "cg=\"/sys/fs/cgroup$(sed -n 's/^0:://p' /proc/self/cgroup)\"\n"
        'mkdir "$cg/sub"\n'
        f'sh -c \'echo $$ > "$0/sub/cgroup.procs"; echo $$ > {pid_file};'
        ' exec sleep 64\' "$cg" &\n'
        f"while [ ! -s {pid_file} ]; do sleep 0.01; done\n"
    )
    with caplog.at_level(logging.WARNING, logger="lambkin"):
        shell.sh(str(script))

    nested = int(pid_file.read_text())
    assert wait_gone(nested), f"nested {nested} still alive"
    assert any(
        str(nested) in r.getMessage() and "sleep 64" in r.getMessage()
        for r in caplog.records
    ), caplog.text
    assert "Could not remove cgroup" not in caplog.text
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_failure_cleans_up_cgroup(shell, iteration_cgroup):
    """A command that exits non-zero still has its cgroup removed."""
    with pytest.raises(CommandError):
        shell.sh("-c", "exit 3")
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_not_found_cleans_up_cgroup(shell, iteration_cgroup):
    """A command that cannot be started still has its cgroup removed."""
    with pytest.raises(CommandError, match="Command not found"):
        shell.lambkin_no_such_command_xyz()
    assert _child_cgroups(iteration_cgroup) == []


class _InterruptedError(Exception):
    pass


def _raise_after(seconds: float, exc: type[BaseException]):
    """Arrange for exc to be raised in the main thread after a delay."""

    def _raise(signum, frame):
        raise exc

    previous = signal.signal(signal.SIGALRM, _raise)
    signal.setitimer(signal.ITIMER_REAL, seconds)

    def _restore():
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)

    return _restore


def test_foreground_interrupted_kills_whole_tree(
    wait_gone, shell, tmp_path, iteration_cgroup
):
    """An exception raised while waiting kills the command and its children."""
    pid_file = tmp_path / "child.pid"
    restore = _raise_after(1.0, _InterruptedError)
    try:
        with pytest.raises(_InterruptedError):
            shell.sh("-c", f"sleep 60 & echo $! > {pid_file}; wait")
    finally:
        restore()

    assert wait_gone(int(pid_file.read_text()))
    assert pids_in_cgroup(iteration_cgroup) == []
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_sigusr1_interrupt_kills_whole_tree(
    wait_gone, shell, tmp_path, iteration_cgroup
):
    """A dying background process interrupts the command; it is cleaned up."""
    pid_file = tmp_path / "child.pid"
    restore = _raise_after(1.0, LambkinSIGUSR1Interrupt)
    try:
        result = shell.sh("-c", f"sleep 60 & echo $! > {pid_file}; wait")
    finally:
        restore()

    assert result.returncode == -1
    assert wait_gone(int(pid_file.read_text()))
    assert pids_in_cgroup(iteration_cgroup) == []
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_interrupted_without_cgroup_kills_command(wait_gone, tmp_path):
    """Without a cgroup, an interrupted command is still killed."""
    shell = ShellProxy(dry_run=False, cwd=tmp_path)
    pid_file = tmp_path / "cmd.pid"
    restore = _raise_after(1.0, _InterruptedError)
    try:
        with pytest.raises(_InterruptedError):
            shell.sh("-c", f"echo $$ > {pid_file}; exec sleep 60")
    finally:
        restore()

    assert wait_gone(int(pid_file.read_text()))


def test_foreground_daemon_in_new_session_is_killed(
    wait_gone, shell, tmp_path, iteration_cgroup
):
    """A process that left the command's session is still in its cgroup."""
    pid_file = tmp_path / "daemon.pid"
    shell.sh(
        "-c",
        f"setsid sh -c 'echo $$ > {pid_file}; exec sleep 61' "
        "</dev/null >/dev/null 2>&1 &"
        f" while [ ! -s {pid_file} ]; do sleep 0.01; done",
    )

    assert wait_gone(int(pid_file.read_text()))
    assert pids_in_cgroup(iteration_cgroup) == []
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_orphan_ignoring_sigterm_is_sigkilled(
    wait_gone, shell, tmp_path, iteration_cgroup, monkeypatch
):
    """An orphan that ignores SIGTERM is killed after the grace period."""
    monkeypatch.setattr(defaults, "SIGTERM_GRACE_PERIOD", 0.2)
    pid_file = tmp_path / "stubborn.pid"
    shell.sh(
        "-c",
        f"sh -c 'trap \"\" TERM; echo $$ > {pid_file}; exec sleep 62' &"
        f" while [ ! -s {pid_file} ]; do sleep 0.01; done",
    )

    assert wait_gone(int(pid_file.read_text()))
    assert pids_in_cgroup(iteration_cgroup) == []
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_child_setup_failure_cleans_up_cgroup(
    shell, iteration_cgroup, monkeypatch
):
    """If the child cannot join its cgroup, the command fails cleanly."""

    def _fail():
        raise OSError(errno.EACCES, "denied")

    monkeypatch.setattr("lambkin.core.shell.proxy.cgroup_joiner", lambda cgroup: _fail)
    with pytest.raises(CommandError, match="Could not prepare 'true'"):
        shell.true()
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_missing_iteration_cgroup_is_reported(shell, iteration_cgroup):
    """A missing iteration cgroup is not reported as a missing command."""
    iteration_cgroup.rmdir()
    with pytest.raises(CommandError, match="Could not create a cgroup for 'true'"):
        shell.true()


@pytest.mark.parametrize(("script", "returncode"), [("exit 0", 0), ("exit 3", 3)])
def test_foreground_cleanup_failure_keeps_command_result(
    shell, iteration_cgroup, caplog, monkeypatch, script, returncode
):
    """A cgroup that cannot be removed is logged; the command's result stands."""
    monkeypatch.setattr(logging.getLogger("lambkin"), "propagate", True)

    def _fail(cgroup):
        raise OSError(errno.EACCES, "denied")

    monkeypatch.setattr("lambkin.core.shell.proxy.remove_cgroup", _fail)
    with caplog.at_level(logging.WARNING, logger="lambkin.core.shell.proxy"):
        if returncode:
            with pytest.raises(CommandError) as excinfo:
                shell.sh("-c", script)
            assert excinfo.value.returncode == returncode
        else:
            assert shell.sh("-c", script).returncode == 0
    assert "Could not clean up cgroup" in caplog.text


def test_foreground_streams_closed_when_cleanup_raises(
    shell, iteration_cgroup, monkeypatch
):
    """Log files are closed even if releasing the cgroup raises."""
    opened = []
    original = CommandProxy.open_streams

    def _record(self, *args, **kwargs):
        streams = original(self, *args, **kwargs)
        opened.extend(s for s in streams if s is not None)
        return streams

    real_remove = proxy.remove_cgroup

    def _remove_then_fail(cgroup):
        real_remove(cgroup)
        raise _InterruptedError

    monkeypatch.setattr(CommandProxy, "open_streams", _record)
    monkeypatch.setattr(proxy, "remove_cgroup", _remove_then_fail)
    with pytest.raises(_InterruptedError):
        shell.true()
    assert opened
    assert all(s.closed for s in opened)
    assert _child_cgroups(iteration_cgroup) == []


@pytest.fixture
def lambkin_sigusr1():
    """Install lambkin's SIGUSR1 handler; return a function that fires it."""
    previous = signal.signal(signal.SIGUSR1, signals._make_handler(signal.SIG_IGN))

    def _fire_after(seconds: float) -> threading.Timer:
        def _fire():
            signals.sigusr1_pending.set()
            os.kill(os.getpid(), signal.SIGUSR1)

        timer = threading.Timer(seconds, _fire)
        timer.start()
        return timer

    yield _fire_after
    signal.signal(signal.SIGUSR1, previous)
    signals.sigusr1_pending.clear()


def test_foreground_sigusr1_during_cleanup_finishes_cleanup(
    wait_gone, shell, tmp_path, iteration_cgroup, lambkin_sigusr1, caplog, monkeypatch
):
    """A background death while an orphan is being killed does not cut cleanup.

    The orphan is reported once, and cleanup does not start a second SIGTERM
    grace period.
    """
    monkeypatch.setattr(logging.getLogger("lambkin"), "propagate", True)
    pid_file = tmp_path / "stubborn.pid"
    # The orphan ignores SIGTERM, so cleanup spends the grace period waiting
    # on it; the interrupt lands in that wait.
    timer = lambkin_sigusr1(1.0)
    start = time.monotonic()
    try:
        with caplog.at_level(logging.WARNING, logger="lambkin.core.shell.proxy"):
            result = shell.sh(
                "-c",
                f"sh -c 'trap \"\" TERM; echo $$ > {pid_file}; exec sleep 63' &"
                f" while [ ! -s {pid_file} ]; do sleep 0.01; done",
            )
    finally:
        timer.cancel()
    elapsed = time.monotonic() - start

    warnings = [r for r in caplog.records if "left 1 process(es)" in r.getMessage()]
    assert len(warnings) == 1, caplog.text
    # One grace period started at about 0 s and cut short at 1 s; a restarted
    # one would run until about 1 + SIGTERM_GRACE_PERIOD.
    assert elapsed < defaults.SIGTERM_GRACE_PERIOD, elapsed

    assert result.returncode == -1
    assert wait_gone(int(pid_file.read_text()))
    assert pids_in_cgroup(iteration_cgroup) == []
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_dry_run_creates_no_cgroup(tmp_path, iteration_cgroup):
    """A dry run never creates a command cgroup."""
    shell = ShellProxy(dry_run=True, cwd=tmp_path, cgroup=iteration_cgroup)
    shell.sleep(60)
    assert _child_cgroups(iteration_cgroup) == []


def test_foreground_without_cgroup_stays_in_caller_cgroup(tmp_path):
    """A proxy without a cgroup runs commands in the caller's cgroup, as before."""
    shell = ShellProxy(dry_run=False, cwd=tmp_path)
    shell.cat("/proc/self/cgroup")
    assert (tmp_path / "cat.stdout.log").read_text() == Path(
        "/proc/self/cgroup"
    ).read_text()
