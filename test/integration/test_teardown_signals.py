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

"""Integration test: background deaths while commands start, run and stop.

Teardown always runs to completion; a background death meanwhile is raised
once it is done. A death whose interrupt was used up is not lost: the next
foreground command raises it instead of running.
"""

import errno
import logging
import os
import signal
import sys
import time
from pathlib import Path

import pytest

from lambkin.common import defaults, signals
from lambkin.common.exceptions import (
    CommandError,
    LambkinProcessDiedUnexpectedlyError,
)
from lambkin.core.process import cgroup as cgroup_module
from lambkin.core.process.background import background


def _child_cgroups(cgroup: Path) -> list[Path]:
    return [p for p in cgroup.iterdir() if p.is_dir()]


def _kill_and_wait_recorded(pid: int) -> None:
    """Kill a background process and wait until its monitor recorded it."""
    os.kill(pid, signal.SIGKILL)
    deadline = time.monotonic() + 5
    while not signals.death_pending():
        assert time.monotonic() < deadline, "death was never recorded"
        time.sleep(0.01)


def _wait_for_file(path: Path) -> None:
    deadline = time.monotonic() + 5
    while not (path.exists() and path.read_text().strip()):
        assert time.monotonic() < deadline, f"{path} never appeared"
        time.sleep(0.01)


@pytest.fixture
def logs(monkeypatch, caplog):
    """Capture lambkin's warnings."""
    monkeypatch.setattr(logging.getLogger("lambkin"), "propagate", True)
    caplog.set_level(logging.WARNING, logger="lambkin")
    return caplog


def test_interrupted_command_unwinds_benchmark_code(
    shell, iteration_cgroup, lambkin_signals
):
    """Code after an interrupted foreground command does not run."""
    reached = False
    start = time.monotonic()
    with pytest.raises(LambkinProcessDiedUnexpectedlyError) as excinfo:
        with background(shell.sleep, "30") as bp:
            lambkin_signals.start_unsignalled(
                lambda: (time.sleep(0.3), os.kill(bp._proc.pid, signal.SIGKILL))
            )
            shell.sleep(30)
            reached = True
    assert not reached
    assert time.monotonic() - start < 5
    assert excinfo.value.argv == ["sleep", "30"]
    assert not signals.death_pending()
    assert _child_cgroups(iteration_cgroup) == []


def test_caught_start_error_does_not_hide_background_death(
    shell, tmp_path, iteration_cgroup, lambkin_signals, monkeypatch
):
    """The case the one-shot signal used to lose.

    A background process dies while a failed foreground start is cleaned up.
    The benchmark catches the CommandError. The next foreground command must
    not run; the death is reported when the background block ends.
    """
    marker = tmp_path / "ran"
    kill_cgroup = cgroup_module.kill_cgroup
    caught = reached = False
    with pytest.raises(LambkinProcessDiedUnexpectedlyError) as excinfo:
        with background(shell.sleep, "30") as bp:

            def _bg_dies_then_kill(cgroup, grace_period=3.0):
                monkeypatch.undo()
                _kill_and_wait_recorded(bp._proc.pid)
                kill_cgroup(cgroup, grace_period)

            monkeypatch.setattr(cgroup_module, "kill_cgroup", _bg_dies_then_kill)
            try:
                shell.lambkin_no_such_command_xyz()
            except CommandError:
                caught = True
            shell.touch(marker)
            reached = True
    assert caught
    assert not reached
    assert not marker.exists()
    assert excinfo.value.argv == ["sleep", "30"]
    assert not signals.death_pending()
    assert _child_cgroups(iteration_cgroup) == []


def test_death_during_failed_background_start_keeps_start_error(
    shell, iteration_cgroup, lambkin_signals, logs, monkeypatch
):
    """A death while a failed background start is cleaned up keeps the error."""
    kill_cgroup = cgroup_module.kill_cgroup
    with pytest.raises(CommandError):
        with background(shell.sleep, "30") as bp:

            def _bg_dies_then_kill(cgroup, grace_period=3.0):
                # Not monkeypatch.undo(): that would also undo the logs fixture.
                monkeypatch.setattr(cgroup_module, "kill_cgroup", kill_cgroup)
                _kill_and_wait_recorded(bp._proc.pid)
                kill_cgroup(cgroup, grace_period)

            monkeypatch.setattr(cgroup_module, "kill_cgroup", _bg_dies_then_kill)
            with background(shell.lambkin_no_such_command_xyz):
                pass
    assert "sleep 30 died unexpectedly (return code -9) while CommandError" in (
        logs.text
    )
    assert not signals.death_pending()
    assert _child_cgroups(iteration_cgroup) == []


def test_death_during_background_teardown_waits_out_grace(
    wait_gone, shell, tmp_path, iteration_cgroup, lambkin_signals, monkeypatch
):
    """Another background death does not cut a SIGTERM grace period short.

    The inner process ignores SIGTERM, so its teardown waits the whole grace
    period. The outer one dies during that wait; its death is raised only
    after the inner process is gone and its cgroup removed.
    """
    monkeypatch.setattr(defaults, "SIGTERM_GRACE_PERIOD", 1.5)
    pid_file = tmp_path / "stubborn.pid"
    with pytest.raises(LambkinProcessDiedUnexpectedlyError) as excinfo:
        with background(shell.sh, "-c", "sleep 0.5; exit 3"):
            with background(
                shell.sh,
                "-c",
                f"trap '' TERM; echo $$ > {pid_file}; exec sleep 30",
            ) as inner:
                _wait_for_file(pid_file)
                inner_cgroup = inner._cgroup
                teardown_start = time.monotonic()
    teardown = time.monotonic() - teardown_start

    assert excinfo.value.argv == ["sh", "-c", "sleep 0.5; exit 3"]
    assert excinfo.value.returncode == 3
    assert teardown >= 1.5, teardown
    assert wait_gone(int(pid_file.read_text()))
    assert not inner_cgroup.exists()
    assert not signals.death_pending()
    assert _child_cgroups(iteration_cgroup) == []


def test_death_during_nested_teardown_does_not_replace_error(
    shell, tmp_path, iteration_cgroup, lambkin_signals, logs, monkeypatch
):
    """An outer death during an inner block's teardown keeps the user's error."""
    # The inner block tears down because the benchmark raised. The outer
    # process dies meanwhile; its block logs that against the error instead
    # of replacing it.
    monkeypatch.setattr(defaults, "SIGTERM_GRACE_PERIOD", 1.5)
    pid_file = tmp_path / "stubborn.pid"
    with pytest.raises(ValueError, match="benchmark failed"):
        with background(shell.sh, "-c", "sleep 0.5; exit 3"):
            with background(
                shell.sh,
                "-c",
                f"trap '' TERM; echo $$ > {pid_file}; exec sleep 30",
            ):
                _wait_for_file(pid_file)
                raise ValueError("benchmark failed")
    assert "died unexpectedly (return code 3) while ValueError" in logs.text
    assert not signals.death_pending()
    assert _child_cgroups(iteration_cgroup) == []


def test_death_while_another_error_propagates_is_logged(
    shell, iteration_cgroup, lambkin_signals, logs
):
    """The other error wins, but the death leaves a warning."""
    with pytest.raises(ValueError, match="benchmark failed"):
        with background(shell.sleep, "30") as bp:
            with signals.deferred():
                _kill_and_wait_recorded(bp._proc.pid)
                raise ValueError("benchmark failed")
    assert "sleep 30 died unexpectedly (return code -9) while ValueError" in logs.text
    assert not signals.death_pending()
    assert _child_cgroups(iteration_cgroup) == []


def test_failed_background_teardown_still_forgets_death(
    shell, iteration_cgroup, lambkin_signals, logs, monkeypatch
):
    """A death is never left recorded, or later commands would all stop.

    Teardown fails after the process died. The teardown error propagates,
    the death is logged, and a later foreground command runs normally.
    """

    def _fail(cgroup, grace_period):
        raise OSError(errno.EACCES, "denied")

    # The package attribute "background" is the function, not the module.
    monkeypatch.setattr(sys.modules[background.__module__], "destroy_cgroup", _fail)
    with pytest.raises(OSError, match="denied"):
        with background(shell.sleep, "30") as bp:
            os.kill(bp._proc.pid, signal.SIGKILL)
            time.sleep(5)
    monkeypatch.undo()
    assert "sleep 30 died unexpectedly (return code -9) while PermissionError" in (
        logs.text
    )
    assert not signals.death_pending()
    assert shell.true().returncode == 0


def test_foreground_in_worker_thread_does_not_hold_off_interrupt(
    shell, iteration_cgroup, lambkin_signals
):
    """A command running in a worker thread does not delay a death report.

    The worker's command runs to completion; the main thread is interrupted
    as soon as the background process dies.
    """
    results = []
    start = time.monotonic()
    with pytest.raises(LambkinProcessDiedUnexpectedlyError):
        with background(shell.sleep, "30") as bp:
            worker = lambkin_signals.start_unsignalled(
                lambda: results.append(shell.sh("-c", "sleep 1.5").returncode)
            )
            time.sleep(0.3)
            os.kill(bp._proc.pid, signal.SIGKILL)
            time.sleep(10)
    assert time.monotonic() - start < 1.5
    worker.join()
    assert results == [0]
    assert not signals.death_pending()


@pytest.mark.parametrize("attempt", range(3))
def test_simultaneous_deaths_are_not_forwarded(
    shell, iteration_cgroup, lambkin_signals, attempt
):
    """Two processes dying together never reach the default SIGUSR1 handler."""
    with pytest.raises(LambkinProcessDiedUnexpectedlyError) as excinfo:
        with background(shell.sleep, "31") as outer:
            with background(shell.sleep, "32") as inner:
                lambkin_signals.start_unsignalled(
                    lambda: (
                        os.kill(outer._proc.pid, signal.SIGKILL),
                        os.kill(inner._proc.pid, signal.SIGKILL),
                    )
                )
                shell.sleep(30)
    # Whichever death interrupts first starts teardown; the other process may
    # only be seen dead once its own teardown started, and then counts as
    # expected. Either way exactly one death is raised and none is lost to
    # the default SIGUSR1 handler.
    assert excinfo.value.argv in (["sleep", "31"], ["sleep", "32"])
    assert lambkin_signals.forwarded == []
    assert not signals.death_pending()
    assert _child_cgroups(iteration_cgroup) == []
