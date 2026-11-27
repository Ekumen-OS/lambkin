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

"""Unit tests: background deaths, the SIGUSR1 interrupt, and holding it off."""

import os
import signal
import subprocess
import sys
import threading
import time

import pytest

from lambkin.common import signals
from lambkin.common.exceptions import LambkinSIGUSR1Interrupt


def test_unrelated_sigusr1_is_forwarded(lambkin_signals):
    """A SIGUSR1 lambkin did not send reaches the previous handler."""
    os.kill(os.getpid(), signal.SIGUSR1)
    lambkin_signals.settle()
    assert lambkin_signals.forwarded == [signal.SIGUSR1]


def test_death_interrupts_main_thread(lambkin_signals):
    """Outside any deferred block, a death interrupts the main thread."""
    with pytest.raises(LambkinSIGUSR1Interrupt):
        lambkin_signals.die()
        lambkin_signals.settle()
    assert signals.death_pending()
    assert lambkin_signals.forwarded == []


def test_death_in_deferred_block_raises_when_it_ends(lambkin_signals):
    """The block runs to the end; the death is raised after it."""
    finished = False
    with pytest.raises(LambkinSIGUSR1Interrupt):
        with signals.deferred():
            lambkin_signals.die()
            lambkin_signals.settle()
            finished = True
    assert finished
    assert signals.death_pending()


def test_deferred_block_error_wins_over_death(lambkin_signals):
    """An error leaving the block is not replaced; the death stays recorded."""
    with pytest.raises(ValueError, match="teardown failed"):
        with signals.deferred():
            lambkin_signals.die()
            lambkin_signals.settle()
            raise ValueError("teardown failed")
    assert signals.death_pending()


def test_deferred_block_without_death_raises_nothing(lambkin_signals):
    """A deferred block with no death meanwhile ends normally."""
    with signals.deferred():
        lambkin_signals.settle()
    assert not signals.death_pending()


def test_nested_deferred_blocks_raise_only_at_the_outermost(lambkin_signals):
    """An inner block ending does not raise while an outer one still runs."""
    inner_done = False
    with pytest.raises(LambkinSIGUSR1Interrupt):
        with signals.deferred():
            with signals.deferred():
                lambkin_signals.die()
                lambkin_signals.settle()
            inner_done = True
            lambkin_signals.settle()
    assert inner_done


def test_interruptible_block_is_interrupted_inside_deferred(lambkin_signals):
    """The one interruptible wait inside a deferred block is cut short."""
    reached = False
    with pytest.raises(LambkinSIGUSR1Interrupt):
        with signals.deferred():
            with signals.interruptible():
                lambkin_signals.die()
                lambkin_signals.settle()
                reached = True
    assert not reached


def test_interruptible_block_raises_on_entry_for_an_earlier_death(lambkin_signals):
    """A death whose signal was held off is raised as soon as a wait starts."""
    entered = False
    with signals.deferred():
        lambkin_signals.die()
        lambkin_signals.settle()
        with pytest.raises(LambkinSIGUSR1Interrupt):
            with signals.interruptible():
                entered = True
        # Handled here, so the deferred block still raises on exit below.
        signals.clear_death(next(iter(signals._dead)))
    assert not entered


def test_death_while_an_interrupt_unwinds_is_held_off(lambkin_signals):
    """A second death during cleanup after an interrupt does not cut it short."""
    cleaned_up = False
    with pytest.raises(LambkinSIGUSR1Interrupt):
        with signals.deferred():
            try:
                with signals.interruptible():
                    lambkin_signals.die()
                    lambkin_signals.settle()
            except LambkinSIGUSR1Interrupt:
                lambkin_signals.die()
                lambkin_signals.settle()
                cleaned_up = True
    assert cleaned_up
    assert len(signals._dead) == 2


def test_interruptible_resets_after_a_normal_exit(lambkin_signals):
    """A death after the wait returned normally is held off, not raised."""
    finished = False
    with pytest.raises(LambkinSIGUSR1Interrupt):
        with signals.deferred():
            with signals.interruptible():
                pass
            lambkin_signals.die()
            lambkin_signals.settle()
            finished = True
    assert finished


def test_interrupt_ends_the_interruptible_window_at_once(lambkin_signals):
    """The handler itself ends the window when it raises.

    A second signal can land before ``interruptible()``'s ``finally`` runs;
    it must be held off so the cleanup after the first interrupt completes.
    """
    handler = signal.getsignal(signal.SIGUSR1)
    assert callable(handler)
    with signals.deferred():
        signals._dead.add(key := object())
        signals._interruptible = True
        with pytest.raises(LambkinSIGUSR1Interrupt):
            handler(signal.SIGUSR1, None)
        assert not signals._interruptible
        handler(signal.SIGUSR1, None)  # held off: does not raise
        signals.clear_death(key)


def test_late_signal_for_a_reported_death_is_dropped(lambkin_signals):
    """A signal that arrives after its death was reported does nothing."""
    signals.sigusr1_pending.set()
    os.kill(os.getpid(), signal.SIGUSR1)
    lambkin_signals.settle()
    assert lambkin_signals.forwarded == []
    assert not signals.sigusr1_pending.is_set()


def test_second_death_signal_after_the_flag_was_cleared_is_not_forwarded(
    lambkin_signals,
):
    """While a death is unreported, a SIGUSR1 is treated as lambkin's.

    Two deaths in quick succession can deliver the second signal after the
    first one's handler cleared the flag. Forwarding it to a default handler
    would kill the benchmark.
    """
    with pytest.raises(LambkinSIGUSR1Interrupt):
        with signals.deferred():
            lambkin_signals.die()
            lambkin_signals.settle()
            assert not signals.sigusr1_pending.is_set()
            os.kill(os.getpid(), signal.SIGUSR1)
            lambkin_signals.settle()
    assert lambkin_signals.forwarded == []


def test_raise_if_died_checks_recorded_deaths(lambkin_signals):
    """raise_if_died raises exactly while a death is unreported."""
    signals.raise_if_died()
    with signals.deferred():
        key = lambkin_signals.die()
        lambkin_signals.settle()
        with pytest.raises(LambkinSIGUSR1Interrupt):
            signals.raise_if_died()
        signals.clear_death(key)
        signals.raise_if_died()


def test_clear_death_forgets_only_that_process(lambkin_signals):
    """Each process clears only its own death."""
    with signals.deferred():
        first = lambkin_signals.die()
        second = lambkin_signals.die()
        lambkin_signals.settle()
        signals.clear_death(first)
        assert signals.death_pending()
        signals.clear_death(second)
        assert not signals.death_pending()


def test_worker_thread_does_not_hold_off_the_main_thread(lambkin_signals):
    """A foreground command in a worker thread must not defer the interrupt.

    deferred() and interruptible() only apply to the main thread, where the
    handler runs. A worker never sees the interrupt raised.
    """
    inside = threading.Event()
    release = threading.Event()
    worker_errors: list[BaseException] = []

    def _worker():
        try:
            with signals.deferred():
                inside.set()
                release.wait(5)
                with signals.interruptible():
                    signals.raise_if_died()
        except BaseException as e:  # noqa: BLE001 - reported to the test
            worker_errors.append(e)

    worker = lambkin_signals.start_unsignalled(_worker)
    assert inside.wait(5)
    start = time.monotonic()
    try:
        with pytest.raises(LambkinSIGUSR1Interrupt):
            lambkin_signals.die()
            time.sleep(5)
        assert time.monotonic() - start < 2
    finally:
        release.set()
        worker.join()
    assert worker_errors == []
    assert signals._deferral_depth == 0


def test_handler_does_not_deadlock_on_the_death_lock():
    """The handler reads the deaths while the main thread may hold their lock."""
    script = """
import os, signal, threading, time
from lambkin.common import signals
from lambkin.common.exceptions import LambkinSIGUSR1Interrupt

signals.setup()
key = object()
with signals._dead_lock:
    signals._dead.add(key)
    signals.sigusr1_pending.set()
    try:
        os.kill(os.getpid(), signal.SIGUSR1)
        time.sleep(5)
    except LambkinSIGUSR1Interrupt:
        print("interrupted")
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "interrupted"
