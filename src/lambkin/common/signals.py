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
"""Shared signalling state for inter-thread communication.

Provides a pending flag that BackgroundProcess monitor threads set before
sending SIGUSR1 to the main thread, allowing the signal handler to distinguish
lambkin-originated signals from unrelated SIGUSR1s sent by other processes.

The signal is one-shot, so each death is also remembered until the
BackgroundProcess reports it (see ``notify_death`` and ``death_pending``).
Teardown code runs inside ``deferred()``, where the signal does not interrupt
it; the death is raised as ``LambkinSIGUSR1Interrupt`` once teardown ends.

The SIGUSR1 handler is registered in the benchmark script process, not in the
CLI, because the CLI launches the script as a separate subprocess. BackgroundProcess
and its monitor thread live in the script's process, so the signal is sent and
handled there.
"""

import contextlib
import os
import signal
import threading
from collections.abc import Callable, Iterator
from types import FrameType

from lambkin.common.exceptions import LambkinSIGUSR1Interrupt

sigusr1_pending = threading.Event()

# Background processes that died unexpectedly and have not reported it yet.
# Monitor threads add to it; the main thread reads and discards.
_dead: set[object] = set()
# Reentrant: the signal handler reads _dead on the main thread, which may
# already hold the lock when the signal lands.
_dead_lock = threading.RLock()

# How many deferred() blocks the main thread is in.
_deferral_depth = 0

# True inside interruptible(): the next lambkin SIGUSR1 raises even within
# deferred(). The handler resets it when it raises, so a second signal that
# lands while the first interrupt unwinds is deferred instead.
_interruptible = False

SignalHandler = signal.Handlers | Callable[[int, FrameType | None], None] | int | None


def _make_handler(previous: SignalHandler) -> Callable[[int, FrameType | None], None]:
    """Create a SIGUSR1 handler that raises LambkinSIGUSR1Interrupt if lambkin sent it.

    Args:
        previous: The previous SIGUSR1 handler to forward to if the signal
            was not sent by lambkin.

    Returns:
        A signal handler function.
    """

    def _handle_sigusr1(signum: int, frame: FrameType | None) -> None:
        """Handle SIGUSR1 by interrupting proc.wait() or forwarding the signal.

        Raises LambkinSIGUSR1Interrupt if a background process died and has
        not reported it yet, which unblocks a foreground command waiting in
        run_foreground. A lambkin signal for a death already reported is
        dropped. Any other SIGUSR1 is forwarded to the previous handler to
        avoid swallowing unrelated SIGUSR1s from other processes or libraries.

        Args:
            signum: The signal number received.
            frame: The current stack frame.

        Raises:
            LambkinSIGUSR1Interrupt: If a death is unreported, outside a
                ``deferred()`` block or inside an ``interruptible()`` one.
        """
        global _interruptible
        # A second death's signal can arrive after the first one's handler
        # cleared the flag, so a recorded death also marks the signal as ours.
        if not (sigusr1_pending.is_set() or death_pending()):
            # Restore the previous handler and re-raise the signal to forward it
            # faithfully, whether it is a callable, SIG_DFL, or SIG_IGN.
            signal.signal(signal.SIGUSR1, previous)
            os.kill(os.getpid(), signal.SIGUSR1)
            return
        sigusr1_pending.clear()
        if not death_pending():
            # Delivered late, after its process already reported the death.
            return
        if _deferral_depth and not _interruptible:
            # The death stays in _dead; deferred() raises it on exit.
            return
        _interruptible = False
        raise LambkinSIGUSR1Interrupt()

    return _handle_sigusr1


def setup() -> None:
    """Register the SIGUSR1 handler for the benchmark script process.

    Must be called once before any BackgroundProcess is started. Registered
    in the benchmark script process rather than the CLI because the CLI
    launches the script as a separate subprocess — BackgroundProcess and its
    monitor thread live in the script's process, so the signal is sent and
    handled there.

    Note: Calling this function multiple times will chain handlers, which is
    harmless but unnecessary.
    """
    previous = signal.signal(signal.SIGUSR1, signal.SIG_DFL)
    signal.signal(signal.SIGUSR1, _make_handler(previous))


def notify_death(process: object) -> None:
    """Record that a background process died and interrupt the main thread.

    Called from a monitor thread. The death stays recorded until
    ``clear_death`` is called for the same process, so it is not lost if the
    interrupt lands where it is caught and dropped.

    Args:
        process: The process that died, used as a key for ``clear_death``.
    """
    with _dead_lock:
        _dead.add(process)
    sigusr1_pending.set()
    os.kill(os.getpid(), signal.SIGUSR1)


def clear_death(process: object) -> None:
    """Forget a recorded death once its process has reported it.

    Args:
        process: The process passed to ``notify_death``.
    """
    with _dead_lock:
        _dead.discard(process)


def death_pending() -> bool:
    """Return True if a background process died and has not reported it yet."""
    with _dead_lock:
        return bool(_dead)


def _on_main_thread() -> bool:
    """Return True if called from the main thread, where the handler runs."""
    return threading.current_thread() is threading.main_thread()


def raise_if_died() -> None:
    """Raise if a background process died and has not reported it yet.

    For code about to block on the main thread, such as a foreground
    command, when the death's interrupt may already be used up. Does
    nothing on other threads, which the interrupt never reaches.

    Raises:
        LambkinSIGUSR1Interrupt: If a death is recorded.
    """
    if _on_main_thread() and death_pending():
        raise LambkinSIGUSR1Interrupt()


@contextlib.contextmanager
def deferred(raise_on_exit: bool = True) -> Iterator[None]:
    """Hold off ``LambkinSIGUSR1Interrupt`` while the block runs.

    For teardown that must run to completion once started. A background
    death during the block is raised as ``LambkinSIGUSR1Interrupt`` when the
    outermost block ends, unless an exception is already leaving it or
    ``raise_on_exit`` is False; the death then stays recorded for whoever
    checks ``death_pending`` next. On other threads, which the interrupt
    never reaches, it does nothing.

    Args:
        raise_on_exit: False for teardown that runs while another exception
            propagates, which a death must not replace.

    Yields:
        None.

    Raises:
        LambkinSIGUSR1Interrupt: On exit, if a background death is still
            unreported, no other exception is propagating and
            ``raise_on_exit`` is True.
    """
    global _deferral_depth
    if not _on_main_thread():
        yield
        return
    _deferral_depth += 1
    try:
        yield
    finally:
        _deferral_depth -= 1
    if raise_on_exit and not _deferral_depth and death_pending():
        raise LambkinSIGUSR1Interrupt()


@contextlib.contextmanager
def interruptible() -> Iterator[None]:
    """Let a background death interrupt the block, even inside ``deferred()``.

    For the one blocking wait inside a deferred section, such as a
    foreground command's ``proc.wait()``. Entering the block already raises
    if a death is recorded, since its signal may have been deferred before.
    Only the first interrupt is raised; one that lands while it unwinds is
    deferred. On other threads it does nothing.

    Yields:
        None.

    Raises:
        LambkinSIGUSR1Interrupt: On entry if a death is already recorded, or
            from the signal handler while the block runs.
    """  # noqa: DOC502 - raised by raise_if_died and the signal handler
    global _interruptible
    if not _on_main_thread():
        yield
        return
    _interruptible = True
    try:
        # After setting the flag: a death recorded before it was set had its
        # signal deferred, one recorded after raises from the handler.
        raise_if_died()
        yield
    finally:
        _interruptible = False
