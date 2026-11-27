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

"""Fixtures shared by the whole test suite."""

import signal
import threading
import time
from collections.abc import Iterator

import pytest

from lambkin.common import signals


class Deaths:
    """Record background deaths the way a BackgroundProcess monitor does."""

    def __init__(self) -> None:
        """Start with no deaths and no forwarded signals."""
        self.forwarded: list[int] = []
        self._threads: list[threading.Thread] = []

    @staticmethod
    def start_unsignalled(target, *args) -> threading.Thread:
        """Start a thread that never receives SIGUSR1, as monitor threads do.

        A process-directed SIGUSR1 must reach the main thread for a blocking
        call there, such as ``time.sleep``, to be interrupted by the handler.

        Returns:
            The started thread.
        """
        thread = threading.Thread(target=target, args=args, daemon=True)
        old_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGUSR1})
        try:
            thread.start()
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, old_mask)
        return thread

    @staticmethod
    def settle() -> None:
        """Give the main thread time to run a pending signal handler."""
        time.sleep(0.2)

    def die(self, key: object | None = None) -> object:
        """Record a death and signal it from another thread; wait until sent.

        The handler may raise LambkinSIGUSR1Interrupt from inside this call.

        Args:
            key: The dead process, or None for a new one.

        Returns:
            The key passed to ``signals.notify_death``.
        """
        key = object() if key is None else key
        thread = self.start_unsignalled(signals.notify_death, key)
        self._threads.append(thread)
        thread.join()
        return key

    def die_after(self, seconds: float) -> object:
        """Record a death and signal it after a delay, without waiting."""
        key = object()

        def _later() -> None:
            time.sleep(seconds)
            signals.notify_death(key)

        self._threads.append(self.start_unsignalled(_later))
        return key

    def join(self) -> None:
        """Wait for every pending notification to be sent."""
        for thread in self._threads:
            thread.join()


@pytest.fixture
def lambkin_signals() -> Iterator[Deaths]:
    """Install lambkin's SIGUSR1 handler and reset its state afterwards.

    The handler forwards unrelated signals to a recorder, so a test sees
    them in ``Deaths.forwarded`` instead of the process being killed.

    Yields:
        The helper that records deaths.
    """
    deaths = Deaths()
    previous = signal.signal(
        signal.SIGUSR1,
        signals._make_handler(lambda signum, frame: deaths.forwarded.append(signum)),
    )
    try:
        yield deaths
    finally:
        deaths.join()
        # Drain signals still on their way without raising in teardown.
        signal.signal(signal.SIGUSR1, signal.SIG_IGN)
        deaths.settle()
        signal.signal(signal.SIGUSR1, previous)
        leaked_depth = signals._deferral_depth
        leaked_interruptible = signals._interruptible
        signals._dead.clear()
        signals.sigusr1_pending.clear()
        signals._deferral_depth = 0
        signals._interruptible = False
    assert leaked_depth == 0
    assert not leaked_interruptible
