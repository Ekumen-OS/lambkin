"""Shared signalling state for inter-thread communication.

Provides a pending flag that BackgroundProcess monitor threads set before
sending SIGUSR1 to the main thread, allowing the signal handler to distinguish
lambkin-originated signals from unrelated SIGUSR1s sent by other processes.

The SIGUSR1 handler is registered in the benchmark script process, not in the
CLI, because the CLI launches the script as a separate subprocess. BackgroundProcess
and its monitor thread live in the script's process, so the signal is sent and
handled there.
"""

import signal
import threading

from lambkin.common.exceptions import LambkinSIGUSR1Interrupt

sigusr1_pending = threading.Event()


def _make_handler(previous):
    def _handle_sigusr1(signum, frame):
        if sigusr1_pending.is_set():
            sigusr1_pending.clear()
            raise LambkinSIGUSR1Interrupt
        elif callable(previous):
            previous(signum, frame)

    return _handle_sigusr1


def setup():
    """Register the SIGUSR1 handler for the benchmark script process."""
    previous = signal.signal(signal.SIGUSR1, signal.SIG_DFL)
    signal.signal(signal.SIGUSR1, _make_handler(previous))
