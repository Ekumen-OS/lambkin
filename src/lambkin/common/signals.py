"""Shared signalling state for inter-thread communication.

Provides a pending flag that BackgroundProcess monitor threads set before
sending SIGUSR1 to the main thread, allowing the signal handler to distinguish
lambkin-originated signals from unrelated SIGUSR1s sent by other processes.

The SIGUSR1 handler is registered in the benchmark script process, not in the
CLI, because the CLI launches the script as a separate subprocess. BackgroundProcess
and its monitor thread live in the script's process, so the signal is sent and
handled there.
"""

import os
import signal
import threading

from lambkin.common.exceptions import LambkinSIGUSR1Interrupt

sigusr1_pending = threading.Event()


def _make_handler(previous):
    """Create a SIGUSR1 handler that raises LambkinSIGUSR1Interrupt if lambkin sent it.

    Args:
        previous: The previous SIGUSR1 handler to forward to if the signal
            was not sent by lambkin.

    Returns:
        A signal handler function.
    """

    def _handle_sigusr1(signum, frame):
        """Handle SIGUSR1 by interrupting proc.wait() or forwarding the signal.

        Raises LambkinSIGUSR1Interrupt if lambkin set the pending flag, which
        unblocks any foreground process waiting in CommandProxy.__call__.
        Otherwise forwards to the previous handler to avoid swallowing
        unrelated SIGUSR1s from other processes or libraries.

        Args:
            signum: The signal number received.
            frame: The current stack frame.

        Raises:
            LambkinSIGUSR1Interrupt: If lambkin set the pending flag before
                sending SIGUSR1.
        """
        if sigusr1_pending.is_set():
            sigusr1_pending.clear()
            raise LambkinSIGUSR1Interrupt()
        else:
            # Restore the previous handler and re-raise the signal to forward it
            # faithfully, whether it is a callable, SIG_DFL, or SIG_IGN.
            signal.signal(signal.SIGUSR1, previous)
            os.kill(os.getpid(), signal.SIGUSR1)

    return _handle_sigusr1


def setup():
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
