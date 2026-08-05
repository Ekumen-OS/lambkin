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

The SIGUSR1 handler is registered in the benchmark script process, not in the
CLI, because the CLI launches the script as a separate subprocess. BackgroundProcess
and its monitor thread live in the script's process, so the signal is sent and
handled there.
"""

import os
import signal
import threading
from collections.abc import Callable
from types import FrameType

from lambkin.common.exceptions import LambkinSIGUSR1Interrupt

sigusr1_pending = threading.Event()

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
