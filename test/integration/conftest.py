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

"""Shared fixtures for the integration tests."""

import time
from collections.abc import Callable
from pathlib import Path

import pytest


def _running(pid: int) -> bool:
    """Return whether pid is a live process.

    A killed process whose parent is gone is reparented to PID 1, and stays a
    zombie until PID 1 reaps it. In a container whose PID 1 is not an init,
    as in CI's ``podman run ... bash -c "... && uv run pytest"``, that never
    happens. A zombie has exited and left its cgroup, so it counts as gone.
    """
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except FileNotFoundError:
        return False
    return stat.rsplit(") ", 1)[1][0] != "Z"


def _wait_gone(pid: int, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _running(pid):
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def wait_gone() -> Callable[..., bool]:
    """Return a function that waits up to a timeout for a process to be gone."""
    return _wait_gone
