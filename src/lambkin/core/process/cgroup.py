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
"""cgroup v2 utilities for process lifecycle management in lambkin benchmarks."""

from __future__ import annotations

import errno
import os
import signal
import sys
import time
import uuid
from pathlib import Path

from lambkin.common import defaults


def find_delegated_cgroup() -> Path:
    """Return the delegated cgroup for the current process.

    Returns:
    -------
    Path
        The cgroup directory for the current process.

    Raises:
    ------
    RuntimeError
        If no cgroup v2 directory is found.
    """
    cgroup_file = Path("/proc/self/cgroup")
    for line in cgroup_file.read_text().splitlines():
        if line.startswith("0::"):
            rel = line[3:].strip()
            candidate = Path("/sys/fs/cgroup") / rel.lstrip("/")
            if not candidate.exists():
                raise RuntimeError(
                    f"cgroup v2 directory not found: {candidate}. "
                    "Make sure cgroup v2 is enabled on this system."
                )
            if not os.access(candidate, os.W_OK):
                raise RuntimeError(
                    f"No write access to cgroup: {candidate}. "
                    "Run the benchmark with: uv run lambkin <your_benchmark.py>, "
                    "or inside a container using Podman with --systemd=always "
                    "or Docker with --privileged."
                )
            return candidate
    raise RuntimeError(
        "No cgroup v2 found in /proc/self/cgroup. "
        "Make sure cgroup v2 is enabled on this system."
    )


def find_app_slice() -> Path | None:
    """Return the user app.slice cgroup if writable, else None.

    Returns:
    -------
    Path or None
        The app.slice cgroup path if it exists and is writable, else None.
    """
    uid = os.getuid()
    app_slice = Path(
        f"/sys/fs/cgroup/user.slice/user-{uid}.slice/user@{uid}.service/app.slice"
    )
    if app_slice.exists() and os.access(app_slice, os.W_OK):
        return app_slice
    return None


def make_cgroup(parent: Path, name: str) -> Path:
    """Create a child cgroup under parent and return its path.

    Parameters
    ----------
    parent : Path
        The parent cgroup directory.
    name : str
        Name for the new child cgroup.

    Returns:
    -------
    Path
        The path to the newly created cgroup directory.
    """
    child = parent / name
    child.mkdir(exist_ok=True)
    return child


def kill_cgroup(cgroup: Path, grace_period: float = 3.0) -> None:
    """Terminate all processes in a cgroup with a graduated signal sequence.

    Sends SIGTERM to all members first, waits for the grace period, then
    sends SIGKILL to any survivors.

    Parameters
    ----------
    cgroup : Path
        The cgroup directory to kill.
    grace_period : float
        Seconds to wait after SIGTERM before sending SIGKILL.
    """
    procs_file = cgroup / "cgroup.procs"

    for pid_str in procs_file.read_text().split():
        try:
            os.kill(int(pid_str), signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + grace_period
    while time.monotonic() < deadline:
        if not procs_file.read_text().strip():
            return
        time.sleep(defaults.CGROUP_POLL_INTERVAL)

    for pid_str in procs_file.read_text().split():
        try:
            os.kill(int(pid_str), signal.SIGKILL)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + defaults.SIGKILL_GRACE_PERIOD
    while time.monotonic() < deadline:
        if not procs_file.read_text().strip():
            return
        time.sleep(defaults.CGROUP_POLL_INTERVAL)
    survivors = procs_file.read_text().split()
    if survivors:
        print(
            f"[WARNING] {len(survivors)} process(es) survived SIGKILL in "
            f"{cgroup} — this indicates a serious system problem. "
            f"PIDs: {', '.join(survivors)}",
            file=sys.stderr,
        )


def remove_cgroup(cgroup: Path) -> None:
    """Remove a cgroup directory once it is empty.

    Parameters
    ----------
    cgroup : Path
        The cgroup directory to remove.
    """
    deadline = time.monotonic() + defaults.SIGKILL_GRACE_PERIOD
    while time.monotonic() < deadline:
        try:
            cgroup.rmdir()
            return
        except OSError as e:
            if e.errno in (errno.ENOTEMPTY, errno.EBUSY):
                time.sleep(defaults.CGROUP_POLL_INTERVAL)
            elif e.errno == errno.ENOENT:
                return
            else:
                raise
    print(
        f"[WARNING] Could not remove cgroup {cgroup} — "
        "it may still contain processes that survived SIGKILL.",
        file=sys.stderr,
    )


def kill_cgroup_tree(cgroup: Path, grace_period: float = 3.0) -> None:
    """Kill all processes in a cgroup tree recursively.

    Sends SIGKILL to all processes in the tree via ``cgroup.kill``
    and waits for them to die.
    ...
    """
    (cgroup / "cgroup.kill").write_text("1")
    deadline = time.monotonic() + grace_period
    while time.monotonic() < deadline:
        if not (cgroup / "cgroup.threads").read_text().strip():
            return
        time.sleep(defaults.CGROUP_POLL_INTERVAL)


def remove_cgroup_tree(cgroup: Path) -> None:
    """Remove a cgroup directory and all its descendants recursively.

    Does not kill processes — call kill_cgroup_tree first if needed.
    ...
    """
    for child in sorted(cgroup.iterdir(), reverse=True):
        if child.is_dir():
            remove_cgroup_tree(child)
    remove_cgroup(cgroup)


def make_iteration_cgroup(delegated: Path, iteration_dir: Path) -> Path:
    """Create a cgroup for one benchmark iteration.

    Parameters
    ----------
    delegated : Path
        The delegated cgroup for this process.
    iteration_dir : Path
        The iteration output directory, used to derive a unique cgroup name.

    Returns:
    -------
    Path
        The path to the newly created iteration cgroup directory.
    """
    name = (
        f"iter-{iteration_dir.parent.name}-{iteration_dir.name}-{uuid.uuid4().hex[:8]}"
    )
    return make_cgroup(delegated, name)


def make_process_cgroup(parent: Path, argv: list[str]) -> Path:
    """Create a cgroup for a background process.

    Parameters
    ----------
    parent : Path
        The parent cgroup directory (typically the iteration cgroup).
    argv : list[str]
        The command argv, used to derive a human-readable cgroup name.

    Returns:
    -------
    Path
        The path to the newly created cgroup directory.
    """
    name = f"{argv[0].split('/')[-1]}-{uuid.uuid4().hex[:8]}"
    return make_cgroup(parent, name)


def cgroup_exists(cgroup: Path) -> bool:
    """Return True if the cgroup directory exists."""
    return cgroup.exists()
