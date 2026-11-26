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

import errno
import logging
import os
import signal
import subprocess
import time
import uuid
from collections.abc import Callable
from pathlib import Path
from typing import Any

from lambkin.common import defaults, exceptions

logger = logging.getLogger(__name__)


def find_delegated_cgroup() -> Path:
    """Return the delegated cgroup for the current process.

    Returns:
        Path: The cgroup directory for the current process.

    Raises:
        RuntimeError: If no cgroup v2 directory is found.
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
        Path | None: The app.slice cgroup path if it exists and is writable,
            else None.
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

    Args:
        parent (Path): The parent cgroup directory.
        name (str): Name for the new child cgroup.

    Returns:
        Path: The path to the newly created cgroup directory.
    """
    child = parent / name
    child.mkdir(exist_ok=True)
    return child


def cgroup_joiner(cgroup: Path) -> Callable[[], None]:
    """Return a ``subprocess.Popen`` ``preexec_fn`` that joins a cgroup.

    The returned function writes ``"0"`` to the cgroup's ``cgroup.procs``,
    which the kernel reads as "the writing process". It runs between
    ``fork()`` and ``exec()`` while other threads of the parent may hold
    locks, so the path is resolved here, in the parent, and the child only
    makes raw ``os`` calls.

    Args:
        cgroup (Path): The cgroup directory to join.

    Returns:
        A function that moves the calling process into ``cgroup``.
    """
    procs = os.fsencode(cgroup / "cgroup.procs")

    def _join() -> None:
        fd = os.open(procs, os.O_WRONLY)
        try:
            os.write(fd, b"0")
        finally:
            os.close(fd)

    return _join


def kill_cgroup(cgroup: Path, grace_period: float = 3.0) -> None:
    """Terminate all processes in a cgroup with a graduated signal sequence.

    Sends SIGTERM to all members first, waits for the grace period, then
    sends SIGKILL to any survivors.

    Args:
        cgroup (Path): The cgroup directory to kill.
        grace_period (float): Seconds to wait after SIGTERM before sending SIGKILL.
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
        logger.warning(
            "%d process(es) survived SIGKILL in %s — this indicates a "
            "serious system problem. PIDs: %s",
            len(survivors),
            cgroup,
            ", ".join(survivors),
        )


def remove_cgroup(cgroup: Path) -> None:
    """Remove a cgroup directory once it is empty.

    Args:
        cgroup (Path): The cgroup directory to remove.

    Raises:
        OSError: If an unexpected OS error occurs while removing the directory.
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
    logger.warning(
        "Could not remove cgroup %s — it may still contain processes "
        "that survived SIGKILL.",
        cgroup,
    )


def kill_cgroup_tree(cgroup: Path, grace_period: float = 3.0) -> None:
    """Kill all processes in a cgroup tree recursively.

    Sends SIGKILL to all processes in the tree via the ``cgroup.kill``
    interface, which propagates to all descendants automatically, then
    waits for all threads to exit.

    Args:
        cgroup (Path): The root cgroup directory to kill.
        grace_period (float): Seconds to wait for all processes to die after SIGKILL.
    """
    (cgroup / "cgroup.kill").write_text("1")
    deadline = time.monotonic() + grace_period
    while time.monotonic() < deadline:
        if not (cgroup / "cgroup.threads").read_text().strip():
            return
        time.sleep(defaults.CGROUP_POLL_INTERVAL)


def remove_cgroup_tree(cgroup: Path) -> None:
    """Remove a cgroup directory and all its descendants recursively.

    Removes child cgroups depth-first before removing the root.
    Does not kill processes — call ``kill_cgroup_tree`` first if needed.

    Args:
        cgroup (Path): The root cgroup directory to remove.
    """
    for child in _child_cgroups(cgroup):
        remove_cgroup_tree(child)
    remove_cgroup(cgroup)


def _child_cgroups(cgroup: Path) -> list[Path]:
    """Return a cgroup's child cgroups.

    Uses the directory entries' types instead of a ``stat`` per entry, as
    most entries are interface files.

    Args:
        cgroup (Path): The cgroup directory.

    Returns:
        The child cgroup directories.
    """
    with os.scandir(cgroup) as entries:
        return [Path(e.path) for e in entries if e.is_dir(follow_symlinks=False)]


def make_iteration_cgroup(delegated: Path, iteration_dir: Path) -> Path:
    """Create a cgroup for one benchmark iteration.

    Args:
        delegated (Path): The delegated cgroup for this process.
        iteration_dir (Path): The iteration output directory, used to derive
            a unique cgroup name.

    Returns:
        Path: The path to the newly created iteration cgroup directory.
    """
    name = (
        f"iter-{iteration_dir.parent.name}-{iteration_dir.name}-{uuid.uuid4().hex[:8]}"
    )
    return make_cgroup(delegated, name)


def make_process_cgroup(parent: Path, argv: list[str]) -> Path:
    """Create a cgroup for one command, background or foreground.

    Args:
        parent (Path): The parent cgroup directory (typically the iteration cgroup).
        argv (list[str]): The command argv, used to derive a human-readable
            cgroup name.

    Returns:
        Path: The path to the newly created cgroup directory.
    """
    name = f"{argv[0].split('/')[-1]}-{uuid.uuid4().hex[:8]}"
    return make_cgroup(parent, name)


def start_process(
    argv: list[str], cgroup: Path | None = None, **popen_kwargs: Any
) -> subprocess.Popen[bytes]:
    """Start a command, optionally inside an existing cgroup.

    Args:
        argv (list[str]): The command to run.
        cgroup (Path | None): The cgroup the child joins before ``exec()``,
            or None to stay in the caller's cgroup.
        **popen_kwargs: Passed to ``subprocess.Popen``.

    Returns:
        The running process.

    Raises:
        CommandError: If the command is missing or not executable, or the
            child cannot join ``cgroup``.
    """
    preexec_fn = cgroup_joiner(cgroup) if cgroup is not None else None
    try:
        return subprocess.Popen(argv, preexec_fn=preexec_fn, **popen_kwargs)
    except subprocess.SubprocessError as e:
        # Raised by Popen when cgroup_joiner fails in the child.
        raise exceptions.CommandError(
            argv, f"Could not prepare {argv[0]!r} to run: {e}"
        ) from e
    except OSError as e:
        raise exceptions.CommandError(argv, _not_started_message(argv, e)) from e


def _not_started_message(argv: list[str], error: OSError) -> str:
    """Describe why ``subprocess.Popen`` could not run a command.

    Args:
        argv: The command that failed to start.
        error: The OSError ``subprocess.Popen`` raised.

    Returns:
        The message for the CommandError raised in its place.
    """
    # FileNotFoundError and PermissionError are subclasses of OSError.
    if isinstance(error, FileNotFoundError):
        return (
            f"Command not found: {argv[0]!r}. "
            "Make sure it is installed and available on PATH."
        )
    if isinstance(error, PermissionError):
        return (
            f"Permission denied: {argv[0]!r} is not executable. Check file permissions."
        )
    return f"OS error while starting {argv[0]!r}: {error}."


def spawn_in_cgroup(
    parent: Path, argv: list[str], **popen_kwargs: Any
) -> tuple[subprocess.Popen[bytes], Path]:
    """Start a command in a new cgroup of its own under ``parent``.

    If the command cannot be started, the cgroup is emptied and removed
    before the error propagates.

    Args:
        parent (Path): The cgroup the command's cgroup goes under, typically
            the iteration cgroup.
        argv (list[str]): The command to run.
        **popen_kwargs: Passed to ``subprocess.Popen``.

    Returns:
        The running process and its cgroup.

    Raises:
        CommandError: If the cgroup cannot be created, or the command cannot
            be started in it.
    """
    try:
        cgroup = make_process_cgroup(parent, argv)
    except OSError as e:
        raise exceptions.CommandError(
            argv, f"Could not create a cgroup for {argv[0]!r} under {parent}: {e}"
        ) from e
    try:
        proc = start_process(argv, cgroup, **popen_kwargs)
    except BaseException:
        _discard_cgroup(cgroup)
        raise
    return proc, cgroup


def _discard_cgroup(cgroup: Path) -> None:
    """Kill and remove a command's cgroup after the command failed to start.

    Runs while another exception propagates, so it never raises: an
    interrupt restarts it, and an OSError is logged and left to the
    iteration teardown. An interrupt can land after ``fork()``, with the
    child already inside, so the cgroup is destroyed, not just removed.

    Args:
        cgroup (Path): The cgroup to discard.
    """
    while True:
        try:
            destroy_cgroup(cgroup, grace_period=0.0)
            return
        except exceptions.LambkinSIGUSR1Interrupt:
            # A background process died. Finish cleaning up; the error
            # already propagating is what went wrong here first.
            continue
        except OSError as e:
            logger.warning("Could not clean up cgroup %s: %s", cgroup, e)
            return


def destroy_cgroup(cgroup: Path, grace_period: float) -> None:
    """Stop every process in a cgroup tree and remove the tree.

    The cgroup's own members get SIGTERM and ``grace_period`` seconds to
    exit before SIGKILL. Anything left in child cgroups is then killed, and
    the tree is removed deepest first. A cgroup that is already gone, or
    goes away meanwhile, counts as destroyed.

    Args:
        cgroup (Path): The root of the cgroup tree.
        grace_period (float): Seconds between SIGTERM and SIGKILL.
    """
    try:
        kill_cgroup(cgroup, grace_period=grace_period)
        # kill_cgroup only signals the cgroup's own members. Look for child
        # cgroups only now: the members that could create them are gone.
        children = _child_cgroups(cgroup)
        if children:
            kill_cgroup_tree(cgroup)
            for child in children:
                remove_cgroup_tree(child)
        remove_cgroup(cgroup)
    except FileNotFoundError:
        # Removed meanwhile, for example by the iteration teardown.
        return


def cgroup_exists(cgroup: Path) -> bool:
    """Return True if the cgroup directory exists."""
    return cgroup.exists()
