#!/usr/bin/env python3
"""Test bench: graduated signal sequence for cgroup v2 process termination.

Scenarios:
    1. cooperative_sigterm    — cooperative process exits cleanly on SIGTERM
    2. sigkill_direct         — cooperative process killed directly via cgroup.kill
    3. non_cooperative        — SIGTERM-ignoring process killed after grace period
    4. tree_graduated         — whole process tree cleaned up via graduated sequence

HOW TO RUN:
    systemd-run --user --scope -- python3 bench_signal_sequence.py
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

GRACE_PERIOD = 3.0
RESULTS: list[tuple[str, str, str]] = []


def log(msg: str) -> None:
    """Print an indented log message, flushing stdout immediately."""
    print(f"  {msg}", flush=True)


def record(scenario: str, expected: str, observed: str) -> None:
    """Append a (scenario, expected, observed) result tuple to RESULTS."""
    RESULTS.append((scenario, expected, observed))


def find_cgroup_v2_dir() -> Path:
    """Return a writable cgroup v2 directory for this process."""
    cgroup_file = Path("/proc/self/cgroup")
    for line in cgroup_file.read_text().splitlines():
        if line.startswith("0::"):
            rel = line[3:].strip()
            candidate = Path("/sys/fs/cgroup") / rel.lstrip("/")
            if candidate.exists() and os.access(candidate, os.W_OK):
                return candidate
    raise RuntimeError(
        "No writable cgroup v2 directory found. "
        "Run with: systemd-run --user --scope -- python3 bench_signal_sequence.py"
    )


def make_step_cgroup(root: Path, name: str) -> Path:
    """Create a fresh child cgroup under root and return its path."""
    step = root / name
    step.mkdir(exist_ok=True)
    return step


def teardown_cgroup(step: Path) -> bool:
    """Remove the cgroup directory. Returns True if it was empty."""
    try:
        step.rmdir()
        return True
    except OSError:
        return False


def cgroup_kill_graduated(step: Path) -> None:
    """Send SIGTERM to all cgroup members, wait grace period, SIGKILL survivors."""
    procs_file = step / "cgroup.procs"

    log("sending SIGTERM to all cgroup members")
    for pid_str in procs_file.read_text().split():
        try:
            os.kill(int(pid_str), signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + GRACE_PERIOD
    while time.monotonic() < deadline:
        if not procs_file.read_text().strip():
            return
        time.sleep(0.05)

    remaining = procs_file.read_text().split()
    if remaining:
        log(f"grace period elapsed, sending SIGKILL to {len(remaining)} survivor(s)")
        for pid_str in remaining:
            try:
                os.kill(int(pid_str), signal.SIGKILL)
            except ProcessLookupError:
                pass

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not procs_file.read_text().strip():
            return
        time.sleep(0.05)


# ── process payloads ──────────────────────────────────────────────────────────

COOPERATIVE = """
import signal, sys, time, pathlib

lock = pathlib.Path(sys.argv[1])
lock.touch()

def handler(sig, frame):
    lock.unlink(missing_ok=True)
    sys.exit(0)

signal.signal(signal.SIGTERM, handler)
while True:
    time.sleep(0.1)
"""

NON_COOPERATIVE = """
import signal, sys, time, pathlib

lock = pathlib.Path(sys.argv[1])
lock.touch()

signal.signal(signal.SIGTERM, signal.SIG_IGN)
while True:
    time.sleep(0.1)
"""

COOPERATIVE_TREE = """
import signal, subprocess, sys, time, pathlib

lock = pathlib.Path(sys.argv[1])
child_lock = pathlib.Path(sys.argv[2])
lock.touch()

child_payload = (
    "import signal, sys, time, pathlib\\n"
    "lock = pathlib.Path(sys.argv[1])\\n"
    "lock.touch()\\n"
    "def handler(sig, frame):\\n"
    "    lock.unlink(missing_ok=True)\\n"
    "    sys.exit(0)\\n"
    "signal.signal(signal.SIGTERM, handler)\\n"
    "while True: time.sleep(0.1)\\n"
)

child = subprocess.Popen([sys.executable, '-c', child_payload, str(child_lock)])

def handler(sig, frame):
    lock.unlink(missing_ok=True)
    sys.exit(0)

signal.signal(signal.SIGTERM, handler)
while True:
    time.sleep(0.1)
"""


# ── scenarios ────────────────────────────────────────────────────────────────


def scenario_cooperative_sigterm(cgroup_root: Path) -> None:
    """Run scenario 1: a cooperative process that exits cleanly on SIGTERM."""
    step = make_step_cgroup(cgroup_root, "bench_s1")

    with tempfile.NamedTemporaryFile(delete=False, suffix=".lock") as f:
        lock = Path(f.name)

    def preexec():
        (step / "cgroup.procs").write_text(str(os.getpid()))

    proc = subprocess.Popen(
        [sys.executable, "-c", COOPERATIVE, str(lock)],
        preexec_fn=preexec,
    )
    time.sleep(0.3)
    assert lock.exists()
    log(f"lock file created: {lock}")

    cgroup_kill_graduated(step)
    proc.wait()

    cleaned = not lock.exists()
    cgroup_empty = teardown_cgroup(step)

    log(f"process cleaned up: {cleaned}")
    log(f"cgroup empty: {cgroup_empty}")
    lock.unlink(missing_ok=True)

    if cleaned and cgroup_empty:
        record(
            "cooperative_sigterm",
            "cleaned up, cgroup empty",
            "🟢 cleaned up, cgroup empty",
        )
    else:
        record("cooperative_sigterm", "cleaned up, cgroup empty", "🔴 not cleaned up")


def scenario_sigkill_direct(cgroup_root: Path) -> None:
    """Run scenario 2: a cooperative process killed immediately via cgroup.kill."""
    step = make_step_cgroup(cgroup_root, "bench_s2")

    with tempfile.NamedTemporaryFile(delete=False, suffix=".lock") as f:
        lock = Path(f.name)

    def preexec():
        (step / "cgroup.procs").write_text(str(os.getpid()))

    proc = subprocess.Popen(
        [sys.executable, "-c", COOPERATIVE, str(lock)],
        preexec_fn=preexec,
    )
    time.sleep(0.3)
    assert lock.exists()
    log(f"lock file created: {lock}")

    kill_file = step / "cgroup.kill"
    if kill_file.exists():
        kill_file.write_text("1")
    else:
        log("cgroup.kill not available, falling back to iterative SIGKILL")
        for pid_str in (step / "cgroup.procs").read_text().split():
            try:
                os.kill(int(pid_str), signal.SIGKILL)
            except ProcessLookupError:
                pass

    proc.wait()
    cgroup_empty = teardown_cgroup(step)

    lock_remains = lock.exists()
    log(f"lock file remains: {lock_remains}")
    log(f"cgroup empty: {cgroup_empty}")
    lock.unlink(missing_ok=True)

    if lock_remains and cgroup_empty:
        record(
            "sigkill_direct",
            "not cleaned up, cgroup empty",
            "🟢 not cleaned up, cgroup empty",
        )
    else:
        record(
            "sigkill_direct", "not cleaned up, cgroup empty", "🔴 unexpected outcome"
        )


def scenario_non_cooperative(cgroup_root: Path) -> None:
    """Run scenario 3: a SIGTERM-ignoring process killed after the grace period."""
    print(
        "\nScenario 3 — Non-cooperative process,"
        " SIGTERM + grace period + SIGKILL via cgroup"
    )
    step = make_step_cgroup(cgroup_root, "bench_s3")

    with tempfile.NamedTemporaryFile(delete=False, suffix=".lock") as f:
        lock = Path(f.name)

    def preexec():
        (step / "cgroup.procs").write_text(str(os.getpid()))

    proc = subprocess.Popen(
        [sys.executable, "-c", NON_COOPERATIVE, str(lock)],
        preexec_fn=preexec,
    )
    time.sleep(0.3)
    assert lock.exists()
    log(f"lock file created: {lock}")

    cgroup_kill_graduated(step)
    proc.wait()

    cgroup_empty = teardown_cgroup(step)
    lock_remains = lock.exists()

    log(f"process dead: {proc.poll() is not None}")
    log(f"lock file remains: {lock_remains}")
    log(f"cgroup empty: {cgroup_empty}")
    lock.unlink(missing_ok=True)

    if cgroup_empty and lock_remains:
        record(
            "non_cooperative_sigterm",
            "dead, not cleaned up, cgroup empty",
            "🟢 dead, not cleaned up, cgroup empty",
        )
    else:
        record(
            "non_cooperative_sigterm",
            "dead, not cleaned up, cgroup empty",
            "🔴 unexpected outcome",
        )


def scenario_tree_graduated(cgroup_root: Path) -> None:
    """Run scenario 4.

    Process tree terminated via graduated SIGTERM/SIGKILL
    sequence.
    """
    step = make_step_cgroup(cgroup_root, "bench_s4")

    with tempfile.NamedTemporaryFile(delete=False, suffix=".lock") as f:
        lock_parent = Path(f.name)
    with tempfile.NamedTemporaryFile(delete=False, suffix=".lock") as f:
        lock_child = Path(f.name)

    def preexec():
        (step / "cgroup.procs").write_text(str(os.getpid()))

    proc = subprocess.Popen(
        [sys.executable, "-c", COOPERATIVE_TREE, str(lock_parent), str(lock_child)],
        preexec_fn=preexec,
    )
    time.sleep(0.5)
    assert lock_parent.exists() and lock_child.exists()
    log(f"parent lock: {lock_parent}")
    log(f"child lock: {lock_child}")

    members = (step / "cgroup.procs").read_text().split()
    log(f"cgroup members: {len(members)} processes")

    cgroup_kill_graduated(step)
    proc.wait()

    cgroup_empty = teardown_cgroup(step)
    parent_cleaned = not lock_parent.exists()
    child_cleaned = not lock_child.exists()

    log(f"parent cleaned up: {parent_cleaned}")
    log(f"child cleaned up: {child_cleaned}")
    log(f"cgroup empty: {cgroup_empty}")

    for p in [lock_parent, lock_child]:
        p.unlink(missing_ok=True)

    if parent_cleaned and child_cleaned and cgroup_empty:
        record(
            "tree_graduated",
            "whole tree cleaned up, cgroup empty",
            "🟢 whole tree cleaned up, cgroup empty",
        )
    elif cgroup_empty:
        record(
            "tree_graduated",
            "whole tree cleaned up, cgroup empty",
            "🔴 cgroup empty but cleanup incomplete",
        )
    else:
        record(
            "tree_graduated",
            "whole tree cleaned up, cgroup empty",
            "🔴 cgroup not empty",
        )


# ── summary ──────────────────────────────────────────────────────────────────


def print_summary() -> None:
    """Print a formatted table of all scenario results."""
    print("\n" + "─" * 75)
    print(f"  {'Scenario':<30} {'Expected':<30} {'Observed'}")
    print("─" * 75)
    for scenario, expected, observed in RESULTS:
        print(f"  {scenario:<30} {expected:<30} {observed}")
    print("─" * 75)


if __name__ == "__main__":
    try:
        cgroup_root = find_cgroup_v2_dir()
    except RuntimeError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    log(f"using cgroup root: {cgroup_root}")

    scenario_cooperative_sigterm(cgroup_root)
    scenario_sigkill_direct(cgroup_root)
    scenario_non_cooperative(cgroup_root)
    scenario_tree_graduated(cgroup_root)
    print_summary()
