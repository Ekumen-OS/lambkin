#!/usr/bin/env python3
"""Test bench: POSIX process groups (setpgid + killpg).

Scenarios:
    1. simple           — LAUNCHER only (baseline)
    2. deep             — 3-level cooperative tree
    3. setpgid          — GRANDCHILD calls setpgid(0,0)
    4. setsid           — GRANDCHILD calls setsid()
    5. ros2_full_stack  — ros2 launch + bag play + bag record (real workload)

HOW TO RUN:
    python3 bench_process_groups.py
"""

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROS2_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = "$HOME/ekumen/lambkin/ws/install/setup.bash"
LAUNCH_PACKAGE = "beluga_ros2"
LAUNCH_FILE = "beluga.launch.py"
MAP_PATH = "$HOME/ekumen/lambkin/examples/map/"
BAG_PATH = "$HOME/ekumen/lambkin/record_1/"
RECORD_TOPICS = ["/tf"]
RUN_DURATION = 30

ROS2_PATTERNS = [
    "ros2",
    "_ros2_daemon",
    "beluga",
    "amcl_node",
    "map_server",
    "lifecycle_manag",
    "nav2",
    "fastdds",
    "cyclone",
    "rmw",
    "rosbag2",
]


def ros2_env():
    """Source ROS 2 setup files and return the resulting environment as a dict.

    Sources each setup file listed in ``ROS2_SETUP`` and ``WS_SETUP`` in a
    bash subprocess and captures the resulting environment variables. The
    returned dict can be passed directly as the ``env`` argument to
    ``subprocess.Popen`` so that ROS 2 commands resolve correctly.

    Returns:
    -------
    dict
        Mapping of environment variable names to their values after sourcing
        the ROS 2 and workspace setup scripts.
    """
    setups = [s for s in [ROS2_SETUP, WS_SETUP] if s]
    source_cmd = " && ".join(f"source {s}" for s in setups)
    result = subprocess.run(
        ["bash", "-c", f"{source_cmd} && env"], capture_output=True, text=True
    )
    env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v
    return env


def proc_name(pid):
    """Return the short command name of a process from ``/proc/<pid>/comm``.

    Parameters
    ----------
    pid : int
        PID of the process to query.

    Returns:
    -------
    str
        The contents of ``/proc/<pid>/comm``, stripped of whitespace,
        or ``'<gone>'`` if the process no longer exists.
    """
    try:
        return open(f"/proc/{pid}/comm").read().strip()
    except FileNotFoundError:
        return "<gone>"


def proc_cmdline(pid):
    """Return the full command line of a process from ``/proc/<pid>/cmdline``.

    Null bytes used as argument separators in the raw file are replaced with
    spaces to produce a human-readable string.

    Parameters
    ----------
    pid : int
        PID of the process to query.

    Returns:
    -------
    str
        The command line of the process with null bytes replaced by spaces,
        or ``'<gone>'`` if the process no longer exists.
    """
    try:
        return open(f"/proc/{pid}/cmdline").read().replace("\x00", " ").strip()
    except FileNotFoundError:
        return "<gone>"


def proc_ppid(pid):
    """Return the parent PID of a process by reading ``/proc/<pid>/status``.

    Parameters
    ----------
    pid : int
        PID of the process to query.

    Returns:
    -------
    int
        The parent PID of the process, or ``0`` if the process no longer
        exists or the status file cannot be read.
    """
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("PPid:"):
                    return int(line.split()[1])
    except FileNotFoundError:
        pass
    return 0


def alive(pid):
    """Return ``True`` if a process exists and is not a zombie.

    Parameters
    ----------
    pid : int
        PID of the process to check.

    Returns:
    -------
    bool
        ``True`` if the process exists and its state is not ``'Z'`` (zombie),
        ``False`` otherwise.
    """
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("State:"):
                    return "Z" not in line
        return False
    except (FileNotFoundError, PermissionError):
        return False


def get_descendants(root_pid):
    """Walk ``/proc`` and return all descendants of a given PID via BFS.

    Builds a parent-to-children mapping from every readable entry in
    ``/proc`` and then performs a breadth-first search starting from
    ``root_pid``. The root itself is included in the result.

    Parameters
    ----------
    root_pid : int
        PID of the root process whose descendants are to be collected.

    Returns:
    -------
    list of int
        All PIDs in the subtree rooted at ``root_pid``, including the root.
    """
    children_of = {}
    for pid_str in os.listdir("/proc"):
        if not pid_str.isdigit():
            continue
        pid = int(pid_str)
        ppid = proc_ppid(pid)
        if ppid:
            children_of.setdefault(ppid, []).append(pid)
    result, stack, seen = [], [root_pid], set()
    while stack:
        pid = stack.pop()
        if pid in seen:
            continue
        seen.add(pid)
        result.append(pid)
        stack.extend(children_of.get(pid, []))
    return result


def scan_ros2_processes(my_uid=None):
    """Scan ``/proc`` for ROS 2-related processes owned by the current user.

    A process is considered ROS 2-related if any substring in
    ``ROS2_PATTERNS`` appears in its lower-cased command line.

    Parameters
    ----------
    my_uid : int, optional
        UID to filter processes by. Defaults to the UID of the current
        process if not provided.

    Returns:
    -------
    set of int
        PIDs of all ROS 2-related processes owned by ``my_uid``.
    """
    if my_uid is None:
        my_uid = os.getuid()
    found = set()
    for pid_str in os.listdir("/proc"):
        if not pid_str.isdigit():
            continue
        pid = int(pid_str)
        try:
            st = os.stat(f"/proc/{pid}")
            if st.st_uid != my_uid:
                continue
            cmd = proc_cmdline(pid).lower()
            if not cmd:
                continue
            if any(p in cmd for p in ROS2_PATTERNS):
                found.add(pid)
        except (FileNotFoundError, PermissionError):
            pass
    return found


def force_kill(pids):
    """Send ``SIGKILL`` to every PID in the given iterable.

    Silently ignores PIDs that no longer exist at the time of the call.

    Parameters
    ----------
    pids : iterable of int
        PIDs to kill.
    """
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def header(title):
    """Print a formatted section header to stdout.

    Parameters
    ----------
    title : str
        Text to display as the section title.
    """
    print()
    print("=" * 60)
    print(title)
    print("=" * 60)


def print_tree(label, pids):
    """Print a formatted list of processes with PID, PPID, name, and cmdline.

    Parameters
    ----------
    label : str
        Heading to display above the process list.
    pids : iterable of int
        PIDs to display. If empty, prints ``'(none)'``.
    """
    print(f"\n{label} ({len(pids)} processes):")
    if not pids:
        print("  (none)")
        return
    for pid in sorted(pids):
        name = proc_name(pid)
        cmd = proc_cmdline(pid)
        ppid = proc_ppid(pid)
        if len(cmd) > 50:
            cmd = cmd[:47] + "..."
        print(f"  PID={pid:6d}  PPID={ppid:6d}  {name:<18}  {cmd}")


# ---------------------------------------------------------------------------
# Process-group helper
# ---------------------------------------------------------------------------


def new_pgroup_only():
    """Move the calling process into its own process group within the runner's session.

    Intended to be used as a ``preexec_fn`` in ``subprocess.Popen``. Calls
    ``os.setpgid(0, 0)`` in the child process between ``fork`` and ``exec``,
    making the child the leader of a new process group whose ID equals its own
    PID. The session is not changed — the child remains in the runner's session.
    """
    os.setpgid(0, 0)


# ---------------------------------------------------------------------------
# Scenario 1
# ---------------------------------------------------------------------------
def scenario_simple():
    """Run scenario 1: single LAUNCHER process with no descendants.

    Baseline test that confirms the process group creation and ``killpg``
    primitive work correctly in the simplest possible case — a single process
    with no children.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``
        where both boolean fields are ``True`` if the launcher was killed.
    """
    header("SCENARIO 1: Simple subject")

    launcher_script = """
import os, time
print(f'[LAUNCHER] PID={os.getpid()} PGID={os.getpgid(0)} SID={os.getsid(0)}',
      flush=True)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=new_pgroup_only,
    )
    pgid = os.getpgid(launcher.pid)
    runner_sid = os.getsid(0)
    time.sleep(0.3)
    print(
        f"Launcher PID={launcher.pid}  PGID={pgid}"
        f"  (still in runner's SID={runner_sid})"
    )

    print(f"\nKilling process group {pgid} with SIGKILL ...")
    os.killpg(pgid, signal.SIGKILL)
    time.sleep(0.5)

    launcher_dead = launcher.poll() is not None
    print(f"\nLauncher dead: {launcher_dead}")

    if launcher_dead:
        print(
            "\n✅ EXPECTED: launcher killed"
            " — killpg reaches the only process in its group"
        )
    else:
        print("\n⚠️  Unexpected: launcher survived")

    launcher.wait()
    return ("simple", "Baseline, no descendants", True, launcher_dead)


# ---------------------------------------------------------------------------
# Scenario 2
# ---------------------------------------------------------------------------
def scenario_deep():
    """Run scenario 2: three-level cooperative process tree.

    Verifies that PGID inheritance propagates correctly through two levels of
    ``fork()`` without any explicit cooperation from the child processes.
    All three processes (LAUNCHER -> CHILD -> GRANDCHILD) should be killed by
    a single ``killpg`` call.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``observed_kill_all`` is ``True`` if both the launcher and the
        grandchild were killed.
    """
    header("SCENARIO 2: Deep cooperative tree")

    grandchild_script = (
        "import os,time; "
        "print(f'[GRANDCHILD] PID={os.getpid()} PGID={os.getpgid(0)}', flush=True); "
        "time.sleep(60)"
    )
    child_script = f"""
import os, subprocess, sys, time
print(f'[CHILD] PID={{os.getpid()}} PGID={{os.getpgid(0)}}', flush=True)
gc = subprocess.Popen([sys.executable, "-c", {repr(grandchild_script)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    launcher_script = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}} PGID={{os.getpgid(0)}}', flush=True)
child = subprocess.Popen(
    [sys.executable, "-c", {repr(child_script)}], stdout=sys.stdout
)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=new_pgroup_only,
        stdout=subprocess.PIPE,
        text=True,
    )
    pgid = os.getpgid(launcher.pid)

    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    print(f"Launcher   PID={launcher.pid}  PGID={pgid}")
    print(f"Grandchild PID={grandchild_pid} (inherited PGID={pgid} through fork)")

    print(f"\nKilling process group {pgid} with SIGKILL ...")
    os.killpg(pgid, signal.SIGKILL)
    time.sleep(0.5)

    launcher_dead = launcher.poll() is not None
    grandchild_alive = alive(grandchild_pid)
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {grandchild_alive}")

    verdict = launcher_dead and not grandchild_alive
    if verdict:
        print("\n✅ EXPECTED: whole tree killed — fork() inherits PGID")
    elif launcher_dead and grandchild_alive:
        print("\n⚠️  Unexpected: grandchild survived even though it shares the PGID")
        os.kill(grandchild_pid, signal.SIGKILL)
    else:
        print("\n⚠️  Unexpected result")

    launcher.wait()
    return ("deep", "3-level tree, no escape", True, verdict)


# ---------------------------------------------------------------------------
# Scenario 3
# ---------------------------------------------------------------------------
def scenario_setpgid():
    """Run scenario 3: GRANDCHILD escapes by calling ``os.setpgid(0, 0)``.

    Demonstrates that a descendant can leave the launcher's process group at
    any time by calling ``setpgid``. After the escape, ``killpg`` on the
    original PGID no longer reaches GRANDCHILD, so it survives while LAUNCHER
    and CHILD are killed.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``expected_kill_all`` is ``False`` because an escape is the expected
        outcome. ``observed_kill_all`` is ``False`` if the grandchild survived
        as expected.
    """
    header("SCENARIO 3: Process group escape (setpgid)")

    grandchild_script = (
        "import os,time; "
        "os.setpgid(0, 0); "
        "print(f'[GRANDCHILD] PID={os.getpid()}"
        " PGID={os.getpgid(0)} SID={os.getsid(0)}', flush=True); "
        "time.sleep(60)"
    )
    child_script = f"""
import os, subprocess, sys, time
print(f'[CHILD] PID={{os.getpid()}} PGID={{os.getpgid(0)}}', flush=True)
gc = subprocess.Popen([sys.executable, "-c", {repr(grandchild_script)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    launcher_script = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}} PGID={{os.getpgid(0)}}', flush=True)
child = subprocess.Popen(
    [sys.executable, "-c", {repr(child_script)}], stdout=sys.stdout
)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=new_pgroup_only,
        stdout=subprocess.PIPE,
        text=True,
    )
    pgid = os.getpgid(launcher.pid)

    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    print(f"Launcher   PID={launcher.pid}  PGID={pgid}")
    print(
        f"Grandchild PID={grandchild_pid}"
        " (called setpgid(0,0) — now leader of its own group)"
    )

    print(f"\nKilling process group {pgid} with SIGKILL ...")
    os.killpg(pgid, signal.SIGKILL)
    time.sleep(0.5)

    launcher_dead = launcher.poll() is not None
    grandchild_alive = alive(grandchild_pid)
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {grandchild_alive}")

    if grandchild_alive:
        os.kill(grandchild_pid, signal.SIGKILL)

    if launcher_dead and grandchild_alive:
        print(
            "\n❌ EXPECTED: grandchild survived — it left the launcher's process group"
        )
    elif launcher_dead and not grandchild_alive:
        print("\n⚠️  All dead — grandchild did not actually escape")
    else:
        print("\n⚠️  Unexpected result")

    launcher.wait()
    return ("setpgid", "GRANDCHILD calls setpgid(0,0)", False, not grandchild_alive)


# ---------------------------------------------------------------------------
# Scenario 4
# ---------------------------------------------------------------------------
def scenario_setsid():
    """Run scenario 4: GRANDCHILD escapes by calling ``os.setsid()``.

    Demonstrates that ``setsid`` is a strict superset of ``setpgid`` as an
    escape mechanism: it creates a new session and, as a side effect, places
    the caller into a new process group it leads itself. The outcome mirrors
    scenario 3 — GRANDCHILD survives ``killpg`` on the original PGID —
    confirming that creating a new session is sufficient but not necessary to
    defeat the cleanup model.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``expected_kill_all`` is ``False`` because an escape is the expected
        outcome. ``observed_kill_all`` is ``False`` if the grandchild survived
        as expected.
    """
    header("SCENARIO 4: Stronger escape (setsid)")

    grandchild_script = (
        "import os,time; "
        "os.setsid(); "
        "print(f'[GRANDCHILD] PID={os.getpid()}"
        " PGID={os.getpgid(0)} SID={os.getsid(0)}', flush=True); "
        "time.sleep(60)"
    )
    child_script = f"""
import os, subprocess, sys, time
print(f'[CHILD] PID={{os.getpid()}} PGID={{os.getpgid(0)}}', flush=True)
gc = subprocess.Popen([sys.executable, "-c", {repr(grandchild_script)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    launcher_script = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}} PGID={{os.getpgid(0)}}', flush=True)
child = subprocess.Popen(
    [sys.executable, "-c", {repr(child_script)}], stdout=sys.stdout
)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=new_pgroup_only,
        stdout=subprocess.PIPE,
        text=True,
    )
    pgid = os.getpgid(launcher.pid)

    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    print(f"Launcher   PID={launcher.pid}  PGID={pgid}")
    print(
        f"Grandchild PID={grandchild_pid}"
        " (called setsid() — now in its own session and group)"
    )

    print(f"\nKilling process group {pgid} with SIGKILL ...")
    os.killpg(pgid, signal.SIGKILL)
    time.sleep(0.5)

    launcher_dead = launcher.poll() is not None
    grandchild_alive = alive(grandchild_pid)
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {grandchild_alive}")

    if grandchild_alive:
        os.kill(grandchild_pid, signal.SIGKILL)

    if launcher_dead and grandchild_alive:
        print(
            "\n❌ EXPECTED: grandchild survived"
            " — setsid is a strict superset of setpgid"
        )
    elif launcher_dead and not grandchild_alive:
        print("\n⚠️  All dead — grandchild did not actually escape")
    else:
        print("\n⚠️  Unexpected result")

    launcher.wait()
    return ("setsid", "GRANDCHILD calls setsid()", False, not grandchild_alive)


# ---------------------------------------------------------------------------
# Scenario 5 — ROS2 full stack
# ---------------------------------------------------------------------------
def scenario_ros2_full_stack(env):
    """Run scenario 5: full ROS 2 workload with launch, bag play, and bag record.

    Launches the Beluga AMCL stack (``ros2 launch``), a bag player
    (``ros2 bag play``), and a bag recorder (``ros2 bag record``) as three
    separate top-level processes, each in its own process group. After a fixed
    run duration, ``killpg`` is sent to each PGID and the system is scanned
    for any surviving ROS 2 processes to detect descendants that may have
    escaped.

    Parameters
    ----------
    env : dict
        Environment variables to pass to all subprocesses, as returned by
        ``ros2_env()``.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``observed_kill_all`` is ``True`` if no ROS 2 processes survived after
        ``killpg`` was applied to all three process groups.
    """
    header("SCENARIO 5: ROS2 full stack (launch + bag play + bag record)")
    print(f"  Launcher: ros2 launch {LAUNCH_PACKAGE} {LAUNCH_FILE}")
    print(f"  Bag:      {BAG_PATH}")
    print("  Cleanup:  killpg(SIGKILL) on each PGID")
    print(f"  Duration: {RUN_DURATION}s")

    baseline = scan_ros2_processes()
    print(f"\nBaseline ROS2 processes already in system: {len(baseline)}")

    record_outdir = tempfile.mkdtemp(prefix="ros2_pg_")

    launch_proc = subprocess.Popen(
        ["ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE, f"map_path:={MAP_PATH}"],
        preexec_fn=new_pgroup_only,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    pgid_launch = os.getpgid(launch_proc.pid)
    time.sleep(8)

    play_proc = subprocess.Popen(
        ["ros2", "bag", "play", BAG_PATH, "--loop"],
        preexec_fn=new_pgroup_only,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    pgid_play = os.getpgid(play_proc.pid)
    time.sleep(2)

    record_proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", record_outdir] + RECORD_TOPICS,
        preexec_fn=new_pgroup_only,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    pgid_record = os.getpgid(record_proc.pid)

    print(f"\nLauncher PID={launch_proc.pid}  PGID={pgid_launch}")
    print(f"Player   PID={play_proc.pid}  PGID={pgid_play}")
    print(f"Recorder PID={record_proc.pid}  PGID={pgid_record}")
    print(f"\nWaiting {RUN_DURATION}s for the workload to fully spawn ...")
    time.sleep(RUN_DURATION)

    full_tree = set()
    for root in [launch_proc.pid, play_proc.pid, record_proc.pid]:
        for pid in get_descendants(root):
            full_tree.add(pid)
    print_tree("Descendant tree before kill", sorted(full_tree))

    print(f"\nSending SIGKILL to PGIDs: {pgid_launch}, {pgid_play}, {pgid_record} ...")
    for pgid in [pgid_launch, pgid_play, pgid_record]:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    time.sleep(2.0)

    tree_survivors = [p for p in full_tree if alive(p)]
    escapees = sorted(scan_ros2_processes() - baseline)
    print_tree("Tree survivors AFTER killpg", tree_survivors)
    print_tree("Global ROS2 escapees AFTER killpg", escapees)

    if escapees:
        n = len(escapees)
        print(
            f"\n⚠️  Force-killing {n} leaked process(es) so the bench can exit cleanly"
        )
        force_kill(escapees)
        time.sleep(0.5)

    for proc in [launch_proc, play_proc, record_proc]:
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                proc.kill()
                proc.wait(timeout=2)
            except Exception:
                pass

    shutil.rmtree(record_outdir, ignore_errors=True)

    ok = len(escapees) == 0 and len(tree_survivors) == 0
    if ok:
        print("\n✅ All ROS2 processes killed — no escapes detected")
    else:
        n_tree = len(tree_survivors)
        n_esc = len(escapees)
        print(f"\n❌ ESCAPE DETECTED ({n_tree} in tree, {n_esc} global)")
    return ("ros2_full_stack", "ROS2 launch + bag play + bag record", True, ok)


def main():
    """Run all POSIX process group test scenarios and print a summary table.

    Verifies that the ROS 2 environment is sourced, then runs the five
    scenarios in sequence. Prints a per-scenario summary table showing
    expected vs observed outcome for each.
    """
    print("=" * 60)
    print("POSIX PROCESS GROUPS — TEST BENCH")
    print("Mechanism: subprocess.Popen(..., preexec_fn=os.setpgid) + os.killpg")
    print("=" * 60)

    env = ros2_env()
    if "ROS_DISTRO" not in env:
        print("❌ ROS2 environment not sourced.")
        sys.exit(1)
    print(f"ROS_DISTRO={env['ROS_DISTRO']}  ✅")

    results = [
        scenario_simple(),
        scenario_deep(),
        scenario_setpgid(),
        scenario_setsid(),
        scenario_ros2_full_stack(env),
    ]

    header("SUMMARY")
    print(f"{'Scenario':<18} {'Description':<40} {'Expected':>10} {'Observed':>10}")
    print("-" * 82)
    for name, description, should_kill_all, did_kill_all in results:
        exp = "kill all" if should_kill_all else "  escape"
        obs = "kill all" if did_kill_all else "  escape"
        mark = "✓" if (should_kill_all == did_kill_all) else "✗"
        print(f"{name:<18} {description:<40} {exp:>10} {obs:>10}   {mark}")

    cooperative = [r for r in results if r[2]]
    escape = [r for r in results if not r[2]]
    n_coop = sum(1 for r in cooperative if r[3])
    n_esc = sum(1 for r in escape if r[3])
    print(f"\nCooperative scenarios cleaned up: {n_coop}/{len(cooperative)}")
    print(f"Escape scenarios cleaned up:      {n_esc}/{len(escape)}")


if __name__ == "__main__":
    main()
