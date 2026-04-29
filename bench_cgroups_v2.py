#!/usr/bin/env python3
"""Test bench: cgroups v2.

Scenarios:
    1. simple           — LAUNCHER only (baseline)
    2. deep             — 3-level cooperative tree
    3. setpgid          — GRANDCHILD calls setpgid(0,0)
    4. setsid           — GRANDCHILD calls setsid()
    5. ros2_full_stack  — ros2 launch + bag play + bag record (real workload)

HOW TO RUN:
    systemd-run --user --scope -- python3 bench_cgroups_v2.py
"""

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

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
# cgroup-specific helpers
# ---------------------------------------------------------------------------

CGROUP_ROOT = "/sys/fs/cgroup"


def find_writable_parent():
    """Find a writable cgroup directory in the current process's cgroup hierarchy.

    Reads ``/proc/self/cgroup`` to determine the current cgroup path, then
    walks up the hierarchy until a directory with write permission is found.
    This is the parent under which per-step child cgroups will be created.

    Returns:
    -------
    str
        Absolute path to the nearest writable ancestor cgroup directory.

    Raises:
    ------
    RuntimeError
        If no writable cgroup directory is found. This typically means the
        process was not launched inside a delegated scope. Re-run with
        ``systemd-run --user --scope``.
    """
    with open("/proc/self/cgroup") as f:
        for line in f:
            if line.startswith("0::"):
                rel = line.strip().split("::", 1)[1]
                candidate = CGROUP_ROOT + rel
                if os.access(candidate, os.W_OK):
                    return candidate
                parts = candidate.rstrip("/").split("/")
                while len(parts) > 3:
                    parts.pop()
                    ancestor = "/".join(parts)
                    if os.access(ancestor, os.W_OK):
                        return ancestor
    raise RuntimeError(
        "No writable cgroup found.\n"
        "Run with: systemd-run --user --scope -- python3 bench_cgroups_v2.py"
    )


def make_cgroup(parent):
    """Create a fresh child cgroup directory under the given parent.

    The directory name is a ``bench-`` prefix followed by a random 8-character
    hex string to avoid collisions between concurrent bench runs.

    Parameters
    ----------
    parent : str
        Absolute path to the writable parent cgroup directory.

    Returns:
    -------
    str
        Absolute path to the newly created child cgroup directory.
    """
    cg = os.path.join(parent, f"bench-{uuid.uuid4().hex[:8]}")
    os.makedirs(cg, exist_ok=True)
    return cg


def enter_cgroup(cg):
    """Move the calling process into the given cgroup.

    Writes the current PID to ``cgroup.procs`` inside ``cg``. Intended to be
    used as a ``preexec_fn`` in ``subprocess.Popen`` so that the child process
    joins the cgroup before any of its own children are created.

    Parameters
    ----------
    cg : str
        Absolute path to the target cgroup directory.
    """
    with open(os.path.join(cg, "cgroup.procs"), "w") as f:
        f.write(str(os.getpid()))


def cgroup_pids(cg):
    """Return the list of PIDs currently in a cgroup.

    Reads ``cgroup.procs`` from the given cgroup directory. Returns an empty
    list if the file does not exist (e.g. after the cgroup has been removed).

    Parameters
    ----------
    cg : str
        Absolute path to the cgroup directory.

    Returns:
    -------
    list of int
        PIDs of all processes currently in the cgroup.
    """
    try:
        with open(os.path.join(cg, "cgroup.procs")) as f:
            return [int(p) for p in f.read().split() if p.strip()]
    except FileNotFoundError:
        return []


def cgroup_kill(cg):
    """Kill all processes in a cgroup using the best available strategy.

    Prefers the atomic ``cgroup.kill`` interface (Linux >= 5.14). Falls back
    to an iterative ``SIGKILL`` loop on older kernels, retrying up to 20 times
    with a short sleep between attempts.

    Parameters
    ----------
    cg : str
        Absolute path to the cgroup directory to kill.

    Returns:
    -------
    str
        ``'cgroup.kill'`` if the atomic interface was used, or
        ``'iterative'`` if the fallback loop was used.
    """
    kill_file = os.path.join(cg, "cgroup.kill")
    if os.path.exists(kill_file):
        with open(kill_file, "w") as f:
            f.write("1")
        return "cgroup.kill"
    for _ in range(20):
        pids = cgroup_pids(cg)
        if not pids:
            return "iterative"
        force_kill(pids)
        time.sleep(0.05)
    return "iterative"


def cgroup_remove(cg):
    """Remove a cgroup directory, retrying until it is empty.

    The kernel rejects ``rmdir`` on a non-empty cgroup, so this function
    retries up to 20 times with a short sleep between attempts to give
    processes time to exit after being killed.

    Parameters
    ----------
    cg : str
        Absolute path to the cgroup directory to remove.
    """
    for _ in range(20):
        try:
            os.rmdir(cg)
            return
        except OSError:
            time.sleep(0.05)


# ---------------------------------------------------------------------------
# Scenario 1
# ---------------------------------------------------------------------------
def scenario_simple(parent):
    """Run scenario 1: single LAUNCHER process with no descendants.

    Baseline test that confirms the cgroup creation and ``cgroup.kill``
    primitive work correctly in the simplest possible case — a single process
    with no children.

    Parameters
    ----------
    parent : str
        Absolute path to the writable parent cgroup directory.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``
        where both boolean fields are ``True`` if the launcher was killed and
        the cgroup was left empty.
    """
    header("SCENARIO 1: Simple subject")
    cg = make_cgroup(parent)

    launcher_script = """
import os, time
print(f'[LAUNCHER] PID={os.getpid()}', flush=True)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=lambda: enter_cgroup(cg),
    )
    time.sleep(0.3)

    pids = cgroup_pids(cg)
    print(f"Launcher PID={launcher.pid}  cgroup={os.path.basename(cg)}  members={pids}")

    print(f"\nKilling cgroup {os.path.basename(cg)} ...")
    strategy = cgroup_kill(cg)
    print(f"  (strategy: {strategy})")
    time.sleep(0.3)

    launcher_dead = launcher.poll() is not None
    cg_empty = cgroup_pids(cg) == []
    print(f"\nLauncher dead: {launcher_dead}")
    print(f"cgroup empty:  {cg_empty}")
    cgroup_remove(cg)

    ok = launcher_dead and cg_empty
    if ok:
        print("\n✅ EXPECTED: launcher killed, cgroup empty and removed")
    else:
        print("\n⚠️  Unexpected result")
    launcher.wait()
    return ("simple", "Baseline, no descendants", True, ok)


# ---------------------------------------------------------------------------
# Scenario 2
# ---------------------------------------------------------------------------
def scenario_deep(parent):
    """Run scenario 2: three-level cooperative process tree.

    Verifies that cgroup membership propagates correctly through two levels of
    ``fork()`` without any explicit cooperation from the child processes.
    All three processes (LAUNCHER -> CHILD -> GRANDCHILD) should be killed by
    a single ``cgroup.kill``.

    Parameters
    ----------
    parent : str
        Absolute path to the writable parent cgroup directory.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``observed_kill_all`` is ``True`` if all three processes were killed
        and the cgroup was left empty.
    """
    header("SCENARIO 2: Deep cooperative tree")
    cg = make_cgroup(parent)

    grandchild_script = (
        "import os,time; "
        "print(f'[GRANDCHILD] PID={os.getpid()}', flush=True); "
        "time.sleep(60)"
    )
    child_script = f"""
import os, subprocess, sys, time
print(f'[CHILD] PID={{os.getpid()}}', flush=True)
gc = subprocess.Popen([sys.executable, "-c", {repr(grandchild_script)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    launcher_script = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}}', flush=True)
child = subprocess.Popen(
    [sys.executable, "-c", {repr(child_script)}], stdout=sys.stdout
)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=lambda: enter_cgroup(cg),
        stdout=subprocess.PIPE,
        text=True,
    )
    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    pids = cgroup_pids(cg)
    print(f"Launcher   PID={launcher.pid}")
    print(f"Grandchild PID={grandchild_pid}")
    print(f"cgroup members: {pids}  (all 3 inherited membership through fork)")

    print("\nKilling cgroup ...")
    strategy = cgroup_kill(cg)
    print(f"  (strategy: {strategy})")
    time.sleep(0.3)

    launcher_dead = launcher.poll() is not None
    gc_alive = alive(grandchild_pid) if grandchild_pid else False
    cg_empty = cgroup_pids(cg) == []
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {gc_alive}")
    print(f"cgroup empty:     {cg_empty}")
    cgroup_remove(cg)

    ok = launcher_dead and not gc_alive and cg_empty
    if ok:
        print("\n✅ EXPECTED: whole tree killed — fork() propagates cgroup membership")
    else:
        print("\n⚠️  Unexpected result")
    if gc_alive and grandchild_pid:
        os.kill(grandchild_pid, signal.SIGKILL)
    launcher.wait()
    return ("deep", "3-level tree, no escape", True, ok)


# ---------------------------------------------------------------------------
# Scenario 3
# ---------------------------------------------------------------------------
def scenario_setpgid(parent):
    """Run scenario 3: GRANDCHILD calls ``os.setpgid(0, 0)``.

    Confirms that changing the process group ID has no effect on cgroup
    membership. Even though GRANDCHILD moves itself to a new process group,
    it remains in the same cgroup and is killed along with the rest of the
    tree.

    Parameters
    ----------
    parent : str
        Absolute path to the writable parent cgroup directory.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``observed_kill_all`` is ``True`` if all processes were killed despite
        the ``setpgid`` call.
    """
    header("SCENARIO 3: setpgid has no effect on cgroup membership")
    cg = make_cgroup(parent)

    grandchild_script = (
        "import os,time; "
        "os.setpgid(0,0); "
        "print(f'[GRANDCHILD] PID={os.getpid()} PGID={os.getpgid(0)}', flush=True); "
        "time.sleep(60)"
    )
    child_script = f"""
import os, subprocess, sys, time
gc = subprocess.Popen([sys.executable, "-c", {repr(grandchild_script)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    launcher_script = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}}', flush=True)
child = subprocess.Popen(
    [sys.executable, "-c", {repr(child_script)}], stdout=sys.stdout
)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=lambda: enter_cgroup(cg),
        stdout=subprocess.PIPE,
        text=True,
    )
    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    pids = cgroup_pids(cg)
    print(f"Launcher   PID={launcher.pid}")
    print(
        f"Grandchild PID={grandchild_pid} (called setpgid(0,0) — but still in cgroup)"
    )
    print(f"cgroup members: {pids}")

    print("\nKilling cgroup ...")
    cgroup_kill(cg)
    time.sleep(0.3)

    launcher_dead = launcher.poll() is not None
    gc_alive = alive(grandchild_pid) if grandchild_pid else False
    cg_empty = cgroup_pids(cg) == []
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {gc_alive}")
    print(f"cgroup empty:     {cg_empty}")
    cgroup_remove(cg)

    ok = launcher_dead and not gc_alive and cg_empty
    if ok:
        print("\n✅ EXPECTED: setpgid changes PGID but not cgroup membership")
    else:
        print("\n⚠️  Unexpected: cgroup membership leaked")
    if gc_alive and grandchild_pid:
        os.kill(grandchild_pid, signal.SIGKILL)
    launcher.wait()
    return ("setpgid", "GRANDCHILD calls setpgid(0,0)", True, ok)


# ---------------------------------------------------------------------------
# Scenario 4
# ---------------------------------------------------------------------------
def scenario_setsid(parent):
    """Run scenario 4: GRANDCHILD calls ``os.setsid()``.

    Confirms that creating a new session has no effect on cgroup membership.
    Even though GRANDCHILD moves itself to a new session and process group,
    it remains in the same cgroup and is killed along with the rest of the
    tree.

    Parameters
    ----------
    parent : str
        Absolute path to the writable parent cgroup directory.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``observed_kill_all`` is ``True`` if all processes were killed despite
        the ``setsid`` call.
    """
    header("SCENARIO 4: setsid has no effect on cgroup membership")
    cg = make_cgroup(parent)

    grandchild_script = (
        "import os,time; "
        "os.setsid(); "
        "print(f'[GRANDCHILD] PID={os.getpid()} SID={os.getsid(0)}', flush=True); "
        "time.sleep(60)"
    )
    child_script = f"""
import os, subprocess, sys, time
gc = subprocess.Popen([sys.executable, "-c", {repr(grandchild_script)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    launcher_script = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}}', flush=True)
child = subprocess.Popen(
    [sys.executable, "-c", {repr(child_script)}], stdout=sys.stdout
)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", launcher_script],
        preexec_fn=lambda: enter_cgroup(cg),
        stdout=subprocess.PIPE,
        text=True,
    )
    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    pids = cgroup_pids(cg)
    print(f"Launcher   PID={launcher.pid}")
    print(
        f"Grandchild PID={grandchild_pid}"
        " (called setsid() — new session, but still in cgroup)"
    )
    print(f"cgroup members: {pids}")

    print("\nKilling cgroup ...")
    cgroup_kill(cg)
    time.sleep(0.3)

    launcher_dead = launcher.poll() is not None
    gc_alive = alive(grandchild_pid) if grandchild_pid else False
    cg_empty = cgroup_pids(cg) == []
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {gc_alive}")
    print(f"cgroup empty:     {cg_empty}")
    cgroup_remove(cg)

    ok = launcher_dead and not gc_alive and cg_empty
    if ok:
        print("\n✅ EXPECTED: setsid creates a new session but not a new cgroup")
    else:
        print("\n⚠️  Unexpected: cgroup membership leaked")
    if gc_alive and grandchild_pid:
        os.kill(grandchild_pid, signal.SIGKILL)
    launcher.wait()
    return ("setsid", "GRANDCHILD calls setsid()", True, ok)


# ---------------------------------------------------------------------------
# Scenario 5 — ROS2 full stack
# ---------------------------------------------------------------------------
def scenario_ros2_full_stack(parent, env):
    """Run scenario 5: full ROS 2 workload with launch, bag play, and bag record.

    Launches the Beluga AMCL stack (``ros2 launch``), a bag player
    (``ros2 bag play``), and a bag recorder (``ros2 bag record``) as three
    separate top-level processes, all placed into the same cgroup. After a
    fixed run duration the cgroup is killed and the system is scanned for any
    surviving ROS 2 processes to detect descendants that may have escaped.

    Parameters
    ----------
    parent : str
        Absolute path to the writable parent cgroup directory.
    env : dict
        Environment variables to pass to all subprocesses, as returned by
        ``ros2_env()``.

    Returns:
    -------
    tuple
        A four-element tuple
        ``(name, description, expected_kill_all, observed_kill_all)``.
        ``observed_kill_all`` is ``True`` if no ROS 2 processes survived after
        ``cgroup.kill`` was applied.
    """
    header("SCENARIO 5: ROS2 full stack (launch + bag play + bag record)")
    print(f"  Launcher: ros2 launch {LAUNCH_PACKAGE} {LAUNCH_FILE}")
    print(f"  Bag:      {BAG_PATH}")
    print("  Cleanup:  cgroup.kill")
    print(f"  Duration: {RUN_DURATION}s")

    baseline = scan_ros2_processes()
    print(f"\nBaseline ROS2 processes already in system: {len(baseline)}")

    cg = make_cgroup(parent)
    record_outdir = tempfile.mkdtemp(prefix="ros2_cg_")
    print(f"Using cgroup: {os.path.basename(cg)}")

    launch_proc = subprocess.Popen(
        ["ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE, f"map_path:={MAP_PATH}"],
        preexec_fn=lambda: enter_cgroup(cg),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(8)

    play_proc = subprocess.Popen(
        ["ros2", "bag", "play", BAG_PATH, "--loop"],
        preexec_fn=lambda: enter_cgroup(cg),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(2)

    record_proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", record_outdir] + RECORD_TOPICS,
        preexec_fn=lambda: enter_cgroup(cg),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    print(f"\nLauncher PID={launch_proc.pid}")
    print(f"Player   PID={play_proc.pid}")
    print(f"Recorder PID={record_proc.pid}")
    print(f"\nWaiting {RUN_DURATION}s for the workload to fully spawn ...")
    time.sleep(RUN_DURATION)

    full_tree = set()
    for root in [launch_proc.pid, play_proc.pid, record_proc.pid]:
        for pid in get_descendants(root):
            full_tree.add(pid)
    for pid in cgroup_pids(cg):
        full_tree.add(pid)
    print_tree("Descendant tree before kill", sorted(full_tree))
    print(f"\ncgroup.procs members: {len(cgroup_pids(cg))}")

    print("\nKilling cgroup with cgroup.kill ...")
    cgroup_kill(cg)
    time.sleep(2.0)

    tree_survivors = [p for p in full_tree if alive(p)]
    escapees = sorted(scan_ros2_processes() - baseline)
    print_tree("Tree survivors AFTER cgroup.kill", tree_survivors)
    print_tree("Global ROS2 escapees AFTER cgroup.kill", escapees)

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

    cgroup_remove(cg)
    shutil.rmtree(record_outdir, ignore_errors=True)

    ok = len(escapees) == 0 and len(tree_survivors) == 0
    if ok:
        print("\n✅ All ROS2 processes killed by cgroup.kill — full tree contained")
    else:
        n_tree = len(tree_survivors)
        n_esc = len(escapees)
        print(f"\n❌ ESCAPE DETECTED ({n_tree} in tree, {n_esc} global)")
    return ("ros2_full_stack", "ROS2 launch + bag play + bag record", True, ok)


def main():
    """Run all cgroups v2 test scenarios and print a summary table.

    Verifies that the cgroups v2 mechanism is available and that the ROS 2
    environment is sourced, then runs the five scenarios in sequence. Prints
    a per-scenario summary table showing expected vs observed outcome for each.
    """
    print("=" * 60)
    print("CGROUPS v2 — TEST BENCH")
    print("Mechanism: dedicated cgroup per step + cgroup.kill")
    print("=" * 60)

    if not os.path.exists(os.path.join(CGROUP_ROOT, "cgroup.controllers")):
        print("\nERROR: cgroups v2 not mounted at /sys/fs/cgroup.")
        print("Run with: systemd-run --user --scope -- python3 bench_cgroups_v2.py")
        sys.exit(2)

    env = ros2_env()
    if "ROS_DISTRO" not in env:
        print("❌ ROS2 environment not sourced.")
        sys.exit(1)
    print(f"ROS_DISTRO={env['ROS_DISTRO']}  ✅")

    parent = find_writable_parent()
    print(f"\nUsing parent cgroup: {parent}")

    results = [
        scenario_simple(parent),
        scenario_deep(parent),
        scenario_setpgid(parent),
        scenario_setsid(parent),
        scenario_ros2_full_stack(parent, env),
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
