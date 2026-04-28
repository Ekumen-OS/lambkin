#!/usr/bin/env python3
"""
Test bench: Subprocess sessions (start_new_session=True + SIGHUP)

Scenarios:
    1. simple           — LAUNCHER only (baseline)
    2. deep             — Cooperative tree, SIGHUP propagation test
    3. ignhup           — GRANDCHILD installs SIG_IGN for SIGHUP (nohup)
    4. setsid           — GRANDCHILD calls setsid() — escapes session
    5. ros2_full_stack  — ros2 launch + bag play + bag record (real workload)

HOW TO RUN:
    python3 bench_subprocess_sessions.py
"""

import os, signal, subprocess, sys, time, shutil, tempfile


ROS2_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = "/home/teresa/ekumen/lambkin/ws/install/setup.bash"
LAUNCH_PACKAGE = "beluga_ros2"
LAUNCH_FILE = "beluga.launch.py"
MAP_PATH = "/home/teresa/ekumen/lambkin/examples/map/"
BAG_PATH = "/home/teresa/ekumen/lambkin/record_1/"
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
    try:
        return open(f"/proc/{pid}/comm").read().strip()
    except FileNotFoundError:
        return "<gone>"


def proc_cmdline(pid):
    try:
        return open(f"/proc/{pid}/cmdline").read().replace("\x00", " ").strip()
    except FileNotFoundError:
        return "<gone>"


def proc_ppid(pid):
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("PPid:"):
                    return int(line.split()[1])
    except FileNotFoundError:
        pass
    return 0


def alive(pid):
    """True if pid exists and is not a zombie."""
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("State:"):
                    return "Z" not in line
        return False
    except (FileNotFoundError, PermissionError):
        return False


def get_descendants(root_pid):
    """Walk /proc to find all descendants of root_pid (BFS)."""
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
    """Scan /proc for ROS2-related processes belonging to the current user."""
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
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def header(title):
    print()
    print("=" * 60)
    print(title)
    print("=" * 60)


def print_tree(label, pids):
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
# Scenario 1
# ---------------------------------------------------------------------------
def scenario_simple():
    header("SCENARIO 1: Simple subject")

    LAUNCHER = """
import os, time
print(f'[LAUNCHER] PID={os.getpid()} SID={os.getsid(0)}', flush=True)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", LAUNCHER],
        start_new_session=True,
    )
    sid = os.getsid(launcher.pid)
    time.sleep(0.3)
    print(f"Launcher PID={launcher.pid}  SID={sid}")

    print(f"\nSending SIGTERM to session leader ...")
    try:
        os.kill(launcher.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    time.sleep(0.5)

    launcher_dead = launcher.poll() is not None
    print(f"\nLauncher dead: {launcher_dead}")

    if launcher_dead:
        print("\n✅ EXPECTED: launcher killed")
    else:
        print("\n⚠️  Unexpected: launcher survived")

    launcher.wait()
    return ("simple", "Baseline, no descendants", True, launcher_dead)


# ---------------------------------------------------------------------------
# Scenario 2
# ---------------------------------------------------------------------------
def scenario_deep():
    header("SCENARIO 2: Deep cooperative tree (SIGHUP propagation)")

    GRANDCHILD = (
        "import os,time; "
        "print(f'[GRANDCHILD] PID={os.getpid()} SID={os.getsid(0)}', flush=True); "
        "time.sleep(60)"
    )
    CHILD = f"""
import os, subprocess, sys, time
print(f'[CHILD] PID={{os.getpid()}}', flush=True)
gc = subprocess.Popen([sys.executable, "-c", {repr(GRANDCHILD)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    LAUNCHER = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}} SID={{os.getsid(0)}}', flush=True)
child = subprocess.Popen([sys.executable, "-c", {repr(CHILD)}], stdout=sys.stdout)
time.sleep(60)
"""

    launcher = subprocess.Popen(
        [sys.executable, "-c", LAUNCHER],
        start_new_session=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    sid = os.getsid(launcher.pid)

    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    print(f"Launcher   PID={launcher.pid}  SID={sid}")
    print(f"Grandchild PID={grandchild_pid} (cooperative, same SID)")

    print(f"\nSending SIGHUP to session leader (PID={launcher.pid}) ...")
    try:
        os.kill(launcher.pid, signal.SIGHUP)
    except ProcessLookupError:
        pass
    time.sleep(1.0)

    launcher_dead = launcher.poll() is not None
    grandchild_alive = alive(grandchild_pid)
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {grandchild_alive}")

    if grandchild_alive:
        os.kill(grandchild_pid, signal.SIGKILL)

    if launcher_dead and grandchild_alive:
        print(
            "\n❌ EXPECTED: grandchild survived — SIGHUP does not propagate "
            "without controlling terminal"
        )
    elif launcher_dead and not grandchild_alive:
        print("\n⚠️  All dead — SIGHUP propagated unexpectedly")
    else:
        print("\n⚠️  Unexpected result")

    launcher.wait()
    return ("deep", "Cooperative tree, SIGHUP to leader", False, not grandchild_alive)


# ---------------------------------------------------------------------------
# Scenario 3
# ---------------------------------------------------------------------------
def scenario_ignhup():
    header("SCENARIO 3: SIGHUP ignored (nohup pattern)")

    GRANDCHILD = (
        "import os,signal,time; "
        "signal.signal(signal.SIGHUP, signal.SIG_IGN); "
        "print(f'[GRANDCHILD] PID={os.getpid()} (ignoring SIGHUP)', flush=True); "
        "time.sleep(60)"
    )
    CHILD = f"""
import os, signal, subprocess, sys, time
signal.signal(signal.SIGHUP, signal.SIG_IGN)
gc = subprocess.Popen([sys.executable, "-c", {repr(GRANDCHILD)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    LAUNCHER = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}}', flush=True)
child = subprocess.Popen([sys.executable, "-c", {repr(CHILD)}], stdout=sys.stdout)
time.sleep(60)
"""

    launcher = subprocess.Popen(
        [sys.executable, "-c", LAUNCHER],
        start_new_session=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    sid = os.getsid(launcher.pid)

    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    print(f"Launcher   PID={launcher.pid}  SID={sid}")
    print(f"Grandchild PID={grandchild_pid} (SIG_IGN for SIGHUP — nohup pattern)")

    print(f"\nSending SIGHUP to session leader ...")
    try:
        os.kill(launcher.pid, signal.SIGHUP)
    except ProcessLookupError:
        pass
    time.sleep(1.0)

    launcher_dead = launcher.poll() is not None
    grandchild_alive = alive(grandchild_pid)
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {grandchild_alive}")

    if grandchild_alive:
        os.kill(grandchild_pid, signal.SIGKILL)

    if launcher_dead and grandchild_alive:
        print("\n❌ EXPECTED: grandchild survived — SIGHUP was ignored")
    elif launcher_dead and not grandchild_alive:
        print("\n⚠️  Grandchild died — SIG_IGN did not take effect?")
    else:
        print("\n⚠️  Unexpected result")

    launcher.wait()
    return (
        "ignhup",
        "GRANDCHILD installs SIG_IGN for SIGHUP",
        False,
        not grandchild_alive,
    )


# ---------------------------------------------------------------------------
# Scenario 4
# ---------------------------------------------------------------------------
def scenario_setsid():
    header("SCENARIO 4: Session escape (setsid)")

    GRANDCHILD = (
        "import os,time; "
        "os.setsid(); "
        "print(f'[GRANDCHILD] PID={os.getpid()} new SID={os.getsid(0)}', flush=True); "
        "time.sleep(60)"
    )
    CHILD = f"""
import os, subprocess, sys, time
gc = subprocess.Popen([sys.executable, "-c", {repr(GRANDCHILD)}])
print(f'GRANDCHILD_PID:{{gc.pid}}', flush=True)
time.sleep(60)
"""
    LAUNCHER = f"""
import os, subprocess, sys, time
print(f'[LAUNCHER] PID={{os.getpid()}}', flush=True)
child = subprocess.Popen([sys.executable, "-c", {repr(CHILD)}], stdout=sys.stdout)
time.sleep(60)
"""

    launcher = subprocess.Popen(
        [sys.executable, "-c", LAUNCHER],
        start_new_session=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    sid = os.getsid(launcher.pid)

    grandchild_pid = None
    for line in launcher.stdout:
        if line.strip().startswith("GRANDCHILD_PID:"):
            grandchild_pid = int(line.strip().split(":")[1])
            break

    time.sleep(0.3)
    print(f"Launcher   PID={launcher.pid}  SID={sid}")
    print(f"Grandchild PID={grandchild_pid} (called setsid() — left the session)")

    print(f"\nSending SIGHUP to session leader ...")
    try:
        os.kill(launcher.pid, signal.SIGHUP)
    except ProcessLookupError:
        pass
    time.sleep(1.0)

    launcher_dead = launcher.poll() is not None
    grandchild_alive = alive(grandchild_pid)
    print(f"\nLauncher   dead:  {launcher_dead}")
    print(f"Grandchild alive: {grandchild_alive}")

    if grandchild_alive:
        os.kill(grandchild_pid, signal.SIGKILL)

    if launcher_dead and grandchild_alive:
        print("\n❌ EXPECTED: grandchild survived — escaped to its own session")
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
    header("SCENARIO 5: ROS2 full stack (launch + bag play + bag record)")
    print(f"  Launcher: ros2 launch {LAUNCH_PACKAGE} {LAUNCH_FILE}")
    print(f"  Bag:      {BAG_PATH}")
    print(f"  Cleanup:  SIGHUP to each session leader")
    print(f"  Duration: {RUN_DURATION}s")

    baseline = scan_ros2_processes()
    print(f"\nBaseline ROS2 processes already in system: {len(baseline)}")

    record_outdir = tempfile.mkdtemp(prefix="ros2_sess_")

    launch_proc = subprocess.Popen(
        ["ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE, f"map_path:={MAP_PATH}"],
        start_new_session=True,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    sid_launch = os.getsid(launch_proc.pid)
    time.sleep(8)

    play_proc = subprocess.Popen(
        ["ros2", "bag", "play", BAG_PATH, "--loop"],
        start_new_session=True,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    sid_play = os.getsid(play_proc.pid)
    time.sleep(2)

    record_proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", record_outdir] + RECORD_TOPICS,
        start_new_session=True,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    sid_record = os.getsid(record_proc.pid)

    print(f"\nLauncher PID={launch_proc.pid}  SID={sid_launch}")
    print(f"Player   PID={play_proc.pid}  SID={sid_play}")
    print(f"Recorder PID={record_proc.pid}  SID={sid_record}")
    print(f"\nWaiting {RUN_DURATION}s for the workload to fully spawn ...")
    time.sleep(RUN_DURATION)

    full_tree = set()
    for root in [launch_proc.pid, play_proc.pid, record_proc.pid]:
        for pid in get_descendants(root):
            full_tree.add(pid)
    print_tree("Descendant tree before kill", sorted(full_tree))

    print(
        f"\nSending SIGHUP to session leaders: "
        f"{launch_proc.pid}, {play_proc.pid}, {record_proc.pid} ..."
    )
    for proc in [launch_proc, play_proc, record_proc]:
        try:
            os.kill(proc.pid, signal.SIGHUP)
        except ProcessLookupError:
            pass
    time.sleep(2.0)

    tree_survivors = [p for p in full_tree if alive(p)]
    escapees = sorted(scan_ros2_processes() - baseline)
    print_tree("Tree survivors AFTER SIGHUP", tree_survivors)
    print_tree("Global ROS2 escapees AFTER SIGHUP", escapees)

    if escapees:
        print(
            f"\n⚠️  Force-killing {len(escapees)} leaked process(es) so the bench can exit cleanly"
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
        print(
            f"\n❌ ESCAPE DETECTED ({len(tree_survivors)} in tree, {len(escapees)} global)"
        )
    return ("ros2_full_stack", "ROS2 launch + bag play + bag record", True, ok)


def main():
    print("=" * 60)
    print("SUBPROCESS SESSIONS — TEST BENCH")
    print("Mechanism: subprocess.Popen(..., start_new_session=True) + SIGHUP")
    print("=" * 60)

    env = ros2_env()
    if "ROS_DISTRO" not in env:
        print("❌ ROS2 environment not sourced.")
        sys.exit(1)
    print(f"ROS_DISTRO={env['ROS_DISTRO']}  ✅")

    results = [
        scenario_simple(),
        scenario_deep(),
        scenario_ignhup(),
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
    print(
        f"\nCooperative scenarios cleaned up: {sum(1 for r in cooperative if r[3])}/{len(cooperative)}"
    )
    print(
        f"Escape scenarios cleaned up:      {sum(1 for r in escape if r[3])}/{len(escape)}"
    )


if __name__ == "__main__":
    main()
