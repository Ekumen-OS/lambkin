#!/usr/bin/env python3
"""
Test bench: cgroups v2

Scenarios:
    1. simple           — LAUNCHER only (baseline)
    2. deep             — 3-level cooperative tree
    3. setpgid          — GRANDCHILD calls setpgid(0,0)
    4. setsid           — GRANDCHILD calls setsid()
    5. ros2_full_stack  — ros2 launch + bag play + bag record (real workload)

HOW TO RUN:
    systemd-run --user --scope -- python3 bench_cgroups_v2.py
"""

import os, signal, subprocess, sys, time, uuid, shutil, tempfile


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
# cgroup-specific helpers
# ---------------------------------------------------------------------------

CGROUP_ROOT = "/sys/fs/cgroup"


def find_writable_parent():
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
    cg = os.path.join(parent, f"bench-{uuid.uuid4().hex[:8]}")
    os.makedirs(cg, exist_ok=True)
    return cg


def enter_cgroup(cg):
    with open(os.path.join(cg, "cgroup.procs"), "w") as f:
        f.write(str(os.getpid()))


def cgroup_pids(cg):
    try:
        with open(os.path.join(cg, "cgroup.procs")) as f:
            return [int(p) for p in f.read().split() if p.strip()]
    except FileNotFoundError:
        return []


def cgroup_kill(cg):
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
    header("SCENARIO 1: Simple subject")
    cg = make_cgroup(parent)

    LAUNCHER = """
import os, time
print(f'[LAUNCHER] PID={os.getpid()}', flush=True)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", LAUNCHER],
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
    header("SCENARIO 2: Deep cooperative tree")
    cg = make_cgroup(parent)

    GRANDCHILD = (
        "import os,time; "
        "print(f'[GRANDCHILD] PID={os.getpid()}', flush=True); "
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
print(f'[LAUNCHER] PID={{os.getpid()}}', flush=True)
child = subprocess.Popen([sys.executable, "-c", {repr(CHILD)}], stdout=sys.stdout)
time.sleep(60)
"""
    launcher = subprocess.Popen(
        [sys.executable, "-c", LAUNCHER],
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

    print(f"\nKilling cgroup ...")
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
    header("SCENARIO 3: setpgid has no effect on cgroup membership")
    cg = make_cgroup(parent)

    GRANDCHILD = (
        "import os,time; "
        "os.setpgid(0,0); "
        "print(f'[GRANDCHILD] PID={os.getpid()} PGID={os.getpgid(0)}', flush=True); "
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

    print(f"\nKilling cgroup ...")
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
    header("SCENARIO 4: setsid has no effect on cgroup membership")
    cg = make_cgroup(parent)

    GRANDCHILD = (
        "import os,time; "
        "os.setsid(); "
        "print(f'[GRANDCHILD] PID={os.getpid()} SID={os.getsid(0)}', flush=True); "
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
        f"Grandchild PID={grandchild_pid} (called setsid() — new session, but still in cgroup)"
    )
    print(f"cgroup members: {pids}")

    print(f"\nKilling cgroup ...")
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
    header("SCENARIO 5: ROS2 full stack (launch + bag play + bag record)")
    print(f"  Launcher: ros2 launch {LAUNCH_PACKAGE} {LAUNCH_FILE}")
    print(f"  Bag:      {BAG_PATH}")
    print(f"  Cleanup:  cgroup.kill")
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

    print(f"\nKilling cgroup with cgroup.kill ...")
    cgroup_kill(cg)
    time.sleep(2.0)

    tree_survivors = [p for p in full_tree if alive(p)]
    escapees = sorted(scan_ros2_processes() - baseline)
    print_tree("Tree survivors AFTER cgroup.kill", tree_survivors)
    print_tree("Global ROS2 escapees AFTER cgroup.kill", escapees)

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

    cgroup_remove(cg)
    shutil.rmtree(record_outdir, ignore_errors=True)

    ok = len(escapees) == 0 and len(tree_survivors) == 0
    if ok:
        print("\n✅ All ROS2 processes killed by cgroup.kill — full tree contained")
    else:
        print(
            f"\n❌ ESCAPE DETECTED ({len(tree_survivors)} in tree, {len(escapees)} global)"
        )
    return ("ros2_full_stack", "ROS2 launch + bag play + bag record", True, ok)


def main():
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
    print(
        f"\nCooperative scenarios cleaned up: {sum(1 for r in cooperative if r[3])}/{len(cooperative)}"
    )
    print(
        f"Escape scenarios cleaned up:      {sum(1 for r in escape if r[3])}/{len(escape)}"
    )


if __name__ == "__main__":
    main()
