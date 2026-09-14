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

"""Unit tests for per-process resource measurement."""

import os
import shutil
import threading
import time
from pathlib import Path

import pytest
import yaml

from lambkin.core.process.resources import (
    ResourceSampler,
    pids_in_cgroup,
    read_memory,
    read_name,
    read_stat,
)

# Clock ticks per second, to express expected CPU times in the tests.
TICKS = os.sysconf("SC_CLK_TCK")


def write_proc_entry(
    proc_root,
    pid,
    name,
    utime=0,
    stime=0,
    threads=1,
    starttime=1000,
    rss_kb=2048,
    hwm_kb=4096,
    comm=None,
):
    """Write a synthetic /proc/<pid> entry and return its directory.

    Args:
        proc_root: Root of the fake proc filesystem.
        pid: Process ID to create an entry for.
        name: argv[0] written to cmdline.
        utime: User CPU time in clock ticks.
        stime: Kernel CPU time in clock ticks.
        threads: Thread count reported in stat.
        starttime: Process start time in clock ticks since boot.
        rss_kb: VmRSS value in kB, or None to omit the field.
        hwm_kb: VmHWM value in kB, or None to omit the field.
        comm: Value of stat's comm field. Defaults to the basename of name.

    Returns:
        The path to the created entry.
    """
    entry = proc_root / str(pid)
    entry.mkdir(parents=True, exist_ok=True)
    (entry / "cmdline").write_bytes(name.encode() + b"\0-arg\0")

    # stat fields 3..22, so that field N lands at index N - 3.
    fields = ["S"] + ["0"] * 19
    fields[11] = str(utime)  # field 14
    fields[12] = str(stime)  # field 15
    fields[17] = str(threads)  # field 20
    fields[19] = str(starttime)  # field 22
    label = comm if comm is not None else os.path.basename(name)
    (entry / "stat").write_text(f"{pid} ({label}) " + " ".join(fields) + "\n")

    status = ["Name:\t" + os.path.basename(name)[:15]]
    if rss_kb is not None:
        status.append(f"VmRSS:\t{rss_kb} kB")
    if hwm_kb is not None:
        status.append(f"VmHWM:\t{hwm_kb} kB")
    (entry / "status").write_text("\n".join(status) + "\n")
    return entry


@pytest.fixture
def proc_root(tmp_path):
    """Return an empty fake /proc root."""
    root = tmp_path / "proc"
    root.mkdir()
    return root


@pytest.fixture
def fake_cgroup(tmp_path):
    """Return a fake cgroup directory with an empty cgroup.procs file."""
    cgroup = tmp_path / "fake-proc-cgroup"
    cgroup.mkdir()
    (cgroup / "cgroup.procs").write_text("")
    return cgroup


@pytest.fixture
def output_dir(tmp_path):
    """Return a directory for sampler artifacts."""
    out = tmp_path / "iter_1"
    out.mkdir()
    return out


def set_pids(cgroup, *pids):
    """Write the given PIDs into a fake cgroup's cgroup.procs file."""
    (cgroup / "cgroup.procs").write_text("\n".join(str(p) for p in pids) + "\n")


# ── /proc parsing ─────────────────────────────────────────────────────────────


def test_read_stat_extracts_cpu_times(proc_root):
    """read_stat converts utime and stime from clock ticks to seconds."""
    write_proc_entry(proc_root, 42, "node", utime=2 * TICKS, stime=TICKS)
    stat = read_stat(42, proc_root)
    assert stat.utime_s == pytest.approx(2.0)
    assert stat.stime_s == pytest.approx(1.0)


def test_read_stat_extracts_num_threads(proc_root):
    """read_stat reports the thread count."""
    write_proc_entry(proc_root, 42, "node", threads=7)
    assert read_stat(42, proc_root).num_threads == 7


def test_read_stat_extracts_starttime(proc_root):
    """read_stat reports the process start time."""
    write_proc_entry(proc_root, 42, "node", starttime=987654)
    assert read_stat(42, proc_root).starttime == 987654


def test_read_stat_handles_comm_with_spaces_and_parentheses(proc_root):
    """read_stat parses stat even when comm contains spaces and parentheses."""
    write_proc_entry(proc_root, 42, "node", utime=TICKS, comm="my (weird) name")
    stat = read_stat(42, proc_root)
    assert stat is not None
    assert stat.utime_s == pytest.approx(1.0)


def test_read_stat_returns_none_for_missing_pid(proc_root):
    """read_stat returns None when the process is gone."""
    assert read_stat(4242, proc_root) is None


def test_read_stat_returns_none_for_malformed_file(proc_root):
    """read_stat returns None when stat cannot be parsed."""
    entry = proc_root / "42"
    entry.mkdir()
    (entry / "stat").write_text("nonsense without the expected shape\n")
    assert read_stat(42, proc_root) is None


def test_read_memory_extracts_rss_and_peak(proc_root):
    """read_memory converts VmRSS and VmHWM from kB to MiB."""
    write_proc_entry(proc_root, 42, "node", rss_kb=1024, hwm_kb=4096)
    rss, peak = read_memory(42, proc_root)
    assert rss == pytest.approx(1.0)
    assert peak == pytest.approx(4.0)


def test_read_memory_returns_none_when_fields_absent(proc_root):
    """read_memory returns None for a process with no memory fields."""
    write_proc_entry(proc_root, 42, "kthread", rss_kb=None, hwm_kb=None)
    assert read_memory(42, proc_root) is None


def test_read_memory_returns_none_for_missing_pid(proc_root):
    """read_memory returns None when the process is gone."""
    assert read_memory(4242, proc_root) is None


def test_read_name_returns_basename_of_argv0(proc_root):
    """read_name returns the basename of argv[0], not the full path."""
    write_proc_entry(proc_root, 42, "/opt/ros/jazzy/lib/pkg/my_node")
    assert read_name(42, proc_root) == "my_node"


def test_read_name_matches_seventeen_character_name(proc_root):
    """read_name matches names longer than comm's 15-character limit."""
    write_proc_entry(proc_root, 42, "/opt/ros/lib/cartographer_node")
    assert read_name(42, proc_root) == "cartographer_node"
    assert len("cartographer_node") > 15
    assert not (proc_root / "42" / "comm").exists()


def test_read_name_returns_none_for_empty_cmdline(proc_root):
    """read_name returns None for a process with an empty command line."""
    entry = proc_root / "42"
    entry.mkdir()
    (entry / "cmdline").write_bytes(b"")
    assert read_name(42, proc_root) is None


# ── PID discovery ─────────────────────────────────────────────────────────────


def test_pids_in_cgroup_reads_cgroup_procs(fake_cgroup):
    """pids_in_cgroup returns the PIDs listed in cgroup.procs, ascending."""
    set_pids(fake_cgroup, 30, 10, 20)
    assert pids_in_cgroup(fake_cgroup) == [10, 20, 30]


def test_pids_in_cgroup_returns_empty_when_missing(tmp_path):
    """pids_in_cgroup returns an empty list when the cgroup is gone."""
    assert pids_in_cgroup(tmp_path / "nope") == []


def test_find_pid_returns_none_when_cgroup_is_empty(fake_cgroup, proc_root, output_dir):
    """A target stays unresolved while its cgroup holds no processes."""
    sampler = ResourceSampler(["node"], fake_cgroup, output_dir, proc_root=proc_root)
    assert sampler._find_pid(sampler._targets["node"]) is None


def test_find_pid_returns_none_when_no_name_matches(fake_cgroup, proc_root, output_dir):
    """A target stays unresolved when no process has its name."""
    write_proc_entry(proc_root, 11, "other")
    set_pids(fake_cgroup, 11)
    sampler = ResourceSampler(["node"], fake_cgroup, output_dir, proc_root=proc_root)
    assert sampler._find_pid(sampler._targets["node"]) is None


def test_find_pid_matches_named_process(fake_cgroup, proc_root, output_dir):
    """A target resolves to the PID whose argv[0] basename matches."""
    write_proc_entry(proc_root, 11, "other")
    write_proc_entry(proc_root, 12, "/usr/lib/node", starttime=500)
    set_pids(fake_cgroup, 11, 12)
    sampler = ResourceSampler(["node"], fake_cgroup, output_dir, proc_root=proc_root)
    assert sampler._find_pid(sampler._targets["node"]) == (12, 500)


def test_find_pid_picks_earliest_started_when_several_match(
    fake_cgroup, proc_root, output_dir
):
    """When several processes share a name, the earliest started one wins."""
    write_proc_entry(proc_root, 11, "node", starttime=900)
    write_proc_entry(proc_root, 12, "node", starttime=100)
    set_pids(fake_cgroup, 11, 12)
    sampler = ResourceSampler(["node"], fake_cgroup, output_dir, proc_root=proc_root)
    assert sampler._find_pid(sampler._targets["node"]) == (12, 100)


def test_find_pid_ignores_pids_with_no_proc_entry(fake_cgroup, proc_root, output_dir):
    """PIDs that vanished between reads are skipped rather than raising."""
    write_proc_entry(proc_root, 12, "node", starttime=500)
    set_pids(fake_cgroup, 11, 12)
    sampler = ResourceSampler(["node"], fake_cgroup, output_dir, proc_root=proc_root)
    assert sampler._find_pid(sampler._targets["node"]) == (12, 500)


def test_find_pid_survives_missing_cgroup(tmp_path, proc_root, output_dir):
    """A removed cgroup resolves to no PID instead of raising."""
    sampler = ResourceSampler(
        ["node"], tmp_path / "gone", output_dir, proc_root=proc_root
    )
    assert sampler._find_pid(sampler._targets["node"]) is None


# ── sampler behaviour ─────────────────────────────────────────────────────────


def read_summary(output_dir, name="node"):
    """Read back a sampler summary YAML file."""
    return yaml.safe_load((output_dir / f"{name}.resources.yaml").read_text())


def run_sampler(fake_cgroup, proc_root, output_dir, names=("node",), ticks=3):
    """Start a sampler, let it take a few samples, and stop it.

    Args:
        fake_cgroup: Cgroup the sampler searches.
        proc_root: Fake proc root the sampler reads.
        output_dir: Directory artifacts are written to.
        names: Process names to measure.
        ticks: Approximate number of sampling intervals to wait for.

    Returns:
        The stopped sampler.
    """
    sampler = ResourceSampler(
        list(names), fake_cgroup, output_dir, interval=0.01, proc_root=proc_root
    )
    sampler.start()
    time.sleep(0.01 * ticks + 0.05)
    sampler.stop()
    return sampler


def test_sampler_writes_summary_and_series_files(fake_cgroup, proc_root, output_dir):
    """The sampler writes a YAML summary and a CSV series per target."""
    write_proc_entry(proc_root, 12, "node")
    set_pids(fake_cgroup, 12)
    run_sampler(fake_cgroup, proc_root, output_dir)
    assert (output_dir / "node.resources.yaml").exists()
    assert (output_dir / "node.resources.csv").exists()


def test_sampler_summary_reports_a_found_process(fake_cgroup, proc_root, output_dir):
    """The summary records the PID and sample count of a measured process."""
    write_proc_entry(proc_root, 12, "node", utime=TICKS, stime=0, threads=4)
    set_pids(fake_cgroup, 12)
    run_sampler(fake_cgroup, proc_root, output_dir)
    summary = read_summary(output_dir)
    assert summary["pid_found"] is True
    assert summary["pid"] == 12
    assert summary["samples"] > 0
    assert summary["cpu_user_s"] == pytest.approx(1.0)
    assert summary["max_threads"] == 4


def test_sampler_reports_pid_not_found_when_absent(fake_cgroup, proc_root, output_dir):
    """A name that never appears yields an empty summary and a header-only CSV."""
    run_sampler(fake_cgroup, proc_root, output_dir)
    summary = read_summary(output_dir)
    assert summary["pid_found"] is False
    assert summary["samples"] == 0
    assert summary["peak_rss_mib"] == 0.0
    rows = (output_dir / "node.resources.csv").read_text().splitlines()
    assert rows == ["time_s,rss_mib,cpu_user_s,cpu_sys_s,threads"]


def test_sampler_peak_rss_comes_from_vmhwm(fake_cgroup, proc_root, output_dir):
    """Peak RSS is taken from VmHWM, not from the sampled VmRSS values."""
    write_proc_entry(proc_root, 12, "node", rss_kb=1024, hwm_kb=100 * 1024)
    set_pids(fake_cgroup, 12)
    run_sampler(fake_cgroup, proc_root, output_dir)
    summary = read_summary(output_dir)
    assert summary["final_rss_mib"] == pytest.approx(1.0)
    assert summary["peak_rss_mib"] == pytest.approx(100.0)


def test_sampler_detects_pid_reuse(fake_cgroup, proc_root, output_dir):
    """A latched PID whose start time changes stops being measured."""
    write_proc_entry(proc_root, 12, "node", starttime=100)
    set_pids(fake_cgroup, 12)
    sampler = ResourceSampler(
        ["node"], fake_cgroup, output_dir, interval=0.01, proc_root=proc_root
    )
    sampler.start()
    time.sleep(0.05)
    write_proc_entry(proc_root, 12, "node", starttime=999)
    time.sleep(0.05)
    sampler.stop()
    assert read_summary(output_dir)["pid_reused"] is True


def test_sampler_marks_exited_early_when_process_disappears(
    fake_cgroup, proc_root, output_dir
):
    """A process that vanishes mid-run is reported as having exited early."""
    write_proc_entry(proc_root, 12, "node")
    set_pids(fake_cgroup, 12)
    sampler = ResourceSampler(
        ["node"], fake_cgroup, output_dir, interval=0.01, proc_root=proc_root
    )
    sampler.start()
    time.sleep(0.05)
    shutil.rmtree(proc_root / "12")
    time.sleep(0.05)
    sampler.stop()
    assert read_summary(output_dir)["exited_early"] is True


def test_sampler_survives_cgroup_removal(fake_cgroup, proc_root, output_dir):
    """Removing the cgroup under a running sampler does not raise."""
    write_proc_entry(proc_root, 12, "node")
    set_pids(fake_cgroup, 12)
    sampler = ResourceSampler(
        ["node"], fake_cgroup, output_dir, interval=0.01, proc_root=proc_root
    )
    sampler.start()
    time.sleep(0.03)
    shutil.rmtree(fake_cgroup)
    time.sleep(0.03)
    sampler.stop()
    assert (output_dir / "node.resources.yaml").exists()


def test_sampler_measures_several_targets(fake_cgroup, proc_root, output_dir):
    """One sampler measures every named target it was given."""
    write_proc_entry(proc_root, 12, "node_a")
    write_proc_entry(proc_root, 13, "node_b")
    set_pids(fake_cgroup, 12, 13)
    run_sampler(fake_cgroup, proc_root, output_dir, names=("node_a", "node_b"))
    assert read_summary(output_dir, "node_a")["pid"] == 12
    assert read_summary(output_dir, "node_b")["pid"] == 13


def test_sampler_series_rows_follow_the_header(fake_cgroup, proc_root, output_dir):
    """Every series row has one value per declared column."""
    write_proc_entry(proc_root, 12, "node")
    set_pids(fake_cgroup, 12)
    run_sampler(fake_cgroup, proc_root, output_dir)
    rows = (output_dir / "node.resources.csv").read_text().splitlines()
    assert len(rows) > 1
    for row in rows[1:]:
        assert len(row.split(",")) == len(rows[0].split(","))


def test_sampler_sanitizes_process_name_in_filenames(
    fake_cgroup, proc_root, output_dir
):
    """Characters unsafe in a filename are replaced in artifact names."""
    run_sampler(fake_cgroup, proc_root, output_dir, names=("weird/name",))
    assert (output_dir / "weird_name.resources.yaml").exists()


def test_sampler_stop_is_idempotent(fake_cgroup, proc_root, output_dir):
    """Calling stop twice is harmless."""
    sampler = run_sampler(fake_cgroup, proc_root, output_dir)
    sampler.stop()


def test_sampler_stop_without_start_writes_nothing(fake_cgroup, proc_root, output_dir):
    """A sampler that was never started writes no artifacts on stop."""
    ResourceSampler(["node"], fake_cgroup, output_dir, proc_root=proc_root).stop()
    assert not list(output_dir.iterdir())


# ── regression guards for the Thread._stop shadowing bug ──────────────────────


def test_sampler_is_not_a_thread_subclass():
    """ResourceSampler owns a thread rather than subclassing one."""
    assert not issubclass(ResourceSampler, threading.Thread)


def test_sampler_stop_event_is_an_event(fake_cgroup, output_dir):
    """The stop event is named _stop_event, not _stop."""
    sampler = ResourceSampler(["node"], fake_cgroup, output_dir)
    assert isinstance(sampler._stop_event, threading.Event)
    assert not hasattr(sampler, "_stop")


def test_sampler_start_stop_round_trip_does_not_raise(
    fake_cgroup, proc_root, output_dir
):
    """A start/stop round trip completes; shadowing _stop used to break join()."""
    run_sampler(fake_cgroup, proc_root, output_dir, ticks=1)


# ── a real process, without needing a real cgroup ─────────────────────────────


def test_sampler_measures_the_current_process(fake_cgroup, output_dir):
    """The sampler measures a real process read from the real /proc."""
    name = read_name(os.getpid(), Path("/proc"))
    set_pids(fake_cgroup, os.getpid())
    sampler = ResourceSampler([name], fake_cgroup, output_dir, interval=0.01)
    sampler.start()
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        pass  # burn CPU so the measurement is non-zero
    sampler.stop()
    summary = read_summary(output_dir, name)
    assert summary["pid_found"] is True
    assert summary["pid"] == os.getpid()
    assert summary["cpu_user_s"] > 0
    assert summary["peak_rss_mib"] > 0
    assert summary["max_threads"] >= 1
