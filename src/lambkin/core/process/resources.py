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

"""Per-process resource measurement for lambkin benchmarks.

Samples CPU time, resident memory and thread count for named processes from
``/proc``, writing a YAML summary and a CSV time series per process into the
iteration directory.

Reading ``/proc`` rather than cgroup v2 accounting files is deliberate. It needs
no ``cgroup.subtree_control`` writes and no extra privileges, and it measures one
*named* process rather than a whole process set: a background ``ros2 launch``
cgroup also holds the launch machinery, and an iteration cgroup also holds bag
playback and recording. Neither is what a benchmark wants to report. The cgroup
is used only as the set of PIDs to search.
"""

from __future__ import annotations

import csv
import dataclasses
import datetime
import logging
import os
import re
import threading
import time
from collections.abc import Sequence
from pathlib import Path
from typing import IO, Any, Final

import yaml

from lambkin.common import defaults

logger = logging.getLogger(__name__)

# Clock ticks per second, used to convert /proc/<pid>/stat CPU times to seconds.
_CLOCK_TICKS: Final[float] = float(os.sysconf("SC_CLK_TCK"))

# Kibibytes per mebibyte; /proc/<pid>/status reports memory in kB.
_KIB_PER_MIB: Final[float] = 1024.0

# Column order of the time series CSV.
_SERIES_COLUMNS: Final[tuple[str, ...]] = (
    "time_s",
    "rss_mib",
    "cpu_user_s",
    "cpu_sys_s",
    "threads",
)

# Characters allowed in an artifact filename; anything else is replaced.
_UNSAFE_NAME_CHARS: Final[re.Pattern[str]] = re.compile(r"[^A-Za-z0-9._-]")


@dataclasses.dataclass(frozen=True)
class ProcessStat:
    """One reading of ``/proc/<pid>/stat``.

    Attributes:
        utime_s: User mode CPU time consumed so far, in seconds.
        stime_s: Kernel mode CPU time consumed so far, in seconds.
        num_threads: Number of threads in the process.
        starttime: Process start time in clock ticks since boot. Together with
            the PID this identifies a process across PID reuse.
    """

    utime_s: float
    stime_s: float
    num_threads: int
    starttime: int


def read_stat(pid: int, proc_root: Path) -> ProcessStat | None:
    """Read CPU times, thread count and start time for a process.

    Fields are taken from after the last ``") "`` rather than by splitting the
    whole line, because the ``comm`` field may itself contain spaces and
    parentheses.

    Args:
        pid: Process ID to read.
        proc_root: Root of the proc filesystem, ``/proc`` in production.

    Returns:
        The parsed reading, or None if the process is gone or the file is
        unreadable or malformed.
    """
    try:
        text = (proc_root / str(pid) / "stat").read_text()
    except OSError:
        return None
    try:
        fields = text[text.rindex(") ") + 2 :].split()
        # fields[0] is stat field 3 (state), so stat field N is fields[N - 3]:
        # utime=14, stime=15, num_threads=20, starttime=22.
        return ProcessStat(
            utime_s=int(fields[11]) / _CLOCK_TICKS,
            stime_s=int(fields[12]) / _CLOCK_TICKS,
            num_threads=int(fields[17]),
            starttime=int(fields[19]),
        )
    except (ValueError, IndexError):
        return None


def read_memory(pid: int, proc_root: Path) -> tuple[float, float] | None:
    """Read current and peak resident set size for a process, in MiB.

    The peak comes from ``VmHWM``, the kernel's own high water mark, so it is
    exact no matter how coarse the sampling interval is.

    Args:
        pid: Process ID to read.
        proc_root: Root of the proc filesystem, ``/proc`` in production.

    Returns:
        A ``(rss_mib, peak_rss_mib)`` tuple, or None if the process is gone or
        reports neither field, as kernel threads do.
    """
    try:
        text = (proc_root / str(pid) / "status").read_text()
    except OSError:
        return None
    rss: float | None = None
    peak: float | None = None
    for line in text.splitlines():
        if line.startswith("VmRSS:"):
            rss = _parse_kib(line)
        elif line.startswith("VmHWM:"):
            peak = _parse_kib(line)
        if rss is not None and peak is not None:
            break
    if rss is None and peak is None:
        return None
    return (rss or 0.0), (peak if peak is not None else (rss or 0.0))


def _parse_kib(line: str) -> float | None:
    """Parse a ``VmFoo:  1234 kB`` line into mebibytes.

    Args:
        line: The line to parse.

    Returns:
        The value in MiB, or None if the line is malformed.
    """
    try:
        return int(line.split()[1]) / _KIB_PER_MIB
    except (ValueError, IndexError):
        return None


def read_name(pid: int, proc_root: Path) -> str | None:
    """Return the basename of ``argv[0]`` for a process.

    ``/proc/<pid>/cmdline`` is used rather than ``/proc/<pid>/comm`` because
    ``comm`` is truncated to 15 characters, so a name such as
    ``cartographer_node`` (17 characters) would never compare equal.

    Args:
        pid: Process ID to read.
        proc_root: Root of the proc filesystem, ``/proc`` in production.

    Returns:
        The basename of ``argv[0]``, or None if the process is gone or has an
        empty command line, as kernel threads do.
    """
    try:
        raw = (proc_root / str(pid) / "cmdline").read_bytes()
    except OSError:
        return None
    argv0 = raw.split(b"\0", 1)[0].decode("utf-8", "replace")
    return os.path.basename(argv0) or None


def pids_in_cgroup(cgroup: Path) -> list[int]:
    """List the PIDs in a cgroup.

    Only the cgroup's own ``cgroup.procs`` is read. Child processes inherit
    their parent's cgroup, and lambkin never creates cgroups below a process
    cgroup, so descending would cost a directory scan per sample for nothing.

    Args:
        cgroup: The cgroup directory to read.

    Returns:
        The PIDs found, ascending. Empty if the cgroup is gone or unreadable.
    """
    try:
        tokens = (cgroup / "cgroup.procs").read_text().split()
    except OSError:
        return []
    pids = []
    for token in tokens:
        try:
            pids.append(int(token))
        except ValueError:
            continue
    return sorted(pids)


def _safe_filename(name: str) -> str:
    """Return a filename-safe form of a process name.

    Args:
        name: The process name to sanitize.

    Returns:
        The name with any character outside ``[A-Za-z0-9._-]`` replaced by an
        underscore.
    """
    return _UNSAFE_NAME_CHARS.sub("_", name)


@dataclasses.dataclass
class _Target:
    """Mutable measurement state for one named process."""

    name: str
    pid: int | None = None
    starttime: int | None = None
    samples: int = 0
    peak_rss_mib: float = 0.0
    final_rss_mib: float = 0.0
    cpu_user_s: float = 0.0
    cpu_sys_s: float = 0.0
    max_threads: int = 0
    exited_early: bool = False
    pid_reused: bool = False
    done: bool = False
    handle: IO[str] | None = None
    writer: Any = None


class ResourceSampler:
    """Samples CPU, memory and thread count for named processes in a cgroup.

    Runs one daemon thread that, on each tick, resolves every target name to a
    PID inside the given cgroup and reads that PID's ``/proc`` entries. A target
    latches onto the first matching PID and keeps it: a process that dies is
    reported as having exited early rather than followed to a replacement, since
    concatenating two processes' cumulative CPU counters would produce a
    meaningless total.

    All filesystem access is best effort. A missing process, a torn down cgroup
    or an unreadable file ends measurement for that target and is recorded in
    its summary. Nothing raises out of the sampling thread, so measurement can
    never fail a benchmark.

    On :meth:`stop` each target gets a ``<name>.resources.yaml`` summary and a
    ``<name>.resources.csv`` time series in ``output_dir``.
    """

    def __init__(
        self,
        names: Sequence[str],
        cgroup: Path,
        output_dir: Path,
        interval: float = defaults.MEASURE_INTERVAL,
        proc_root: Path = Path("/proc"),
    ) -> None:
        """Initialize the sampler without starting it or touching the disk.

        Args:
            names: Basenames of ``argv[0]`` to measure.
            cgroup: Cgroup directory whose ``cgroup.procs`` is searched.
            output_dir: Directory to write the summary and series files to.
            interval: Seconds between samples. Affects only the resolution of
                the time series; peak memory is exact at any interval.
            proc_root: Root of the proc filesystem. Overridden in tests.
        """
        self._targets: dict[str, _Target] = {name: _Target(name=name) for name in names}
        self._cgroup = cgroup
        self._output_dir = output_dir
        self._interval = interval
        self._proc_root = proc_root
        # Not `_stop`: threading.Thread defines a private _stop() that join()
        # calls internally, and shadowing it turns join() into
        # "TypeError: 'Event' object is not callable". This class owns a thread
        # rather than subclassing one, but keep the name unambiguous anyway.
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started_at: str | None = None
        self._t0: float = 0.0
        self._stopped: bool = False

    def start(self) -> None:
        """Start the sampling thread and return immediately."""
        self._started_at = datetime.datetime.now().astimezone().isoformat()
        self._t0 = time.monotonic()
        self._open_series_files()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Take a final sample, stop the thread and write the artifacts.

        Safe to call more than once, and safe to call without a prior
        :meth:`start`. Never raises: every failure is logged instead, so that
        measurement cannot turn a passing benchmark into a failing one.
        """
        if self._stopped:
            return
        self._stopped = True
        if self._thread is None:
            return
        # Sample before signalling the thread: the caller is about to kill the
        # process, after which /proc/<pid> is gone and both VmHWM and the CPU
        # counters become unreadable.
        try:
            self._sample_all()
        except Exception:
            logger.exception("Final resource sample failed.")
        self._stop_event.set()
        self._thread.join(timeout=self._interval * 2.0 + 1.0)
        if self._thread.is_alive():
            logger.warning("Resource sampling thread did not stop in time.")
        self._close_series_files()
        self._write_summaries()

    def _run(self) -> None:
        """Sample every target until stopped. Never raises."""
        try:
            tick = 1
            while not self._stop_event.is_set():
                deadline = self._t0 + tick * self._interval
                if self._stop_event.wait(max(0.0, deadline - time.monotonic())):
                    break
                self._sample_all()
                tick += 1
        except Exception:
            logger.exception("Resource sampling stopped after an unexpected error.")

    def _open_series_files(self) -> None:
        """Open one CSV per target and write its header."""
        for target in self._targets.values():
            path = self._output_dir / f"{_safe_filename(target.name)}.resources.csv"
            try:
                target.handle = open(path, "w", newline="")
            except OSError:
                logger.exception("Could not open %s for writing.", path)
                target.done = True
                continue
            target.writer = csv.writer(target.handle)
            target.writer.writerow(_SERIES_COLUMNS)
            target.handle.flush()

    def _close_series_files(self) -> None:
        """Close every open CSV."""
        for target in self._targets.values():
            if target.handle is not None:
                try:
                    target.handle.close()
                except OSError:
                    logger.exception(
                        "Could not close the series file for %s.", target.name
                    )
                target.handle = None
                target.writer = None

    def _find_pid(self, target: _Target) -> tuple[int, int] | None:
        """Find the PID of a target's process inside the monitored cgroup.

        When several processes share the name the earliest started one wins,
        breaking ties on PID, and a warning names every candidate.

        Args:
            target: The target to resolve.

        Returns:
            A ``(pid, starttime)`` tuple, or None if nothing matched.
        """
        candidates: list[tuple[int, int]] = []
        for pid in pids_in_cgroup(self._cgroup):
            if read_name(pid, self._proc_root) != target.name:
                continue
            stat = read_stat(pid, self._proc_root)
            if stat is None:
                continue  # vanished between the two reads
            candidates.append((stat.starttime, pid))
        if not candidates:
            return None
        candidates.sort()
        if len(candidates) > 1:
            logger.warning(
                "%d processes named %r in %s; measuring PID %d of %s.",
                len(candidates),
                target.name,
                self._cgroup,
                candidates[0][1],
                ", ".join(str(pid) for _, pid in candidates),
            )
        starttime, pid = candidates[0]
        return pid, starttime

    def _sample_all(self) -> None:
        """Take one sample for every target that is still being measured."""
        elapsed = time.monotonic() - self._t0
        for target in self._targets.values():
            if target.done:
                continue
            if target.pid is None:
                found = self._find_pid(target)
                if found is None:
                    continue  # not started yet, or never will
                target.pid, target.starttime = found
            self._sample_one(target, elapsed)

    def _sample_one(self, target: _Target, elapsed: float) -> None:
        """Read one sample for a latched target and record it.

        Args:
            target: The target to sample. Must already have a PID.
            elapsed: Seconds since sampling started, used as the row timestamp.
        """
        assert target.pid is not None
        stat = read_stat(target.pid, self._proc_root)
        memory = read_memory(target.pid, self._proc_root)
        if stat is None or memory is None:
            target.done = True
            target.exited_early = True
            return
        if target.starttime is not None and stat.starttime != target.starttime:
            # The PID was recycled by an unrelated process; the counters we
            # would read from here on belong to something else.
            target.done = True
            target.pid_reused = True
            return

        rss_mib, peak_rss_mib = memory
        target.samples += 1
        target.final_rss_mib = rss_mib
        target.peak_rss_mib = max(target.peak_rss_mib, peak_rss_mib)
        target.cpu_user_s = stat.utime_s
        target.cpu_sys_s = stat.stime_s
        target.max_threads = max(target.max_threads, stat.num_threads)

        if target.writer is not None and target.handle is not None:
            target.writer.writerow([
                f"{elapsed:.3f}",
                f"{rss_mib:.3f}",
                f"{stat.utime_s:.3f}",
                f"{stat.stime_s:.3f}",
                stat.num_threads,
            ])
            # Flush every row so a hard kill still leaves a valid CSV.
            target.handle.flush()

    def _write_summaries(self) -> None:
        """Write one YAML summary per target."""
        duration_s = round(time.monotonic() - self._t0, 3)
        for target in self._targets.values():
            path = self._output_dir / f"{_safe_filename(target.name)}.resources.yaml"
            summary = {
                "process": target.name,
                "pid": target.pid,
                "pid_found": target.pid is not None,
                "started_at": self._started_at,
                "duration_s": duration_s,
                "sample_interval_s": self._interval,
                "samples": target.samples,
                "peak_rss_mib": round(target.peak_rss_mib, 3),
                "final_rss_mib": round(target.final_rss_mib, 3),
                "cpu_user_s": round(target.cpu_user_s, 3),
                "cpu_sys_s": round(target.cpu_sys_s, 3),
                "cpu_total_s": round(target.cpu_user_s + target.cpu_sys_s, 3),
                "max_threads": target.max_threads,
                "exited_early": target.exited_early,
                "pid_reused": target.pid_reused,
            }
            try:
                with open(path, "w") as handle:
                    yaml.dump(
                        summary, handle, default_flow_style=False, sort_keys=False
                    )
            except OSError:
                logger.exception("Could not write %s.", path)
                continue
            if target.pid is None:
                logger.warning(
                    "No process named %r was found in %s; wrote an empty summary.",
                    target.name,
                    self._cgroup,
                )
