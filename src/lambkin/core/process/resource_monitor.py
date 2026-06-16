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

"""Resource monitoring for lambkin benchmark processes.

Polls cgroup v2 accounting files at a fixed interval while a process runs,
appending one JSON object per tick to a JSONL file in the iteration directory.
One monitor instance per process; started in BackgroundProcess.__enter__
and stopped in BackgroundProcess.__exit__.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path

from lambkin.common import defaults

logger = logging.getLogger(__name__)


class ResourceMonitor:
    """Polls cgroup v2 files and records resource usage samples to a JSONL file.

    Runs a daemon thread that wakes up every ``interval`` seconds, reads
    memory and CPU accounting files from the given cgroup, and appends one
    JSON object per sample to the output file. The file is kept open for
    the duration of the run and flushed after every write, so partial data
    is always available for inspection even if the process terminates
    unexpectedly.

    The first line of the output file is a header object containing the
    process name and sampling interval. Subsequent lines are sample objects.

    The output file is only created if ``start()`` is called. If no samples
    were collected before ``stop()`` is called (e.g. the process exited
    before the first tick), a warning is logged.

    Args:
        process_name: Human-readable name for the monitored process, stored
            in the output file header and used in log messages.
        cgroup: The process cgroup directory to monitor.
        output_path: Path to write the JSONL output file.
        interval: Sampling interval in seconds.
    """

    _CPU_STAT_KEYS = frozenset({
        "usage_usec",
        "user_usec",
        "system_usec",
        "nr_periods",
        "nr_throttled",
        "throttled_usec",
    })

    def __init__(
        self,
        process_name: str,
        cgroup: Path,
        output_path: Path,
        interval: float = defaults.RESOURCE_MONITOR_INTERVAL,
    ) -> None:
        self._process_name = process_name
        self._cgroup = cgroup
        self._output_path = output_path
        self._interval = interval
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sample_count: int = 0

    def _read_memory_current(self) -> int | None:
        """Read current memory usage in bytes from cgroup v2 memory.current.

        Returns:
            Current RSS in bytes, or None if the file is unavailable.
        """
        try:
            return int((self._cgroup / "memory.current").read_text().strip())
        except (OSError, ValueError):
            return None

    def _read_cpu_stat(self) -> dict[str, int]:
        """Read CPU usage statistics from cgroup v2 cpu.stat.

        Parses usage_usec, user_usec, system_usec, nr_periods, nr_throttled,
        and throttled_usec. The throttling fields are included because a
        non-zero nr_throttled indicates the process was CPU-rate-limited by
        the kernel, which would directly affect benchmark results.

        Returns:
            Dict with any of the above keys that were present and parseable,
            or empty dict if the file is unavailable.
        """
        result = {}
        try:
            for line in (self._cgroup / "cpu.stat").read_text().splitlines():
                parts = line.split()
                if len(parts) == 2 and parts[0] in self._CPU_STAT_KEYS:
                    result[parts[0]] = int(parts[1])
        except (OSError, ValueError):
            pass
        return result

    def sample(self) -> dict:
        """Collect one resource sample from the cgroup.

        Returns:
            Dict with timestamp_s (wall clock) and any available cgroup metrics.
            Elapsed time can be computed by subtracting the first sample's
            timestamp_s from subsequent ones.
        """
        result: dict = {"timestamp_s": time.time()}

        rss = self._read_memory_current()
        if rss is not None:
            result["rss_bytes"] = rss

        result.update(self._read_cpu_stat())

        return result

    def _run(self) -> None:
        """Sampling thread body: open file, write header, poll until stopped."""
        with self._output_path.open("w") as f:
            header = {
                "process": self._process_name,
                "interval_s": self._interval,
            }
            f.write(json.dumps(header) + "\n")
            f.flush()

            while not self._stop.wait(self._interval):
                f.write(json.dumps(self.sample()) + "\n")
                f.flush()
                self._sample_count += 1

    def start(self) -> None:
        """Start the background sampling thread.

        Launches a daemon thread that polls the cgroup at each interval
        until ``stop()`` is called.
        """
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the sampling thread.

        Signals the thread to exit and blocks until it has done so.
        Logs a warning if no samples were collected.
        """
        self._stop.set()
        if self._thread is not None:
            self._thread.join()

        if self._sample_count == 0:
            logger.warning(
                "ResourceMonitor: no samples collected for %s — "
                "process may have exited before the first poll tick.",
                self._process_name,
            )
            return

        logger.debug(
            "ResourceMonitor: %d sample(s) written to %s",
            self._sample_count,
            self._output_path,
        )
