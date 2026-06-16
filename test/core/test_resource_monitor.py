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

"""Tests for ResourceMonitor."""

import json
import time

import pytest

from lambkin.core.process.resource_monitor import ResourceMonitor


@pytest.fixture
def fake_cgroup(tmp_path):
    """Fake cgroup directory with minimal cgroup v2 accounting files."""
    cgroup = tmp_path / "fake-proc-cgroup"
    cgroup.mkdir()
    (cgroup / "cgroup.procs").write_text("")
    (cgroup / "memory.current").write_text("102400\n")
    (cgroup / "cpu.stat").write_text(
        "usage_usec 500000\n"
        "user_usec 300000\n"
        "system_usec 200000\n"
        "nr_periods 10\n"
        "nr_throttled 0\n"
        "throttled_usec 0\n"
    )
    return cgroup


@pytest.fixture
def output_path(tmp_path):
    return tmp_path / "myprocess.resources.jsonl"


def test_sample_reads_rss(fake_cgroup, output_path):
    """sample() reads rss_bytes from memory.current."""
    monitor = ResourceMonitor("myprocess", fake_cgroup, output_path)
    s = monitor.sample()
    assert s["rss_bytes"] == 102400


def test_sample_reads_cpu_stat(fake_cgroup, output_path):
    """sample() reads all expected cpu.stat fields."""
    monitor = ResourceMonitor("myprocess", fake_cgroup, output_path)
    s = monitor.sample()
    assert s["usage_usec"] == 500000
    assert s["user_usec"] == 300000
    assert s["system_usec"] == 200000
    assert s["nr_periods"] == 10
    assert s["nr_throttled"] == 0
    assert s["throttled_usec"] == 0


def test_sample_has_timestamp(fake_cgroup, output_path):
    """sample() includes a timestamp_s wall clock value."""
    monitor = ResourceMonitor("myprocess", fake_cgroup, output_path)
    before = time.time()
    s = monitor.sample()
    after = time.time()
    assert before <= s["timestamp_s"] <= after


def test_sample_missing_memory_file(tmp_path, output_path):
    """sample() omits rss_bytes if memory.current is absent."""
    cgroup = tmp_path / "no-memory-cgroup"
    cgroup.mkdir()
    (cgroup / "cpu.stat").write_text("usage_usec 0\nuser_usec 0\nsystem_usec 0\n")
    monitor = ResourceMonitor("myprocess", cgroup, output_path)
    s = monitor.sample()
    assert "rss_bytes" not in s


def test_sample_missing_cpu_file(tmp_path, output_path):
    """sample() omits cpu fields if cpu.stat is absent."""
    cgroup = tmp_path / "no-cpu-cgroup"
    cgroup.mkdir()
    (cgroup / "memory.current").write_text("1024\n")
    monitor = ResourceMonitor("myprocess", cgroup, output_path)
    s = monitor.sample()
    assert "usage_usec" not in s
    assert "rss_bytes" in s


def test_output_file_created(fake_cgroup, output_path):
    """start()/stop() creates the output JSONL file."""
    monitor = ResourceMonitor("myprocess", fake_cgroup, output_path, interval=0.05)
    monitor.start()
    time.sleep(0.15)
    monitor.stop()
    assert output_path.exists()


def test_output_header_line(fake_cgroup, output_path):
    """First line of the JSONL file is the header with process name and interval."""
    monitor = ResourceMonitor("myprocess", fake_cgroup, output_path, interval=0.05)
    monitor.start()
    time.sleep(0.15)
    monitor.stop()
    lines = output_path.read_text().splitlines()
    header = json.loads(lines[0])
    assert header["process"] == "myprocess"
    assert header["interval_s"] == 0.05


def test_output_sample_lines(fake_cgroup, output_path):
    """Subsequent lines after the header are valid sample objects."""
    monitor = ResourceMonitor("myprocess", fake_cgroup, output_path, interval=0.05)
    monitor.start()
    time.sleep(0.2)
    monitor.stop()
    lines = output_path.read_text().splitlines()
    samples = [json.loads(line) for line in lines[1:]]
    assert len(samples) >= 2
    for s in samples:
        assert "timestamp_s" in s
        assert "rss_bytes" in s


def test_output_file_readable_before_stop(fake_cgroup, output_path):
    """JSONL file contains valid lines while the monitor is still running."""
    monitor = ResourceMonitor("myprocess", fake_cgroup, output_path, interval=0.05)
    monitor.start()
    time.sleep(0.2)
    lines = [line for line in output_path.read_text().splitlines() if line]
    monitor.stop()
    assert len(lines) >= 1
    for line in lines:
        json.loads(line)  # must not raise
