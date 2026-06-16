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

"""Integration tests for resource monitoring of background processes."""

import json
import time

from lambkin.core.process.background import background
from lambkin.core.process.cgroup import find_delegated_cgroup, make_iteration_cgroup
from lambkin.core.shell.proxy import ShellProxy

COOPERATIVE = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "time.sleep(30)"
)


def test_resource_monitor_records_background_process(tmp_path):
    """ResourceMonitor writes a JSONL file with samples for a real background process."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with background(shell.python3, "-c", COOPERATIVE):
        time.sleep(2.5)

    jsonl_files = list(iteration_dir.glob("*.resources.jsonl"))
    assert len(jsonl_files) == 1, "Expected exactly one .resources.jsonl file"

    lines = jsonl_files[0].read_text().splitlines()
    assert len(lines) >= 2, "Expected header + at least one sample"

    header = json.loads(lines[0])
    assert header["process"] == "python3"
    assert "interval_s" in header

    samples = [json.loads(line) for line in lines[1:]]
    assert all("timestamp_s" in s for s in samples)
    assert any("rss_bytes" in s for s in samples)
    assert any(s["rss_bytes"] > 0 for s in samples)
