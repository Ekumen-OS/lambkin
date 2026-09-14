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

"""Unit tests for lambkin.data.resources."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from lambkin.data import resources

PROCESS = "slam_node"

# Three samples, one second apart, burning half a core of user time.
SERIES_ROWS = [
    (0.0, 100.0, 0.0, 0.0, 4),
    (1.0, 150.0, 0.5, 0.1, 6),
    (2.0, 120.0, 1.0, 0.2, 5),
]


def write_summary(iter_dir: Path, process: str = PROCESS, **overrides) -> Path:
    """Write a resource summary YAML file and return its path.

    Args:
        iter_dir: Iteration directory to write into.
        process: Measured process name.
        **overrides: Summary fields to override.

    Returns:
        The path to the written file.
    """
    summary = {
        "process": process,
        "pid": 4242,
        "pid_found": True,
        "started_at": "2026-01-01T00:00:00+00:00",
        "duration_s": 2.0,
        "sample_interval_s": 1.0,
        "samples": 3,
        "peak_rss_mib": 150.0,
        "final_rss_mib": 120.0,
        "cpu_user_s": 1.0,
        "cpu_sys_s": 0.2,
        "cpu_total_s": 1.2,
        "max_threads": 6,
        "exited_early": False,
        "pid_reused": False,
    }
    summary.update(overrides)
    path = iter_dir / f"{process}.resources.yaml"
    with open(path, "w") as handle:
        yaml.dump(summary, handle, default_flow_style=False, sort_keys=False)
    return path


def write_series(iter_dir: Path, process: str = PROCESS, rows=None) -> Path:
    """Write a resource time series CSV file and return its path.

    Args:
        iter_dir: Iteration directory to write into.
        process: Measured process name.
        rows: Sample rows to write. Defaults to SERIES_ROWS.

    Returns:
        The path to the written file.
    """
    rows = SERIES_ROWS if rows is None else rows
    path = iter_dir / f"{process}.resources.csv"
    lines = ["time_s,rss_mib,cpu_user_s,cpu_sys_s,threads"]
    lines += [",".join(str(value) for value in row) for row in rows]
    path.write_text("\n".join(lines) + "\n")
    return path


def make_iteration(
    base_dir: Path, variant_index: int, iteration: int, variant: dict
) -> Path:
    """Write iteration metadata plus resource artifacts under ``base_dir``."""
    iter_dir = base_dir / f"var_{variant_index}" / f"iter_{iteration}"
    iter_dir.mkdir(parents=True)
    metadata = {
        "iteration": iteration,
        "variant": variant,
        "variant_index": variant_index - 1,
        "completed_at": "2026-01-01T00:00:00",
    }
    with open(iter_dir / "lambkin_metadata.yaml", "w") as handle:
        yaml.dump(metadata, handle)
    write_summary(iter_dir)
    write_series(iter_dir)
    return iter_dir


@pytest.fixture
def benchmark_dir(tmp_path):
    """Benchmark output tree with two variants and two iterations each."""
    make_iteration(tmp_path, 1, 1, {"sensor_model": "beam", "num_particles": 10})
    make_iteration(tmp_path, 1, 2, {"sensor_model": "beam", "num_particles": 10})
    make_iteration(tmp_path, 2, 1, {"sensor_model": "likelihood", "num_particles": 100})
    make_iteration(tmp_path, 2, 2, {"sensor_model": "likelihood", "num_particles": 100})
    return tmp_path


# ── summary ───────────────────────────────────────────────────────────────────


def test_summary_returns_one_entry_per_iteration(benchmark_dir):
    """summary() returns one entry per (variant, iteration) pair."""
    assert len(resources.summary(benchmark_dir, PROCESS)) == 4


def test_summary_entry_has_expected_attributes(benchmark_dir):
    """Each summary entry exposes the expected attributes."""
    entry = resources.summary(benchmark_dir, PROCESS)[0]
    for attr in (
        "iter_dir",
        "variant",
        "iteration",
        "params",
        "pid",
        "pid_found",
        "samples",
        "peak_rss_mib",
        "final_rss_mib",
        "cpu_user_s",
        "cpu_sys_s",
        "cpu_total_s",
        "max_threads",
        "exited_early",
        "pid_reused",
    ):
        assert hasattr(entry, attr), f"missing attribute: {attr}"


def test_summary_values_have_expected_types(benchmark_dir):
    """Summary scalars are read back with their expected types."""
    entry = resources.summary(benchmark_dir, PROCESS)[0]
    assert isinstance(entry.peak_rss_mib, float)
    assert isinstance(entry.cpu_total_s, float)
    assert isinstance(entry.max_threads, int)
    assert isinstance(entry.pid_found, bool)


def test_summary_params_match_variant(benchmark_dir):
    """Params namespace reflects the variant parameters."""
    entry = next(
        e for e in resources.summary(benchmark_dir, PROCESS) if e.variant == "var_1"
    )
    assert entry.params.sensor_model == "beam"
    assert entry.params.num_particles == 10


def test_summary_reports_a_process_that_was_never_found(tmp_path):
    """A summary written for an absent process is still returned."""
    iter_dir = make_iteration(tmp_path, 1, 1, {"a": 1})
    write_summary(iter_dir, pid=None, pid_found=False, samples=0)
    entry = resources.summary(tmp_path, PROCESS)[0]
    assert entry.pid_found is False
    assert entry.samples == 0


def test_summary_skips_missing_file(benchmark_dir):
    """summary() skips iterations with no summary file."""
    assert resources.summary(benchmark_dir, "absent_node") == []


def test_summary_empty_dir_returns_empty_list(tmp_path):
    """summary() returns an empty list when there are no iterations."""
    assert resources.summary(tmp_path, PROCESS) == []


# ── series ────────────────────────────────────────────────────────────────────


def test_series_returns_one_entry_per_iteration(benchmark_dir):
    """series() returns one entry per (variant, iteration) pair."""
    assert len(resources.series(benchmark_dir, PROCESS)) == 4


def test_series_entry_has_expected_attributes(benchmark_dir):
    """Each series entry exposes the expected attributes."""
    entry = resources.series(benchmark_dir, PROCESS)[0]
    for attr in (
        "iter_dir",
        "variant",
        "iteration",
        "params",
        "time_s",
        "rss_mib",
        "cpu_user_s",
        "cpu_sys_s",
        "cpu_total_s",
        "threads",
    ):
        assert hasattr(entry, attr), f"missing attribute: {attr}"


def test_series_arrays_are_numpy(benchmark_dir):
    """Series columns are numpy arrays."""
    entry = resources.series(benchmark_dir, PROCESS)[0]
    for attr in ("time_s", "rss_mib", "cpu_total_s", "threads"):
        assert isinstance(getattr(entry, attr), np.ndarray), attr


def test_series_array_lengths_match(benchmark_dir):
    """All series columns have the same length."""
    entry = resources.series(benchmark_dir, PROCESS)[0]
    assert len(entry.time_s) == len(SERIES_ROWS)
    assert len(entry.rss_mib) == len(entry.time_s)
    assert len(entry.cpu_total_s) == len(entry.time_s)


def test_series_cpu_total_is_the_sum_of_user_and_system(benchmark_dir):
    """cpu_total_s is the sum of the user and system columns."""
    entry = resources.series(benchmark_dir, PROCESS)[0]
    np.testing.assert_allclose(entry.cpu_total_s, entry.cpu_user_s + entry.cpu_sys_s)


def test_series_skips_missing_file(benchmark_dir):
    """series() skips iterations with no series file."""
    assert resources.series(benchmark_dir, "absent_node") == []


def test_series_skips_header_only_file(tmp_path):
    """series() skips a file that holds a header but no samples."""
    iter_dir = make_iteration(tmp_path, 1, 1, {"a": 1})
    write_series(iter_dir, rows=[])
    assert resources.series(tmp_path, PROCESS) == []


def test_series_empty_dir_returns_empty_list(tmp_path):
    """series() returns an empty list when there are no iterations."""
    assert resources.series(tmp_path, PROCESS) == []
