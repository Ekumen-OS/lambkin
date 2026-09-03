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

"""Unit tests for lambkin.data.evo."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from lambkin.data import evo, results


def make_ape_zip(path: Path) -> Path:
    """Write a minimal evo_ape result zip file to ``path``.

    Holds the same arrays and statistics ``evo_ape`` writes for a real run.
    That this matches genuine evo output is covered by ``test_results.py``.

    Args:
        path: Destination path for the zip file (must not exist).

    Returns:
        The path to the written zip file.
    """
    results.write_result(
        path,
        results.Result(
            stats={
                "rmse": 0.08,
                "mean": 0.076,
                "median": 0.07,
                "std": 0.03,
                "min": 0.04,
                "max": 0.12,
                "sse": 0.032,
            },
            np_arrays={
                "error_array": np.array([0.05, 0.10, 0.07, 0.12, 0.04]),
                "seconds_from_start": np.array([0.0, 1.0, 2.0, 3.0, 4.0]),
                "distances_from_start": np.array([0.0, 0.5, 1.0, 1.5, 2.0]),
            },
        ),
    )
    return path


def make_iteration(
    base_dir: Path, variant_index: int, iteration: int, variant: dict
) -> Path:
    """Write iteration metadata and an evo_ape result zip under ``base_dir``."""
    iter_dir = base_dir / f"var_{variant_index}" / f"iter_{iteration}"
    iter_dir.mkdir(parents=True)
    metadata = {
        "iteration": iteration,
        "variant": variant,
        "variant_index": variant_index - 1,
        "completed_at": "2026-01-01T00:00:00",
    }
    with open(iter_dir / "lambkin_metadata.yaml", "w") as f:
        yaml.dump(metadata, f)
    make_ape_zip(iter_dir / "output.ape.zip")
    return iter_dir


@pytest.fixture
def benchmark_dir(tmp_path):
    """Benchmark output tree with two variants and two iterations each."""
    make_iteration(tmp_path, 1, 1, {"sensor_model": "beam", "num_particles": 10})
    make_iteration(tmp_path, 1, 2, {"sensor_model": "beam", "num_particles": 10})
    make_iteration(tmp_path, 2, 1, {"sensor_model": "likelihood", "num_particles": 100})
    make_iteration(tmp_path, 2, 2, {"sensor_model": "likelihood", "num_particles": 100})
    return tmp_path


def test_series_returns_one_entry_per_iteration(benchmark_dir):
    """series() returns one entry per (variant, iteration) pair."""
    results = evo.series(benchmark_dir, "output.ape.zip")
    assert len(results) == 4


def test_series_entry_has_expected_attributes(benchmark_dir):
    """Each series entry exposes the expected attributes."""
    entry = evo.series(benchmark_dir, "output.ape.zip")[0]
    assert hasattr(entry, "iter_dir")
    assert hasattr(entry, "variant")
    assert hasattr(entry, "iteration")
    assert hasattr(entry, "params")
    assert hasattr(entry, "time")
    assert hasattr(entry, "error")
    assert hasattr(entry, "distance")


def test_series_arrays_are_numpy(benchmark_dir):
    """time, ape, and distance are numpy arrays."""
    entry = evo.series(benchmark_dir, "output.ape.zip")[0]
    assert isinstance(entry.time, np.ndarray)
    assert isinstance(entry.error, np.ndarray)
    assert isinstance(entry.distance, np.ndarray)


def test_series_array_lengths_match(benchmark_dir):
    """Time and ape arrays have the same length."""
    entry = evo.series(benchmark_dir, "output.ape.zip")[0]
    assert len(entry.time) == len(entry.error)


def test_series_params_match_variant(benchmark_dir):
    """Params namespace reflects the variant parameters."""
    results = evo.series(benchmark_dir, "output.ape.zip")
    beam = next(r for r in results if r.variant == "var_1")
    assert beam.params.sensor_model == "beam"
    assert beam.params.num_particles == 10


def test_series_skips_missing_file(benchmark_dir):
    """series() skips iterations with no result file and returns empty list."""
    results = evo.series(benchmark_dir, "nonexistent.ape.zip")
    assert len(results) == 0


def test_series_empty_dir_returns_empty_list(tmp_path):
    """series() returns an empty list when there are no iterations."""
    assert evo.series(tmp_path, "output.ape.zip") == []


def test_stats_returns_one_entry_per_iteration(benchmark_dir):
    """stats() returns one entry per (variant, iteration) pair."""
    results = evo.stats(benchmark_dir, "output.ape.zip")
    assert len(results) == 4


def test_stats_entry_has_expected_attributes(benchmark_dir):
    """Each stats entry exposes the expected scalar attributes."""
    entry = evo.stats(benchmark_dir, "output.ape.zip")[0]
    for attr in ("rmse", "mean", "median", "std", "min", "max", "sse"):
        assert hasattr(entry, attr), f"missing attribute: {attr}"


def test_stats_values_are_floats(benchmark_dir):
    """All stat values are floats."""
    entry = evo.stats(benchmark_dir, "output.ape.zip")[0]
    for attr in ("rmse", "mean", "median", "std", "min", "max", "sse"):
        assert isinstance(getattr(entry, attr), float), f"{attr} is not a float"


def test_stats_rmse_is_positive(benchmark_dir):
    """RMSE is a positive value."""
    for entry in evo.stats(benchmark_dir, "output.ape.zip"):
        assert entry.rmse > 0


def test_stats_skips_missing_file(benchmark_dir):
    """stats() skips iterations with no result file and returns empty list."""
    results = evo.stats(benchmark_dir, "nonexistent.ape.zip")
    assert len(results) == 0


def test_stats_empty_dir_returns_empty_list(tmp_path):
    """stats() returns an empty list when there are no iterations."""
    assert evo.stats(tmp_path, "output.ape.zip") == []
