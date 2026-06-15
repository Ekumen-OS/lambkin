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

"""Unit tests for lambkin.data.access."""

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from lambkin.data import access


def make_iteration(base_dir: Path, variant_index: int, iteration: int, variant: dict):
    """Write a minimal lambkin_metadata.yaml for one (variant, iteration) pair."""
    iter_dir = base_dir / f"var_{variant_index}" / f"iter_{iteration}"
    iter_dir.mkdir(parents=True)
    metadata = {
        "iteration": iteration,
        "variant": variant,
    }
    with open(iter_dir / "lambkin_metadata.yaml", "w") as f:
        yaml.dump(metadata, f)
    return iter_dir


@pytest.fixture
def benchmark_dir(tmp_path):
    """Benchmark output tree with two variants and two iterations each."""
    make_iteration(tmp_path, 1, 1, {"sensor_model": "beam", "num_particles": 10})
    make_iteration(tmp_path, 1, 2, {"sensor_model": "beam", "num_particles": 10})
    make_iteration(tmp_path, 2, 1, {"sensor_model": "likelihood", "num_particles": 100})
    make_iteration(tmp_path, 2, 2, {"sensor_model": "likelihood", "num_particles": 100})
    return tmp_path


def test_iterations_returns_all_entries(benchmark_dir):
    """The iterations() returns one entry per (variant, iteration) pair."""
    entries = access.iterations(benchmark_dir)
    assert len(entries) == 4


def test_iterations_entry_has_expected_attributes(benchmark_dir):
    """Each entry exposes iter_dir, variant, iteration, and params."""
    entries = access.iterations(benchmark_dir)
    entry = entries[0]
    assert hasattr(entry, "iter_dir")
    assert hasattr(entry, "variant")
    assert hasattr(entry, "iteration")
    assert hasattr(entry, "params")


def test_iterations_iter_dir_exists(benchmark_dir):
    """The iter_dir points to an existing directory."""
    for entry in access.iterations(benchmark_dir):
        assert entry.iter_dir.is_dir()


def test_iterations_variant_name(benchmark_dir):
    """Variant attribute matches the var_N directory name."""
    entries = access.iterations(benchmark_dir)
    variant_names = {e.variant for e in entries}
    assert variant_names == {"var_1", "var_2"}


def test_iterations_params_are_namespace(benchmark_dir):
    """Params is a SimpleNamespace with variant parameter attributes."""
    entries = access.iterations(benchmark_dir)
    first = next(e for e in entries if e.variant == "var_1")
    assert isinstance(first.params, SimpleNamespace)
    assert first.params.sensor_model == "beam"
    assert first.params.num_particles == 10


def test_iterations_accepts_context_object(benchmark_dir):
    """iterations() accepts a context-like object with a base_dir attribute."""

    class FakeCtx:
        base_dir = benchmark_dir

    entries = access.iterations(FakeCtx())
    assert len(entries) == 4


def test_iterations_empty_dir_returns_empty_list(tmp_path):
    """iterations() returns an empty list when no iteration dirs exist."""
    assert access.iterations(tmp_path) == []


def test_iterations_are_sorted(benchmark_dir):
    """iterations() returns entries in sorted path order."""
    entries = access.iterations(benchmark_dir)
    paths = [e.iter_dir for e in entries]
    assert paths == sorted(paths)
