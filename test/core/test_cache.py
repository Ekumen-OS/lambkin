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

"""Unit tests for lambkin.core.ctx.cache."""

import pytest
import yaml

from lambkin.core.ctx.cache import compute_run_hash, is_completed
from lambkin.sdk_options import SDK_OPTIONS

METADATA_FILENAME = "lambkin_metadata.yaml"


@pytest.fixture
def variant():
    """Return a base variant dict for testing."""
    return {"sensor_model": "beam", "num_particles": 10}


@pytest.fixture
def options():
    """Return a base options dict for testing."""
    return {"clock_rate": 100.0}


@pytest.fixture
def run_hash(variant, options):
    """Return a precomputed run hash for the base variant and options."""
    return compute_run_hash(variant, 0, options)


@pytest.fixture
def metadata_path(tmp_path):
    """Return a path to a metadata file in a temporary directory."""
    return tmp_path / METADATA_FILENAME


def _write_metadata(metadata_path, run_hash, completed_at=None):
    """Helper to write a minimal metadata file for testing."""
    metadata = {"run_hash": run_hash, "started_at": "2026-01-01T00:00:00"}
    if completed_at is not None:
        metadata["completed_at"] = completed_at
    with open(metadata_path, "w") as f:
        yaml.dump(metadata, f)


def test_compute_run_hash_is_deterministic(variant, options):
    """Same inputs always produce the same hash."""
    h1 = compute_run_hash(variant, 0, options)
    h2 = compute_run_hash(variant, 0, options)
    assert h1 == h2


def test_compute_run_hash_differs_for_different_variant(options):
    """Different variant parameters produce a different hash."""
    h1 = compute_run_hash({"sensor_model": "beam"}, 0, options)
    h2 = compute_run_hash({"sensor_model": "likelihood"}, 0, options)
    assert h1 != h2


def test_compute_run_hash_differs_for_different_iteration(variant, options):
    """Different iteration index produces a different hash."""
    h1 = compute_run_hash(variant, 0, options)
    h2 = compute_run_hash(variant, 1, options)
    assert h1 != h2


def test_compute_run_hash_differs_for_different_relevant_options(variant):
    """Different non-SDK options produce a different hash."""
    h1 = compute_run_hash(variant, 0, {"clock_rate": 50.0})
    h2 = compute_run_hash(variant, 0, {"clock_rate": 100.0})
    assert h1 != h2


@pytest.mark.parametrize(
    "sdk_option",
    [(opt.name, opt.default) for opt in SDK_OPTIONS],
    ids=[opt.name for opt in SDK_OPTIONS],
)
def test_compute_run_hash_ignores_sdk_options(variant, sdk_option):
    """SDK options do not affect the hash."""
    name, default = sdk_option
    base_options = {"clock_rate": 100.0}
    h1 = compute_run_hash(variant, 0, base_options)
    h2 = compute_run_hash(variant, 0, {**base_options, name: default})
    assert h1 == h2


def test_compute_run_hash_returns_hex_string(variant, options):
    """compute_run_hash returns a non-empty hex string."""
    h = compute_run_hash(variant, 0, options)
    assert isinstance(h, str)
    assert len(h) == 64
    assert all(c in "0123456789abcdef" for c in h)


def test_is_completed_returns_false_when_file_missing(tmp_path, run_hash):
    """is_completed returns False when no metadata file exists."""
    assert not is_completed(tmp_path / METADATA_FILENAME, run_hash)


def test_is_completed_returns_false_when_completed_at_missing(metadata_path, run_hash):
    """is_completed returns False when completed_at is absent."""
    _write_metadata(metadata_path, run_hash, completed_at=None)
    assert not is_completed(metadata_path, run_hash)


def test_is_completed_returns_false_when_run_hash_mismatches(metadata_path, run_hash):
    """is_completed returns False when run_hash in file does not match."""
    _write_metadata(metadata_path, "different_hash", completed_at="2026-01-01T00:01:00")
    assert not is_completed(metadata_path, run_hash)


def test_is_completed_returns_false_when_file_is_corrupt(metadata_path, run_hash):
    """is_completed returns False gracefully when the file cannot be parsed."""
    metadata_path.write_text("{{invalid: yaml: content")
    assert not is_completed(metadata_path, run_hash)


def test_is_completed_returns_true_when_complete(metadata_path, run_hash):
    """is_completed returns True when completed_at and run_hash both match."""
    _write_metadata(metadata_path, run_hash, completed_at="2026-01-01T00:01:00")
    assert is_completed(metadata_path, run_hash)
