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

"""Unit tests for lambkin.data.results.

The ``test_reads_real_*`` tests parse archives genuinely written by evo 1.34.3.
They are what keeps the reader honest now that evo isn't installed at all.
"""

import io
import zipfile
from pathlib import Path

import numpy as np
import pytest

from lambkin.data import results

FIXTURES = Path(__file__).parent / "fixtures" / "evo-1.34.3"

# Written for every metric, whatever the alignment mode.
ALWAYS_PRESENT = {
    "error_array",
    "seconds_from_start",
    "timestamps",
    "distances_from_start",
    "distances",
}


@pytest.fixture
def result():
    """A Result with mixed array dtypes and some free-form info."""
    return results.Result(
        stats={"rmse": 0.08, "mean": 0.076, "sse": 0.032},
        np_arrays={
            "error_array": np.array([0.05, 0.10, 0.07]),
            "seconds_from_start": np.array([0.0, 1.0, 2.0], dtype=np.float32),
            "counts": np.array([1, 2, 3], dtype=np.int64),
        },
        info={"label": "APE (m)", "title": "APE\n(not aligned)"},
    )


def write_members(path: Path, members: dict) -> Path:
    """Write an archive from member name to raw bytes."""
    with zipfile.ZipFile(path, "w") as archive:
        for name, payload in members.items():
            archive.writestr(name, payload)
    return path


def npy_bytes(array) -> bytes:
    """Serialize an array to raw .npy bytes, pickling if it has to."""
    buffer = io.BytesIO()
    np.save(buffer, array, allow_pickle=True)
    return buffer.getvalue()


def test_round_trip(tmp_path, result):
    """Stats, arrays, dtypes and info all survive a write/read cycle."""
    results.write_result(tmp_path / "r.zip", result)
    loaded = results.read_result(tmp_path / "r.zip")

    assert loaded.stats == result.stats
    assert loaded.info == result.info
    assert set(loaded.np_arrays) == set(result.np_arrays)
    for name, original in result.np_arrays.items():
        np.testing.assert_array_equal(loaded.np_arrays[name], original)
        assert loaded.np_arrays[name].dtype == original.dtype


def test_round_trip_without_arrays(tmp_path):
    """A result carrying no arrays round-trips to an empty mapping."""
    results.write_result(tmp_path / "r.zip", results.Result(stats={}, np_arrays={}))
    loaded = results.read_result(tmp_path / "r.zip")
    assert (loaded.stats, loaded.np_arrays) == ({}, {})


def test_round_trip_keeps_non_finite_stats(tmp_path):
    """NaN and infinity survive, as a degenerate trajectory may produce them."""
    written = results.Result(
        stats={"rmse": float("nan"), "max": float("inf")}, np_arrays={}
    )
    results.write_result(tmp_path / "r.zip", written)
    loaded = results.read_result(tmp_path / "r.zip")
    assert np.isnan(loaded.stats["rmse"])
    assert np.isinf(loaded.stats["max"])


def test_unknown_members_are_ignored(tmp_path, result):
    """Extra members don't break parsing, so newer evo output still loads."""
    path = tmp_path / "r.zip"
    results.write_result(path, result)
    with zipfile.ZipFile(path, "a") as archive:
        archive.writestr("plot.pdf", b"%PDF-1.4")
    assert set(results.read_result(path).np_arrays) == set(result.np_arrays)


def test_archive_without_arrays_is_not_an_error(tmp_path):
    """An archive holding only the JSON members parses to no arrays."""
    write_members(tmp_path / "r.zip", {"info.json": b"{}", "stats.json": b"{}"})
    assert results.read_result(tmp_path / "r.zip").np_arrays == {}


def test_missing_file_raises_file_not_found(tmp_path):
    """A path that doesn't exist raises FileNotFoundError, not a format error."""
    with pytest.raises(FileNotFoundError):
        results.read_result(tmp_path / "absent.zip")


@pytest.mark.parametrize(
    ("members", "message"),
    [
        ({"stats.json": b"{}"}, "missing required member info.json"),
        ({"info.json": b"{}"}, "missing required member stats.json"),
        ({"info.json": b"{oops", "stats.json": b"{}"}, "not valid UTF-8 JSON"),
        ({"info.json": b"[]", "stats.json": b"{}"}, "must hold a JSON object"),
        ({"info.json": b"{}", "stats.json": b'{"rmse": "high"}'}, "not numeric"),
    ],
)
def test_malformed_archive_raises(tmp_path, members, message):
    """Structural problems are reported as ResultFormatError."""
    write_members(tmp_path / "r.zip", members)
    with pytest.raises(results.ResultFormatError, match=message):
        results.read_result(tmp_path / "r.zip")


def test_not_a_zip_raises(tmp_path):
    """A file that isn't a zip archive is rejected."""
    path = tmp_path / "r.zip"
    path.write_bytes(b"definitely not a zip")
    with pytest.raises(results.ResultFormatError, match="not a valid zip"):
        results.read_result(path)


def test_truncated_array_raises(tmp_path):
    """A truncated .npy member is reported rather than crashing numpy."""
    write_members(
        tmp_path / "r.zip",
        {
            "info.json": b"{}",
            "stats.json": b"{}",
            "error_array.npy": npy_bytes(np.arange(5.0))[:20],
        },
    )
    with pytest.raises(results.ResultFormatError, match="not a readable array"):
        results.read_result(tmp_path / "r.zip")


def test_pickled_array_is_refused(tmp_path):
    """An object array is refused, never unpickled.

    Result archives are untrusted input, so reading one must not be able to
    execute code.
    """
    write_members(
        tmp_path / "r.zip",
        {
            "info.json": b"{}",
            "stats.json": b"{}",
            "error_array.npy": npy_bytes(np.array([{"exploit": True}], dtype=object)),
        },
    )
    with pytest.raises(results.ResultFormatError, match="not a readable array"):
        results.read_result(tmp_path / "r.zip")


def test_writing_object_array_raises(tmp_path):
    """Serializing an object array is refused rather than pickled."""
    result = results.Result(stats={}, np_arrays={"bad": np.array([object()])})
    with pytest.raises(results.ResultFormatError, match="needs pickling"):
        results.write_result(tmp_path / "r.zip", result)


def test_oversized_archive_is_refused(tmp_path, monkeypatch):
    """An archive that unpacks past the limit is refused before it's read."""
    monkeypatch.setattr(results, "MAX_UNCOMPRESSED_BYTES", 2)
    write_members(tmp_path / "r.zip", {"info.json": b"{}", "stats.json": b"{}"})
    with pytest.raises(results.ResultFormatError, match="over the .* byte limit"):
        results.read_result(tmp_path / "r.zip")


@pytest.mark.parametrize(
    ("name", "length"), [("ape.zip", 10), ("ape_aligned.zip", 10), ("rpe.zip", 9)]
)
def test_reads_real_evo_arrays(name, length):
    """Archives written by evo 1.34.3 parse, with the documented arrays."""
    result = results.read_result(FIXTURES / name)
    assert ALWAYS_PRESENT <= set(result.np_arrays)
    for array_name in ALWAYS_PRESENT:
        array = result.np_arrays[array_name]
        assert array.dtype == np.float64, array_name
        assert array.shape == (length,), array_name


def test_reads_real_evo_stats_and_info():
    """Statistics and metadata come back with evo's own values."""
    result = results.read_result(FIXTURES / "ape.zip")

    assert set(result.stats) == {"rmse", "mean", "median", "std", "min", "max", "sse"}
    assert result.stats["rmse"] == pytest.approx(0.05512006621367561)
    assert result.stats["min"] == 0.0
    assert result.info["label"] == "APE (m)"
    assert "\n" in result.info["title"]


def test_alignment_array_is_optional():
    """It appears only when alignment was requested, and is 2-D."""
    aligned = results.read_result(FIXTURES / "ape_aligned.zip")
    assert aligned.np_arrays["alignment_transformation_sim3"].shape == (4, 4)

    plain = results.read_result(FIXTURES / "ape.zip")
    assert "alignment_transformation_sim3" not in plain.np_arrays


def test_real_evo_archive_round_trips(tmp_path):
    """A real archive can be rewritten by us and read back unchanged."""
    original = results.read_result(FIXTURES / "ape.zip")
    results.write_result(tmp_path / "again.zip", original)
    again = results.read_result(tmp_path / "again.zip")

    assert again.stats == original.stats
    assert again.info == original.info
    for name, array in original.np_arrays.items():
        np.testing.assert_array_equal(again.np_arrays[name], array)
