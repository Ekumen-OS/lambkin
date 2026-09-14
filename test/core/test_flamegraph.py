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

"""Unit tests for perf flamegraph folding and rendering."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from lambkin.core.process.flamegraph import PerfProfiler, fold, render_svg

# Real `perf script` output, committed so the parser is checked against genuine
# tool output rather than only against our own idea of the format.
FIXTURE = Path(__file__).parent / "fixtures" / "perf-7.0.14" / "perf-script.txt"

# One synthetic sample in perf's exact shape: header, then leaf-first frames.
SAMPLE = """\
python3 380 39014.550818: 26062180 cpu_core/cycles/P:
\t          61cb73 leaf+0x13 (/usr/bin/python3.12)
\t          6bd119 middle+0x29 (/usr/bin/python3.12)
\t          6b1c3d outer+0xd (/usr/bin/python3.12)

"""


@pytest.fixture
def perf_script():
    """Return the committed perf script fixture."""
    return FIXTURE.read_text()


# ── folding ───────────────────────────────────────────────────────────────────


def test_fold_counts_one_sample():
    """A single sample folds to one stack with a count of one."""
    assert fold(SAMPLE) == {"outer;middle;leaf": 1}


def test_fold_orders_root_to_leaf():
    """Folded stacks read root first, the reverse of perf's output."""
    assert next(iter(fold(SAMPLE))).startswith("outer")


def test_fold_strips_offsets_and_dso():
    """Frame addresses, +offsets and the shared object are dropped."""
    stack = next(iter(fold(SAMPLE)))
    assert "+0x" not in stack
    assert "(" not in stack
    assert "61cb73" not in stack


def test_fold_aggregates_identical_stacks():
    """Identical stacks are counted rather than repeated."""
    assert fold(SAMPLE * 3) == {"outer;middle;leaf": 3}


def test_fold_handles_a_trailing_sample_without_blank_line():
    """A final sample with no trailing blank line is still counted."""
    assert fold(SAMPLE.rstrip("\n")) == {"outer;middle;leaf": 1}


def test_fold_drops_unknown_frames():
    """Frames perf could not symbolise are omitted."""
    sample = SAMPLE.replace("middle+0x29", "[unknown]")
    assert fold(sample) == {"outer;leaf": 1}


def test_fold_keeps_symbols_containing_spaces():
    """A C++ signature with spaces survives folding intact."""
    sample = SAMPLE.replace("middle+0x29", "void ns::fn(int, int)+0x29")
    assert "void ns::fn(int, int)" in next(iter(fold(sample)))


def test_fold_empty_input_returns_empty():
    """Empty input folds to no stacks."""
    assert fold("") == {}


def test_fold_reads_the_real_fixture(perf_script):
    """The committed perf output folds into stacks with sane content."""
    folded = fold(perf_script)
    assert sum(folded.values()) == 10
    assert all("[unknown]" not in stack for stack in folded)
    assert any(stack.startswith("_start") for stack in folded)


# ── rendering ─────────────────────────────────────────────────────────────────


def test_render_svg_is_valid_xml(perf_script):
    """The rendered flamegraph parses as XML."""
    ET.fromstring(render_svg(fold(perf_script), "python3"))


def test_render_svg_draws_one_rect_per_node():
    """Each distinct frame in the tree gets a rectangle, plus the root."""
    svg = render_svg({"a;b": 1, "a;c": 1}, "t")
    assert svg.count("<rect") == 5  # background, all, a, b, c


def test_render_svg_places_the_root_at_the_bottom():
    """Depth grows upwards, so the root row sits below its children."""
    svg = render_svg({"root;child": 1}, "t")
    ys = [float(r.get("y")) for r in ET.fromstring(svg).iter() if r.get("y")]
    titles = [t.text for t in ET.fromstring(svg).iter() if t.tag.endswith("title")]
    assert titles == ["all (1)", "root (1)", "child (1)"]
    assert ys[1] > ys[-1]


def test_render_svg_width_is_proportional_to_samples():
    """A frame holding half the samples is drawn half as wide as the root."""
    svg = render_svg({"a": 1, "b": 1}, "t")
    widths = sorted(
        float(r.get("width"))
        for r in ET.fromstring(svg).iter()
        if r.tag.endswith("rect")
    )
    # background, root, and two halves
    assert widths[0] == pytest.approx(widths[1])
    assert widths[2] == pytest.approx(widths[0] * 2, rel=0.01)


def test_render_svg_escapes_markup_in_names():
    """A function name containing markup is escaped, not injected."""
    svg = render_svg({"<script>": 1}, "t")
    assert "<script>" not in svg
    assert "&lt;script&gt;" in svg
    ET.fromstring(svg)


def test_render_svg_reports_the_sample_count(perf_script):
    """The caption states how many samples the graph covers."""
    assert "(10 samples)" in render_svg(fold(perf_script), "python3")


def test_render_svg_with_no_samples_still_renders():
    """An empty profile renders a captioned placeholder, not nothing."""
    svg = render_svg({}, "nothing")
    ET.fromstring(svg)
    assert "no samples" in svg


# ── profiler ──────────────────────────────────────────────────────────────────


def test_profiler_stop_without_start_does_nothing(tmp_path):
    """Stopping a profiler that never started is harmless."""
    PerfProfiler("node", tmp_path).stop()
    assert not list(tmp_path.iterdir())


def test_profiler_skips_when_perf_is_missing(tmp_path, monkeypatch, caplog):
    """With no perf on PATH, start warns and records nothing."""
    # The lambkin logger stops propagating once configure_logging has run in
    # any earlier test, which would hide the record from caplog.
    monkeypatch.setattr(logging.getLogger("lambkin"), "propagate", True)
    monkeypatch.setattr("lambkin.core.process.flamegraph.shutil.which", lambda _: None)
    profiler = PerfProfiler("node", tmp_path)
    assert profiler.available() is False
    profiler.start(1234)
    profiler.stop()
    assert not list(tmp_path.iterdir())
    assert "perf is not installed" in caplog.text
