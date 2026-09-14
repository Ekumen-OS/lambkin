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

"""Flamegraph profiling of a running process with Linux perf.

``perf record -p`` attaches to an already running PID, so a process discovered
by name inside a cgroup can be profiled without wrapping its launch. On stop
the recording is turned into folded stacks and rendered as an SVG.

The SVG is written here rather than by ``flamegraph.pl`` so that nothing
copyleft is vendored into this repository. ``perf`` itself is only ever run as
an external process, the same way the ``evo_*`` tools are.
"""

from __future__ import annotations

import html
import logging
import shutil
import signal
import subprocess
from collections import Counter
from pathlib import Path
from typing import Final

logger = logging.getLogger(__name__)

# Sampling frequency passed to perf record, in hertz.
PERF_FREQUENCY: Final[int] = 99

# Seconds to wait for perf to flush its recording after being interrupted.
_PERF_FLUSH_TIMEOUT: Final[float] = 30.0

# Frames that carry no information and only widen the graph.
_DROPPED_FRAMES: Final[frozenset[str]] = frozenset({"[unknown]"})

# SVG geometry.
_WIDTH: Final[int] = 1200
_ROW_HEIGHT: Final[int] = 16
_MARGIN: Final[int] = 10
_HEADER: Final[int] = 34
# Frames narrower than this are still drawn but get no label.
_MIN_TEXT_WIDTH: Final[float] = 40.0


def fold(perf_script: str) -> dict[str, int]:
    """Collapse ``perf script`` output into folded stacks.

    Samples are blank-line separated; within one sample perf prints the leaf
    frame first, so each stack is reversed to read root to leaf.

    Args:
        perf_script: Raw stdout of ``perf script``.

    Returns:
        A mapping of ``";"``-joined stacks to their sample counts.
    """
    folded: Counter[str] = Counter()
    frames: list[str] = []
    for line in perf_script.splitlines():
        if not line.strip():
            if frames:
                folded[";".join(reversed(frames))] += 1
                frames = []
            continue
        if not line.startswith((" ", "\t")):
            continue  # the sample's header line
        symbol = _symbol(line)
        if symbol is not None:
            frames.append(symbol)
    if frames:
        folded[";".join(reversed(frames))] += 1
    return dict(folded)


def _symbol(line: str) -> str | None:
    """Extract the function name from one ``perf script`` frame line.

    A frame reads ``<address> <symbol>+<offset> (<dso>)``, and the symbol
    itself may contain spaces, as C++ signatures do.

    Args:
        line: The frame line.

    Returns:
        The function name, or None if the line carries no usable symbol.
    """
    text = line.strip()
    if " " not in text:
        return None
    text = text.split(" ", 1)[1].strip()  # drop the address
    if text.endswith(")") and " (" in text:
        text = text[: text.rindex(" (")]  # drop the dso
    symbol = text.rsplit("+", 1)[0].strip() if "+0x" in text else text.strip()
    if not symbol or symbol in _DROPPED_FRAMES:
        return None
    return symbol


class _Node:
    """One frame in the merged call tree."""

    __slots__ = ("children", "name", "value")

    def __init__(self, name: str) -> None:
        """Initialize an empty node.

        Args:
            name: Function name this node represents.
        """
        self.name = name
        self.value = 0
        self.children: dict[str, _Node] = {}

    def add(self, stack: list[str], count: int) -> None:
        """Merge one folded stack into this subtree.

        Args:
            stack: Frames from this node downwards.
            count: Sample count to attribute to the stack.
        """
        self.value += count
        if stack:
            child = self.children.setdefault(stack[0], _Node(stack[0]))
            child.add(stack[1:], count)


def _colour(name: str) -> str:
    """Return a stable warm fill colour for a function name.

    Args:
        name: Function name.

    Returns:
        An SVG ``rgb()`` colour. The same name always gets the same colour, so
        a frame keeps its colour between runs.
    """
    digest = sum(ord(c) * (i + 1) for i, c in enumerate(name))
    return f"rgb({205 + digest % 50},{digest % 170},{digest % 55})"


def render_svg(folded: dict[str, int], title: str) -> str:
    """Render folded stacks as a flamegraph SVG.

    Args:
        folded: Mapping of ``";"``-joined stacks to sample counts, as
            returned by :func:`fold`.
        title: Caption drawn at the top of the graph.

    Returns:
        The SVG document. A graph with no samples still renders, captioned as
        empty, so a missing file always means measurement did not run.
    """
    root = _Node("all")
    for stack, count in sorted(folded.items()):
        root.add(stack.split(";"), count)

    if not root.value:
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{_WIDTH}" height="60">'
            f'<text x="{_MARGIN}" y="34" font-family="monospace" font-size="12">'
            f"{html.escape(title)}: no samples</text></svg>"
        )

    # Deepest stack, plus the root row, sets the height. Rows are drawn from
    # the bottom up so the graph reads as a flamegraph rather than an icicle.
    rows = 1 + max(stack.count(";") + 1 for stack in folded)
    height = _HEADER + rows * _ROW_HEIGHT + _MARGIN
    rects: list[str] = []
    _draw(root, 0, 0.0, float(_WIDTH - 2 * _MARGIN) / root.value, rows, rects)
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{_WIDTH}" '
        f'height="{height}" font-family="monospace" font-size="11">'
        f'<rect width="{_WIDTH}" height="{height}" fill="#f8f8f8"/>'
        f'<text x="{_MARGIN}" y="20" font-size="13">{html.escape(title)} '
        f"({root.value} samples)</text>" + "".join(rects) + "</svg>"
    )


def _draw(
    node: _Node, depth: int, x: float, scale: float, rows: int, out: list[str]
) -> None:
    """Emit one node and its children.

    Args:
        node: Node to draw.
        depth: Distance from the root, zero for the root itself.
        x: Left edge in user units.
        scale: Pixels per sample.
        rows: Total rows in the graph, used to place depth 0 at the bottom.
        out: Accumulator the SVG fragments are appended to.
    """
    width = node.value * scale
    y = _HEADER + (rows - 1 - depth) * _ROW_HEIGHT
    label = ""
    if width >= _MIN_TEXT_WIDTH:
        chars = int(width / 6.5)
        text = node.name if len(node.name) <= chars else node.name[: chars - 2] + ".."
        label = (
            f'<text x="{x + _MARGIN + 3:.1f}" y="{y + 11}">{html.escape(text)}</text>'
        )
    out.append(
        f"<g><title>{html.escape(node.name)} ({node.value})</title>"
        f'<rect x="{x + _MARGIN:.1f}" y="{y}" width="{max(width, 0.4):.1f}" '
        f'height="{_ROW_HEIGHT - 1}" fill="{_colour(node.name)}"/>{label}</g>'
    )
    child_x = x
    for child in sorted(node.children.values(), key=lambda c: c.name):
        _draw(child, depth + 1, child_x, scale, rows, out)
        child_x += child.value * scale


class PerfProfiler:
    """Profiles one running process with perf and renders a flamegraph.

    Attaching with ``perf record -p`` rather than wrapping a command is what
    allows a grandchild process, discovered by name, to be profiled at all.

    Every step is best effort: if ``perf`` is missing, cannot attach, or
    produces nothing, a warning is logged and the benchmark carries on.
    """

    def __init__(
        self, name: str, output_dir: Path, frequency: int = PERF_FREQUENCY
    ) -> None:
        """Initialize the profiler without starting anything.

        Args:
            name: Process name, used for the artifact filenames and caption.
            output_dir: Directory the recording and SVG are written to.
            frequency: perf sampling frequency in hertz.
        """
        self._name = name
        self._output_dir = output_dir
        self._frequency = frequency
        self._proc: subprocess.Popen[bytes] | None = None
        self._data = output_dir / f"{name}.perf.data"

    @staticmethod
    def available() -> bool:
        """Return whether the perf binary is on PATH."""
        return shutil.which("perf") is not None

    def start(self, pid: int) -> None:
        """Attach perf to a PID. Does nothing if perf is unavailable.

        Args:
            pid: Process to profile.
        """
        if not self.available():
            logger.warning(
                "perf is not installed; no flamegraph will be written for %s.",
                self._name,
            )
            return
        try:
            self._proc = subprocess.Popen(
                [
                    "perf",
                    "record",
                    "-q",
                    "-F",
                    str(self._frequency),
                    "-g",
                    "-p",
                    str(pid),
                    "-o",
                    str(self._data),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except OSError:
            logger.exception("Could not start perf for %s.", self._name)

    def stop(self) -> None:
        """Stop perf and write the flamegraph. Never raises."""
        if self._proc is None:
            return
        try:
            # SIGINT, not SIGTERM: perf flushes its buffers on interrupt.
            self._proc.send_signal(signal.SIGINT)
            _, stderr = self._proc.communicate(timeout=_PERF_FLUSH_TIMEOUT)
        except subprocess.TimeoutExpired:
            logger.warning("perf did not flush in time for %s.", self._name)
            self._proc.kill()
            return
        except OSError:
            logger.exception("Could not stop perf for %s.", self._name)
            return
        finally:
            self._proc = None

        if not self._data.exists():
            logger.warning(
                "perf wrote no recording for %s: %s",
                self._name,
                stderr.decode(errors="replace").strip(),
            )
            return
        self._render()

    def _render(self) -> None:
        """Turn the recording into an SVG. Never raises."""
        try:
            script = subprocess.run(
                ["perf", "script", "-i", str(self._data)],
                capture_output=True,
                text=True,
                check=True,
            ).stdout
        except (OSError, subprocess.CalledProcessError):
            logger.exception("perf script failed for %s.", self._name)
            return
        folded = fold(script)
        if not folded:
            logger.warning("perf recorded no usable stacks for %s.", self._name)
        try:
            (self._output_dir / f"{self._name}.folded").write_text(
                "".join(f"{stack} {count}\n" for stack, count in sorted(folded.items()))
            )
            (self._output_dir / f"{self._name}.flamegraph.svg").write_text(
                render_svg(folded, self._name)
            )
        except OSError:
            logger.exception("Could not write the flamegraph for %s.", self._name)
