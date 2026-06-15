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
"""Benchmark-scoped context.

Owns the outermost lifecycle of a benchmark run: source identity, parsed
options and base output directory.
Lives for the entire duration of the benchmark loop — entered once, exited
once.

Users never construct this directly; ``@benchmark`` creates and enters it
before the variant/iteration loops.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace, TracebackType
from typing import Any

from .source import Source


class BenchmarkContext:
    """Outermost context for a benchmark run.

    Owns ``source``, ``options``, and ``base_dir`` for the entire duration
    of the benchmark loop — entered once, exited once.

    Attributes:
        source: Source object describing the benchmark script being executed.
        options: Namespaced runtime options (read-only).
        base_dir: Root directory for all benchmark results (read-only).
        inputs: Resolved benchmark-scoped inputs. ``None`` until set by the
            registry. Read-only after resolution — raises ``AttributeError``
            if assigned again.
    """

    scope = "benchmark"

    def __init__(
        self,
        source: Source,
        options: dict[str, Any],
        base_dir: Path | str,
    ) -> None:
        """Initialize the benchmark context.

        Construction is lightweight — no directories are created here.
        Call ``__enter__`` (or use as a context manager) to create
        ``base_dir`` on disk.

        Args:
            source: Source object describing the benchmark script.
            options: Parsed options dict from CLI / ``@lambkin.option``.
            base_dir: Root directory for all benchmark results.
        """
        self._source = source
        self._options = SimpleNamespace(**options)
        self._base_dir = Path(base_dir)
        self._inputs: SimpleNamespace | None = None
        self._inputs_locked: bool = False

    @property
    def source(self) -> Source:
        """Source object describing the benchmark script."""
        return self._source

    @property
    def options(self) -> SimpleNamespace:
        """Namespaced runtime options."""
        return self._options

    @property
    def base_dir(self) -> Path:
        """Root directory for all benchmark results."""
        return self._base_dir

    @property
    def inputs(self) -> SimpleNamespace | None:
        """Resolved benchmark-scoped inputs. ``None`` until set by the registry."""
        return self._inputs

    @inputs.setter
    def inputs(self, value: SimpleNamespace) -> None:
        """Set resolved inputs. Read-only after the first assignment.

        Args:
            value: Resolved benchmark-scoped inputs from
                ``InputRegistry.resolve``.

        Raises:
            AttributeError: If called after inputs have already been set.
        """
        if self._inputs_locked:
            raise AttributeError("ctx.inputs is read-only after resolution.")
        self._inputs = value
        self._inputs_locked = True

    def __enter__(self) -> BenchmarkContext:
        """Create the base output directory on disk.

        Returns:
            This BenchmarkContext instance.
        """
        self._base_dir.mkdir(parents=True, exist_ok=True)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """No-op — BenchmarkContext holds no resources that need cleanup."""

    def __repr__(self) -> str:
        """Return a human-readable summary of the BenchmarkContext state."""
        inputs = vars(self._inputs) if self._inputs is not None else None
        return (
            f"BenchmarkContext(\n"
            f"  source   = {self._source},\n"
            f"  base_dir = {self._base_dir},\n"
            f"  options  = {vars(self._options)},\n"
            f"  inputs   = {inputs}\n"
            f")"
        )
