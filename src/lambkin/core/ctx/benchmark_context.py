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
options, base output directory, input resolution, and variants.yaml writing.
Lives for the entire duration of the benchmark loop — entered once, exited
once.

Users never construct this directly; ``@benchmark`` creates and enters it
before the variant/iteration loops.
"""

from __future__ import annotations

import logging
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from .source import Source

logger = logging.getLogger(__name__)


class BenchmarkContext:
    """Outermost context for a benchmark run.

    Owns ``source``, ``options``, and ``base_dir`` for the entire duration
    of the benchmark loop — entered once, exited once.

    Attributes:
        source: Source object describing the benchmark script being executed.
        options: Namespaced runtime options (read-only).
        base_dir: Root directory for all benchmark results (read-only).
    """

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

    def __enter__(self) -> BenchmarkContext:
        """Create the base output directory on disk.

        Returns:
            This BenchmarkContext instance.
        """
        self._base_dir.mkdir(parents=True, exist_ok=True)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """No-op — BenchmarkContext holds no resources that need cleanup."""

    def resolve_inputs(self, inputs) -> SimpleNamespace | None:
        """Resolve all registered input hooks against this context.

        Input hooks receive this BenchmarkContext as ``ctx`` and may access
        ``ctx.base_dir`` and ``ctx.source``. No directories, cgroups, or
        metadata are created.

        Args:
            inputs: The ``InputRegistry`` holding hooks registered via
                ``@benchmark.input``.

        Returns:
            A ``SimpleNamespace`` with one attribute per registered hook,
            or ``None`` if no hooks are registered.
        """
        return inputs.resolve(self)

    def __repr__(self) -> str:
        """Return a human-readable summary of the BenchmarkContext state."""
        return (
            f"BenchmarkContext(\n"
            f"  source   = {self._source},\n"
            f"  base_dir = {self._base_dir},\n"
            f"  options  = {vars(self._options)}\n"
            f")"
        )
