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
"""Variant-scoped context.

Owns the state that is fixed for the lifetime of one variant sweep:
the variant parameters, its 0-based index, and the variant output
directory (``var_N/``). Created and entered once per variant in the
benchmark loop, exited after all iterations of that variant are done.

Users never construct this directly; ``@benchmark`` creates and enters it
inside the variant loop.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

from .benchmark_context import BenchmarkContext


class VariantContext:
    """Mid-level context for one variant within a benchmark run.

    Owns the variant parameters and output directory. Delegates
    ``source``, ``options``, and ``base_dir`` to its parent
    ``BenchmarkContext``.

    Attributes:
        variant: Namespaced algorithm parameters for this variant (read-only).
        variant_index: Zero-based position of this variant in the full sweep
            (read-only).
        variant_dir: Output directory for this variant, e.g. ``results/var_1/``
            (read-only). Created on ``__enter__``.
    """

    def __init__(
        self,
        benchmark_ctx: BenchmarkContext,
        variant: dict[str, Any],
        variant_index: int,
    ) -> None:
        """Initialize the variant context.

        Construction is lightweight — the variant directory is not created
        here. Call ``__enter__`` (or use as a context manager) to create
        it on disk.

        Args:
            benchmark_ctx: The parent BenchmarkContext for this run.
            variant: Algorithm parameters for this variant.
            variant_index: Zero-based index of this variant in the full list.
        """
        self._benchmark_ctx = benchmark_ctx
        self._variant = SimpleNamespace(**variant)
        self._raw_variant = variant  # kept for hash / IterationContext construction
        self._variant_index = variant_index
        self._variant_dir = benchmark_ctx.base_dir / f"var_{variant_index + 1}"

    @property
    def variant(self) -> SimpleNamespace:
        """Namespaced algorithm parameters for this variant."""
        return self._variant

    @property
    def variant_index(self) -> int:
        """Zero-based index of this variant in the full benchmark sweep."""
        return self._variant_index

    @property
    def variant_dir(self) -> Path:
        """Output directory for this variant (e.g. ``results/var_1/``)."""
        return self._variant_dir

    @property
    def raw_variant(self) -> dict[str, Any]:
        """Raw variant dict (used by IterationContext and cache logic)."""
        return self._raw_variant

    @property
    def source(self):
        """Source object — delegated to BenchmarkContext."""
        return self._benchmark_ctx.source

    @property
    def options(self) -> SimpleNamespace:
        """Namespaced runtime options — delegated to BenchmarkContext."""
        return self._benchmark_ctx.options

    @property
    def base_dir(self) -> Path:
        """Root benchmark output directory — delegated to BenchmarkContext."""
        return self._benchmark_ctx.base_dir

    @property
    def raw_options(self) -> dict:
        """Raw options dict — delegated to BenchmarkContext."""
        return self._benchmark_ctx.raw_options

    def __enter__(self) -> VariantContext:
        """Create the variant output directory on disk.

        Returns:
            This VariantContext instance.
        """
        self._variant_dir.mkdir(parents=True, exist_ok=True)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """No-op — VariantContext holds no resources that need cleanup."""

    def __repr__(self) -> str:
        """Return a human-readable summary of the VariantContext state."""
        return (
            f"VariantContext(\n"
            f"  variant       = {vars(self._variant)},\n"
            f"  variant_index = {self._variant_index},\n"
            f"  variant_dir   = {self._variant_dir}\n"
            f")"
        )
