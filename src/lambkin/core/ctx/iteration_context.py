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
"""Iteration-scoped context.

Replaces the monolithic ``Context`` class. Owns the state and lifecycle of
a single (variant, iteration) benchmark run: cache check, cgroup setup,
shell proxy, metadata writes, and cleanup.

Users receive an ``IterationContext`` instance as the ``ctx`` argument
inside the benchmark function body.
"""

from __future__ import annotations

import datetime
import logging
import shutil
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import yaml

from lambkin.core.process.cgroup import (
    find_delegated_cgroup,
    kill_cgroup_tree,
    make_iteration_cgroup,
    remove_cgroup_tree,
)
from lambkin.core.shell import ShellProxy

from .benchmark_context import BenchmarkContext
from .cache import compute_run_hash, is_completed
from .paths import RunPaths
from .source import Source
from .variant_context import VariantContext

logger = logging.getLogger(__name__)


class IterationContext:
    """Innermost context for one (variant, iteration) benchmark run.

    Owns the cache check, cgroup, shell proxy, output paths, and metadata.
    Delegates ``variant``, ``options``, and ``source`` to its parent
    ``VariantContext`` (and transitively to ``BenchmarkContext``).

    Construction is always lightweight — all side effects (directory creation,
    cgroup setup, shell proxy) are deferred to ``__enter__``, which first
    checks the cache and short-circuits entirely on a hit.

    Attributes:
        METADATA_FILENAME: Name of the YAML metadata file written per
            iteration.
        iteration: Zero-based repetition index within this variant.
        paths: Output paths for this (variant, iteration) run.
        inputs: Namespaced input data resolved before the loop (read-only).
        shell: ShellProxy configured for this run. Only available after
            ``__enter__`` on a cache miss; raises ``AttributeError`` on a
            cache hit (``ctx.skipped is True``).
        skipped: ``True`` if this iteration was skipped due to a cache hit.
            Always check before accessing ``ctx.shell`` or writing outputs.
        variant: Namespaced algorithm parameters — delegated to
            VariantContext.
        variant_index: Zero-based variant position — delegated to
            VariantContext.
        options: Namespaced runtime options — delegated to BenchmarkContext.
        source: Benchmark source info — delegated to BenchmarkContext.
    """

    METADATA_FILENAME = "lambkin_metadata.yaml"

    def __init__(
        self,
        variant_ctx: VariantContext,
        iteration: int,
        inputs: SimpleNamespace | None = None,
    ) -> None:
        """Initialize an IterationContext from its parent VariantContext.

        Args:
            variant_ctx: The parent VariantContext for this run.
            iteration: Zero-based repetition index within this variant.
            inputs: Resolved input namespace (from
                ``BenchmarkContext.resolve_inputs``). Defaults to ``None``.
        """
        self._variant_ctx = variant_ctx
        self._iteration = iteration
        self._inputs = inputs

        self._paths = RunPaths.from_indices(
            variant_ctx.base_dir,
            variant_ctx.variant_index,
            iteration,
        )
        self._run_hash = compute_run_hash(
            vars(variant_ctx.variant),
            iteration,
            vars(variant_ctx.options),
        )

        self._shell: ShellProxy | None = None
        self._skipped: bool = False
        self._iteration_cgroup = None
        self._started_at: str | None = None

    @classmethod
    def from_params(
        cls,
        variant: dict[str, Any],
        variant_index: int,
        iteration: int,
        options: dict[str, Any],
        source: Source,
        base_dir: Path | str,
        inputs: SimpleNamespace | None = None,
    ) -> IterationContext:
        """Construct an IterationContext directly from raw parameters.

        Builds the necessary ``BenchmarkContext`` and ``VariantContext``
        parent chain internally. Useful in tests and tooling that need a
        fully configured ``IterationContext`` without going through the
        ``@benchmark`` decorator.

        Args:
            variant: Algorithm parameters for this run.
            variant_index: Zero-based index of this variant.
            iteration: Zero-based repetition index within this variant.
            options: Parsed options dict.
            source: Source object describing the benchmark script.
            base_dir: Root directory for all benchmark results.
            inputs: Optional resolved input namespace.

        Returns:
            A fully initialized ``IterationContext`` ready to be used as a
            context manager.
        """
        bctx = BenchmarkContext(source=source, options=options, base_dir=base_dir)
        vctx = VariantContext(
            benchmark_ctx=bctx,
            variant=variant,
            variant_index=variant_index,
        )
        return cls(variant_ctx=vctx, iteration=iteration, inputs=inputs)

    @property
    def iteration(self) -> int:
        """Zero-based repetition index within this variant."""
        return self._iteration

    @property
    def paths(self) -> RunPaths:
        """Output paths for this (variant, iteration) run."""
        return self._paths

    @property
    def inputs(self) -> SimpleNamespace | None:
        """Resolved input namespace, or ``None`` if no inputs are registered."""
        return self._inputs

    @property
    def shell(self) -> ShellProxy:
        """ShellProxy for this run.

        Only available after ``__enter__`` on a cache miss.

        Raises:
            AttributeError: If accessed before ``__enter__`` or on a cache
                hit (``ctx.skipped is True``).
        """
        if self._shell is None:
            raise AttributeError(
                "ctx.shell is not available: either __enter__ has not been "
                "called yet, or this iteration was skipped (ctx.skipped is True)."
            )
        return self._shell

    @property
    def skipped(self) -> bool:
        """``True`` if this iteration was skipped due to a cache hit."""
        return self._skipped

    @property
    def variant(self) -> SimpleNamespace:
        """Namespaced algorithm parameters — delegated to VariantContext."""
        return self._variant_ctx.variant

    @property
    def variant_index(self) -> int:
        """Zero-based variant position — delegated to VariantContext."""
        return self._variant_ctx.variant_index

    @property
    def options(self) -> SimpleNamespace:
        """Namespaced options delegated to BenchmarkContext via VariantContext."""
        return self._variant_ctx.options

    @property
    def source(self) -> Source:
        """Benchmark source — delegated to BenchmarkContext."""
        return self._variant_ctx.source

    @property
    def _metadata_path(self) -> Path:
        """Path to the metadata YAML file for this iteration."""
        return self._paths.iteration_dir / self.METADATA_FILENAME

    def __enter__(self) -> IterationContext:
        """Initialize the iteration, skipping if already cached.

        On a cache hit: sets ``skipped=True`` and returns immediately —
        no directories, cgroups, or metadata are touched.

        On a cache miss: removes any leftover artifacts from a previous
        failed run, creates output directories, sets up the iteration
        cgroup, configures the shell proxy, and writes the initial metadata.

        Returns:
            This IterationContext instance.
        """
        if not self.options.no_cache and is_completed(
            self._metadata_path, self._run_hash
        ):
            self._skipped = True
            logger.info(
                "var_%d/iter_%d cache hit, skipping.",
                self._variant_ctx.variant_index + 1,
                self._iteration + 1,
            )
            return self

        if not self.options.dry_run:
            self._iteration_cgroup = make_iteration_cgroup(
                find_delegated_cgroup(),
                self._paths.iteration_dir,
            )

        self._shell = ShellProxy(
            dry_run=self.options.dry_run,
            cwd=self._paths.iteration_dir,
            cgroup=self._iteration_cgroup,
            log_output=self.options.log_output,
        )

        self._started_at = datetime.datetime.now().isoformat()
        self._setup_directories()
        self._write_metadata()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Finalize the iteration, writing completion metadata and cleaning up.

        If skipped, exits immediately. Otherwise writes ``completed_at`` to
        the metadata file on clean exit, then kills and removes the iteration
        cgroup.

        Args:
            exc_type: Exception type if propagating, else ``None``.
            exc_val: Exception value if propagating, else ``None``.
            exc_tb: Exception traceback if propagating, else ``None``.
        """
        if self._skipped:
            return

        if exc_type is None and not self.options.dry_run:
            self._write_metadata(completed_at=datetime.datetime.now().isoformat())

        if self._iteration_cgroup is not None:
            kill_cgroup_tree(self._iteration_cgroup)
            remove_cgroup_tree(self._iteration_cgroup)

    def _write_metadata(self, completed_at: str | None = None) -> None:
        """Write a YAML metadata file to the iteration output directory.

        Called once at context entry and again after successful completion
        to record ``completed_at``.

        Args:
            completed_at: ISO timestamp marking successful completion.
                Omitted on the initial write.
        """
        metadata = {
            "started_at": self._started_at,
            "run_hash": self._run_hash,
            "variant_index": self._variant_ctx.variant_index,
            "iteration": self._iteration,
            "variant": vars(self._variant_ctx.variant),
            "options": vars(self._variant_ctx.options),
            "source": str(self._variant_ctx.source.path),
            "paths": {
                "base_dir": str(self._paths.base_dir),
                "variant_dir": str(self._paths.variant_dir),
                "iteration_dir": str(self._paths.iteration_dir),
            },
        }
        if completed_at is not None:
            metadata["completed_at"] = completed_at
        with open(self._metadata_path, "w") as f:
            yaml.dump(metadata, f, default_flow_style=False, sort_keys=False)

    def _setup_directories(self) -> None:
        """Create variant and iteration directories, clearing stale artifacts.

        Creates the variant directory if absent. Removes and recreates the
        iteration directory so stale outputs from a previous failed run are
        never mixed with new results.
        """
        self._paths.variant_dir.mkdir(parents=True, exist_ok=True)
        if self._paths.iteration_dir.exists():
            shutil.rmtree(self._paths.iteration_dir)
        self._paths.iteration_dir.mkdir()

    def __repr__(self) -> str:
        """Return a human-readable summary of the IterationContext state."""
        inputs = vars(self._inputs) if self._inputs is not None else None
        return (
            f"IterationContext(\n"
            f"  variant       = {vars(self.variant)},\n"
            f"  iteration     = {self._iteration},\n"
            f"  source        = {self.source},\n"
            f"  inputs        = {inputs},\n"
            f"  options       = {vars(self.options)},\n"
            f"  variant_dir   = {self._paths.variant_dir},\n"
            f"  iteration_dir = {self._paths.iteration_dir}\n"
            f")"
        )
