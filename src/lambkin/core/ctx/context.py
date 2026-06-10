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

"""Benchmark run context.

Holds the state and lifecycle of a single benchmark execution, including
configuration, runtime metadata, and cleanup hooks. Shared across the process
layer and decorators during a run.
"""

from __future__ import annotations

import datetime
import logging
import shutil
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import yaml

from lambkin.common import defaults
from lambkin.core.process.cgroup import (
    find_delegated_cgroup,
    kill_cgroup_tree,
    make_iteration_cgroup,
    remove_cgroup_tree,
)
from lambkin.core.shell import ShellProxy

from .cache import compute_run_hash, is_completed
from .paths import RunPaths
from .source import Source

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Context:
    """Carries all namespaced information for one benchmark variant.

    Builds all namespaced sub-objects (variant, inputs, options, paths)
    from the given parameters and automatically creates the required output
    folders on disk.

    Attributes:
        METADATA_FILENAME: Name of the metadata file written per iteration.
        variant: Namespaced algorithm parameters for this run.
            All key-value pairs from the variant dict are exposed as attributes.
        options: Namespaced runtime options.
            All key-value pairs from the options dict are exposed as attributes.
        source: Source object describing the benchmark script being executed.
        inputs: Namespaced input information.
        iteration: Zero-based repetition index within this variant.
        paths: Output paths for this (variant, iteration) run.
        shell: ShellProxy instance configured for this run, with working directory
            set to the iteration output folder and cgroup set to the iteration cgroup.
            Only available after ``__enter__`` is called on a cache miss. Accessing
            it on a cache hit (``ctx.skipped is True``) raises ``AttributeError``.
        skipped: True if this iteration was skipped due to a cache hit. Always
            check ``ctx.skipped`` before accessing ``ctx.shell`` or writing any
            outputs.
    """

    METADATA_FILENAME = "lambkin_metadata.yaml"

    def __init__(
        self,
        variant: dict[str, Any],
        iteration: int,
        options: dict[str, Any],
        source: Source,
        base_dir: Path | str,
        inputs: SimpleNamespace | None = None,
        variant_index: int = 0,
    ) -> None:
        """Initialize a Context for one (variant, iteration) benchmark run.

        Construction is lightweight — no directories, cgroups, or metadata
        are created here. Full initialization is deferred to ``__enter__``,
        which first checks the cache and skips setup entirely on a hit.

        Args:
            variant (dict): Algorithm parameters for this run, as defined by the user.
                All key-value pairs are exposed as attributes on ``ctx.variant``.
            iteration (int): Zero-based repetition index within this variant.
                Controls the ``iter_<N>`` subfolder name under the variant
                directory, where ``N = iteration + 1``.
            options (dict): Runtime options, as defined by the user.
                All key-value pairs are exposed as attributes on ``ctx.options``.
            source (Source): Source object describing the benchmark script being
                executed. Its parent directory is used as the default output base
                directory.
            base_dir (Path | str): Root directory for all benchmark results.
            inputs (SimpleNamespace | None): Namespaced input information.
                Defaults to None.
            variant_index (int): Zero-based index of this variant within the
                benchmark sweep. Controls the ``var_<N>`` subfolder name under
                the output base directory, where ``N = variant_index + 1``.
                Defaults to 0.
        """
        # ctx.variant
        object.__setattr__(self, "variant", SimpleNamespace(**variant))

        # ctx.options
        object.__setattr__(self, "options", SimpleNamespace(**options))

        # ctx.source
        object.__setattr__(self, "source", source)

        # ctx.inputs
        object.__setattr__(self, "inputs", inputs)

        # ctx.iteration
        object.__setattr__(self, "iteration", iteration)

        # ctx._variant_index
        object.__setattr__(self, "_variant_index", variant_index)

        # ctx.paths — derived internally from base_dir and indices
        object.__setattr__(
            self,
            "paths",
            RunPaths.from_indices(base_dir, variant_index, iteration),
        )

        # ctx._run_hash — computed internally, never passed in by callers
        object.__setattr__(
            self,
            "_run_hash",
            compute_run_hash(variant, iteration, options),
        )

        # ctx._skipped and ctx._iteration_cgroup set in __enter__
        object.__setattr__(self, "_skipped", False)
        object.__setattr__(self, "_iteration_cgroup", None)

    def __enter__(self) -> Context:
        """Enter the context manager, initializing the iteration if not cached.

        Checks the cache first. On a hit, sets ``_skipped=True`` and returns
        without creating directories, cgroups, or metadata. On a miss, cleans
        up any leftover artifacts from a previous failed run, then performs
        full initialization: creates output directories, sets up the iteration
        cgroup, configures the shell proxy, and writes the initial metadata.

        Returns:
            This Context instance.
        """
        # Check the cache before doing any work. On a hit the iteration is
        # marked as skipped and __exit__ will be a no-op, so no directories,
        # cgroups, or metadata are created for this run.
        no_cache = getattr(self.options, "no_cache", False)
        if not no_cache and is_completed(self.metadata_path, self._run_hash):
            object.__setattr__(self, "_skipped", True)
            logger.info(
                "var_%d/iter_%d cache hit, skipping.",
                self._variant_index + 1,
                self.iteration + 1,
            )
            return self

        # Find cgroup for the current iteration
        iteration_cgroup = make_iteration_cgroup(
            find_delegated_cgroup(),
            self.paths.iteration_dir,
        )
        object.__setattr__(self, "_iteration_cgroup", iteration_cgroup)

        # ctx.shell
        object.__setattr__(
            self,
            "shell",
            ShellProxy(
                dry_run=getattr(self.options, "dry_run", defaults.DRY_RUN),
                cwd=self.paths.iteration_dir,
                cgroup=iteration_cgroup,
                log_output=getattr(self.options, "log_output", defaults.LOG_OUTPUT),
            ),
        )

        # Save the start time for this run, to be written to metadata
        object.__setattr__(self, "_started_at", datetime.datetime.now().isoformat())

        # Setup directories and remove leftovers from any previous failed run
        self._setup_directories()

        # Write metadata after all attributes are set up
        self.write_metadata()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Exit the context manager, killing and removing the iteration cgroup.

        If the iteration was skipped due to a cache hit, exits immediately.
        Otherwise marks the iteration as completed if no exception propagated,
        then kills and removes the iteration cgroup.

        Args:
            exc_type: Exception type if an exception is propagating, else None.
            exc_val: Exception value if an exception is propagating, else None.
            exc_tb: Exception traceback if an exception is propagating, else None.
        """
        if self._skipped:
            return
        dry_run = getattr(self.options, "dry_run", defaults.DRY_RUN)
        if exc_type is None and not dry_run:
            self.write_metadata(completed_at=datetime.datetime.now().isoformat())
        kill_cgroup_tree(self._iteration_cgroup)
        remove_cgroup_tree(self._iteration_cgroup)

    def write_metadata(self, completed_at: str | None = None) -> None:
        """Write a YAML metadata file to the iteration output directory.

        Serializes run identity, parameters, source, and output paths
        to ``lambkin_metadata.yaml`` inside ``paths.iteration_dir``. Called
        once at context initialization time and again after successful
        completion to add the completed_at timestamp.

        Args:
            completed_at: ISO timestamp marking successful completion. If
                provided, it is added to the metadata file. Omitted on the
                initial write at context initialization time.
        """
        metadata = {
            "started_at": self._started_at,
            "run_hash": self._run_hash,
            "variant_index": self._variant_index,
            "iteration": self.iteration,
            "variant": vars(self.variant),
            "options": vars(self.options),
            "source": str(self.source.path),
            "paths": {
                "base_dir": str(self.paths.base_dir),
                "variant_dir": str(self.paths.variant_dir),
                "iteration_dir": str(self.paths.iteration_dir),
            },
        }
        if completed_at is not None:
            metadata["completed_at"] = completed_at
        with open(self.metadata_path, "w") as f:
            yaml.dump(metadata, f, default_flow_style=False, sort_keys=False)

    @property
    def metadata_path(self) -> Path:
        """Path to the metadata file for this iteration."""
        return self.paths.iteration_dir / self.METADATA_FILENAME

    @property
    def skipped(self) -> bool:
        """Return True if this iteration was skipped due to a cache hit."""
        return self._skipped

    def _setup_directories(self) -> None:
        """Prepare variant and iteration directories for a fresh run.

        Creates the variant directory if it does not exist. Removes the
        iteration directory and all its contents if it exists from a previous
        failed run, then recreates it clean. A failed iteration never has
        completed_at written, so its contents are always safe to remove.
        """
        self.paths.variant_dir.mkdir(parents=True, exist_ok=True)
        if self.paths.iteration_dir.exists():
            shutil.rmtree(self.paths.iteration_dir)
        self.paths.iteration_dir.mkdir()

    def __repr__(self) -> str:
        """Return a human-readable summary of the Context state."""
        inputs = vars(self.inputs) if self.inputs is not None else None
        return (
            f"Context(\n"
            f"  variant       = {vars(self.variant)},\n"
            f"  iteration     = {self.iteration},\n"
            f"  source        = {self.source},\n"
            f"  inputs        = {inputs},\n"
            f"  options       = {vars(self.options)},\n"
            f"  variant_dir   = {self.paths.variant_dir},\n"
            f"  iteration_dir = {self.paths.iteration_dir}\n"
            f")"
        )
