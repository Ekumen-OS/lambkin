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

"""Benchmark run output paths.

Defines RunPaths, a value object that owns all path construction logic
for a single (variant, iteration) benchmark run.
"""

from __future__ import annotations

from pathlib import Path


class RunPaths:
    """Output paths for one (variant, iteration) benchmark run.

    Attributes:
        base_dir: Base output folder for the benchmark (e.g. results/).
        variant_dir: Root folder for this variant (e.g. results/var_1/).
        iteration_dir: Folder for the current iteration (e.g. results/var_1/iter_1/).
    """

    def __init__(self, base_dir: Path, variant_dir: Path, iteration_dir: Path) -> None:
        """Initialize output paths directly from fully resolved paths.

        Args:
            base_dir: Base output folder for the benchmark (e.g. results/).
            variant_dir: Root output folder for this variant (e.g. results/var_1/).
            iteration_dir: Output folder for the current iteration
                (e.g. results/var_1/iter_1/).
        """
        self.base_dir = Path(base_dir)
        self.variant_dir = Path(variant_dir)
        self.iteration_dir = Path(iteration_dir)

    @classmethod
    def from_indices(
        cls,
        base_dir: Path | str,
        variant_index: int,
        iteration: int,
    ) -> RunPaths:
        """Derive output paths from a base directory and numeric indices.

        Constructs the canonical variant and iteration subdirectory names
        from zero-based indices (e.g. var_1/iter_1) and returns a fully
        resolved RunPaths.

        Args:
            base_dir: Root directory for all benchmark results.
            variant_index: Zero-based variant index.
            iteration: Zero-based iteration index.

        Returns:
            A fully resolved RunPaths for this (variant, iteration) pair.
        """
        base = Path(base_dir)
        variant_dir = base / f"var_{variant_index + 1}"
        iteration_dir = variant_dir / f"iter_{iteration + 1}"
        return cls(base_dir=base, variant_dir=variant_dir, iteration_dir=iteration_dir)

    def __repr__(self) -> str:
        """Return a human-readable summary of the RunPaths state."""
        return (
            f"RunPaths("
            f"base_dir={self.base_dir}, "
            f"variant_dir={self.variant_dir}, "
            f"iteration_dir={self.iteration_dir})"
        )
