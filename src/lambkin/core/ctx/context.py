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

from pathlib import Path
from types import SimpleNamespace
from typing import Any

from .source import Source


class OutputInfo:
    """Output paths for this variant + iteration.

    Folders are created lazily — only when the path is first accessed.

    Attributes:
    ----------
    base_dir :
        Base output folder for benchmark.(e.g. results/).
    variant_dir:
        Root folder for this variant (e.g.
        results/var_1/).
    iteration_dir:
        Folder for the current iteration (e.g. results/var_1/iter_1/).
    """

    def __init__(self, base_dir: Path, variant_dir: Path, iteration_dir: Path) -> None:
        """Initialize the output paths for one (variant, iteration) pair.

        Parameters
        ----------
        base_dir : Path
            Base output folder for benchmark.(e.g. results/).
        variant_dir : Path
            Root output folder for this variant
            (e.g. results/var_1/).
        iteration_dir : Path
            Output folder for the current iteration (e.g.
            results/var_1/iter_1/).
        """
        self.variant_dir = Path(variant_dir)
        self.base_dir = Path(base_dir)
        self.iteration_dir = Path(iteration_dir)


def _variant_folder_name(index: int) -> str:
    """Build the variant folder name from its index."""
    return f"var_{index + 1}"


def _iteration_folder_name(iteration: int) -> str:
    """Build the per-iteration folder name."""
    return f"iter_{iteration + 1}"


class Context:
    """Carries all namespaced information for one benchmark variant.

    Builds all namespaced sub-objects (variant, inputs, options, output)
    from the given parameters and automatically creates the required output
    folders on disk.

    Attributes:
    ----------
    BENCHMARKS_DIRNAME : str
        Name for the benchmarks directory.
    variation:
        Namespaced algorithm parameters for this run.
        All key-value pairs from the variant dict are exposed as attributes.
    source:
        Source object describing the benchmark script being executed.
    inputs:
        Namespaced input information.
    options:
        Namespaced runtime options.
        All key-value pairs from the options dict are exposed as attributes.
    output:
        Namespaced output paths.
    """

    BENCHMARKS_DIRNAME = "results"

    def __init__(
        self,
        variant: dict[str, Any],
        iteration: int,
        options: dict[str, Any],
        source: Source,
        variant_index: int = 0,
        output_dir: Path | str | None = None,
    ) -> None:
        """Initialize a Context for one (variant, iteration) benchmark run.

        Only the parameters needed to build the namespaced sub-objects are
        stored at construction time. Output subfolders for variant and iteration
        are created immediately, while any other subfolder is created on first access.

        Parameters
        ----------
        variant : dict
            Algorithm parameters for this run, as defined by the user.
            All key-value pairs are exposed as attributes on "ctx.variant".
        iteration : int
            Zero-based repetition index within this variant.
            Controls the "iter_<N>" subfolder name under the variant directory.
            where "N = iteration + 1".
        source : Source
            Source object describing the benchmark script being executed.
            Its parent directory is used as the default output directory.
        options : dict
            Runtime options, as defined by the user.
            All key-value pairs are exposed as attributes on "ctx.options".
        variant_index : int, optional
            Zero-based index of this variant within the benchmark sweep.
            Controls the "var_<N>" subfolder name under the output directory,
            where "N = variant_index + 1". Defaults to 0.
        output_dir : Path or str, optional
            Root directory for all benchmark results. If not provided,
            defaults to "source.path.parent".
        """
        # ctx.variant
        self.variant = SimpleNamespace(**variant)

        # ctx.options
        self.options = SimpleNamespace(**options)

        # ctx.source
        self.source = source

        # ctx.inputs
        self.inputs = SimpleNamespace()

        # ctx.iteration
        self.iteration = iteration

        # ctx.output
        base = (
            Path(output_dir)
            if output_dir
            else source.path.parent / Context.BENCHMARKS_DIRNAME
        )
        variant_dir = base / _variant_folder_name(variant_index)
        iteration_dir = variant_dir / _iteration_folder_name(iteration)
        self.output = OutputInfo(
            base_dir=base,
            variant_dir=variant_dir,
            iteration_dir=iteration_dir,
        )

        self._setup_directories()
        # TODO(teresa-ortega): Implement a metadata file to remap folder names
        # as parameters.

        # ctx.shell
        # TODO(teresa-ortega): self.shell = Shell(self)

    def _setup_directories(self) -> None:
        """Create variant and iteration directories."""
        self.output.variant_dir.mkdir(parents=True, exist_ok=True)
        self.output.iteration_dir.mkdir(parents=True, exist_ok=True)

    def __repr__(self) -> str:
        """Return a human-readable summary of the Context state."""
        return (
            f"Context(\n"
            f"  variant       = {vars(self.variant)},\n"
            f"  iteration     = {self.iteration},\n"
            f"  source        = {self.source},\n"
            f"  inputs        = {vars(self.inputs)},\n"
            f"  options       = {vars(self.options)},\n"
            f"  variant_dir   = {self.output.variant_dir},\n"
            f"  iteration_dir = {self.output.iteration_dir}\n"
            f")"
        )
