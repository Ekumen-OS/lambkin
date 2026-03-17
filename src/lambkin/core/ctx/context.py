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
    """Output paths for this variation + iteration.

    Folders are created lazily — only when the path is first accessed.
    This means a bag/ folder is only created if the user accesses
    ctx.output.bag_dir, and same for metrics/ and any other subfolder.

    Attributes:
    ----------
    variation_dir:
        Root folder for this variation (e.g. <source.path.parent>/var_1/).
    iteration_dir:
        Folder for the current iteration (e.g. <source.path.parent>/var_1/iter_0/).
    bag_dir:
        Subfolder for rosbag output (iter_<N>/bag/).
        Created on first access.
    metrics_dir:
        Subfolder for metrics results (iter_<N>/metrics/).
        Created on first access.
    """

    def __init__(self, variation_dir: Path, iteration_dir: Path) -> None:
        """Initialize the output paths for one (variation, iteration) pair.

        Only the base folders (variation_dir and iteration_dir) are stored
        at construction time. Subfolders like bag/ and metrics/ are created
        lazily on first access via their respective properties.

        Parameters
        ----------
        variation_dir : Path
            Root output folder for this variation
            (e.g. <source.path.parent>/var_1/).
        iteration_dir : Path
            Output folder for the current iteration
            (e.g. <source.path.parent>/var_1/iter_0/).
        """
        self.variation_dir = Path(variation_dir)
        self.iteration_dir = Path(iteration_dir)

    def _make(self, path: Path) -> Path:
        """Create a directory and return its path."""
        path.mkdir(parents=True, exist_ok=True)
        return path

    @property
    def bag_dir(self) -> Path:
        """Path to bag/ subfolder. Created on first access."""
        return self._make(self.iteration_dir / "bag")

    @property
    def metrics_dir(self) -> Path:
        """Path to metrics/ subfolder. Created on first access."""
        return self._make(self.iteration_dir / "metrics")


def _variation_folder_name(index: int) -> str:
    """Build the variation folder name from its index."""
    return f"var_{index + 1}"


def _iteration_folder_name(iteration: int) -> str:
    """Build the per-iteration folder name."""
    return f"iter_{iteration}"


class Context:
    """Carries all namespaced information for one benchmark variation.

    Builds all namespaced sub-objects (variation, inputs, options, output)
    from the given parameters and automatically creates the required output
    folders on disk.

    Attributes:
    ----------
    variation:
        Namespaced algorithm parameters for this run.
        All key-value pairs from the variation dict are exposed as attributes.
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

    def __init__(
        self,
        variation: dict[str, Any],
        iteration: int,
        output_dir: Path | str,
        options: dict[str, Any],
        source: Source,
        variation_index: int = 0,
    ) -> None:
        """Initialize a Context for one (variation, iteration) benchmark run.

        Only the parameters needed to build the namespaced sub-objects are
        stored at construction time. Output subfolders for variation and iteration
        are created immediately, while any other subfolder is created on first access.

        Parameters
        ----------
        variation : dict
            Algorithm parameters for this run, as defined by the user.
            All key-value pairs are exposed as attributes on ``ctx.variation``.
        iteration : int
            Zero-based repetition index within this variation.
            Controls the ``iter_<N>`` subfolder name under the variation directory.
        source : Source
            Source object describing the benchmark script being executed.
            Its parent directory is used as the default output directory.
        options : dict
            Runtime options, as defined by the user.
            All key-value pairs are exposed as attributes on ``ctx.options``.
        variation_index : int, optional
            Zero-based index of this variation within the benchmark sweep.
            Controls the ``variation_<N>`` subfolder name under the output directory,
            where ``N = variation_index + 1``. Defaults to 0.
        output_dir : Path or str, optional
            Root directory for all benchmark results. If not provided,
            defaults to ``source.path.parent``.
        """
        # ctx.variation
        self.variation = SimpleNamespace(**variation)

        # ctx.options
        self.options = SimpleNamespace(**options)

        # ctx.source
        self.source = source

        # ctx.inputs
        self.inputs = SimpleNamespace()

        # ctx.iteration
        self.iteration = iteration

        # ctx.output
        base = Path(output_dir) if output_dir else source.path.parent
        variation_dir = base / _variation_folder_name(variation_index)
        iteration_dir = variation_dir / _iteration_folder_name(iteration)
        self.output = OutputInfo(
            variation_dir=variation_dir,
            iteration_dir=iteration_dir,
        )

        self._setup_directories()
        # TODO(teresa-ortega): Implement a metadata file to remap folder names
        # as parameters.

        # ctx.shell
        # TODO(teresa-ortega): self.shell = Shell(self)

    def _setup_directories(self) -> None:
        """Create variation and iteration directories."""
        self.output.variation_dir.mkdir(parents=True, exist_ok=True)
        self.output.iteration_dir.mkdir(parents=True, exist_ok=True)

    def __repr__(self) -> str:
        """Return a human-readable summary of the Context state."""
        return (
            f"Context(\n"
            f"  variation     = {vars(self.variation)},\n"
            f"  iteration     = {self.iteration},\n"
            f"  source        = {self.source},\n"
            f"  inputs        = {vars(self.inputs)},\n"
            f"  options       = {vars(self.options)},\n"
            f"  variation_dir = {self.output.variation_dir},\n"
            f"  iteration_dir = {self.output.iteration_dir}\n"
            f")"
        )
