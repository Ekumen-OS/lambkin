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

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any


@dataclass
class SourceInfo:
    """Source package path."""

    path: Path

    def __post_init__(self) -> None:
        """Convert path to a Path object."""
        self.path = Path(self.path)


@dataclass
class InputsInfo:
    """Input data paths."""

    dataset: Path

    def __post_init__(self) -> None:
        """Convert dataset to a Path object."""
        self.dataset = Path(self.dataset)


class OutputInfo:
    """Output paths for this variation + iteration.

    Folders are created lazily — only when the path is first accessed.
    This means a bag/ folder is only created if the user accesses
    ctx.output.bag_dir, and same for ape/ and any other subfolder.

    Attributes:
    ----------
    variation_dir:
        Root folder for this variation (e.g. results/beam_p100/).
        Created eagerly on Context instantiation.
    iteration_dir:
        Folder for the current iteration (e.g. results/beam_p100/iter_0/).
        Created eagerly on Context instantiation.
    bag_dir:
        Subfolder for rosbag output (iter_N/bag/).
        Created on first access.
    ape_dir:
        Subfolder for APE results (iter_N/ape/).
        Created on first access.
    """

    def __init__(self, variation_dir: Path, iteration_dir: Path) -> None:
        """Initialize the output paths for one (variation, iteration) pair.

        Only the base folders (variation_dir and iteration_dir) are stored
        at construction time. Subfolders like bag/ and ape/ are created
        lazily on first access via their respective properties.

        Parameters
        ----------
        variation_dir : Path
            Root output folder for this variation
            (e.g. results/beam_p100/).
        iteration_dir : Path
            Output folder for the current iteration
            (e.g. results/beam_p100/iter_0/).
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


def _variation_folder_name(variation: SimpleNamespace) -> str:
    parts = []
    for value in vars(variation).values():
        if isinstance(value, float) and value.is_integer():
            value = int(value)
        parts.append(str(value))
    return "_".join(parts)


def _iteration_folder_name(iteration: int) -> str:
    """Build the per-iteration folder name."""
    return f"iter_{iteration}"


class Context:
    """Carries all namespaced information for one benchmark variation.

    Builds all namespaced sub-objects (variation, source, inputs, option,
    output) from the given parameters and automatically creates the
    required output folders on disk.

    Attributes:
    ----------
    variation:
        Namespaced algorithm parameters for this run.
        All key-value pairs from the variation dict are exposed as attributes.
    source:
        Namespaced source package information (path to the ROS package under test).
    inputs:
        Namespaced input/dataset information (path to the rosbag file).
    option:
        Namespaced runtime options.
        All key-value pairs from the options dict are exposed as attributes.
    output:
        Namespaced output paths and metadata.
        Variation and iteration subfolders are created inside output_dir.
    """

    def __init__(
        self,
        variation: dict[str, Any],
        iteration: int,
        output_dir: Path | str,
        options: dict[str, Any],
        source_path: Path | str | None = None,
        dataset_path: Path | str | None = None,
    ) -> None:
        """Initialize a Context for one (variation, iteration) benchmark run.

        Only the parameters needed to build the namespaced sub-objects are
        stored at construction time. Output subfolders are created eagerly
        for variation and iteration, and lazily for any other subfolder.

        Parameters
        ----------
        variation : dict
            Algorithm parameters for this run, as defined by the user.
            All key-value pairs are exposed as attributes on ``ctx.variation``.
        iteration : int
            Zero-based repetition index within this variation.
            Controls the ``iter_<N>`` subfolder name under the variation directory.
        output_dir : Path or str
            Root directory for all benchmark results.
            Variation and iteration subfolders are created inside it.
        options : dict
            Runtime options, as defined by the user.
            All key-value pairs are exposed as attributes on ``ctx.options``.
        source_path : Path or str, optional
            Path to the ROS source package under test
            (e.g. ``/opt/ros/overlay/amcl``). Defaults to an empty path.
        """
        # ctx.variation
        self.variation = SimpleNamespace(**variation)

        # ctx.options
        self.options = SimpleNamespace(**options)

        # ctx.source — may be overridden by @nominal.input
        self.source = SourceInfo(path=Path(source_path) if source_path else Path())

        # ctx.inputs — may be overridden by @nominal.input
        self.inputs = InputsInfo(dataset=Path(dataset_path) if dataset_path else Path())

        # ctx.iteration
        self.iteration = iteration

        # ctx.output
        base = Path(output_dir)
        variation_dir = base / _variation_folder_name(self.variation)
        iteration_dir = variation_dir / _iteration_folder_name(iteration)
        self.output = OutputInfo(
            variation_dir=variation_dir,
            iteration_dir=iteration_dir,
        )

        self._setup_directories()

        # ctx.shell
        # TODO(teresa-ortega): self.shell = Shell(self)

    def _setup_directories(self) -> None:
        """Create variation and iteration output folders on disk."""
        self.output.variation_dir.mkdir(parents=True, exist_ok=True)
        self.output.iteration_dir.mkdir(parents=True, exist_ok=True)

    def __repr__(self) -> str:
        """Return a human-readable summary of the Context state."""
        source = getattr(self, "source", None)
        inputs = getattr(self, "inputs", None)
        return (
            f"Context(\n"
            f"  variation    = {vars(self.variation)},\n"
            f"  iteration    = {self.iteration},\n"
            f"  source       = {source.path if source else 'not set'},\n"
            f"  inputs       = {inputs.dataset if inputs else 'not set'},\n"
            f"  options      = {vars(self.options)},\n"
            f"  variation_dir= {self.output.variation_dir},\n"
            f"  iteration_dir= {self.output.iteration_dir}\n"
            f")"
        )
