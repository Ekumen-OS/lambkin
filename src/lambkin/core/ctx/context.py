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

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from shell.proxy import Shell


@dataclass
class VariationInfo:
    """Algorithm parameters for this benchmark variation."""

    sensor_model: str
    num_particles: int


@dataclass
class OptionsInfo:
    """Runtime options for this benchmark run."""

    clock: bool
    qos_option_path: str
    rate: int


@dataclass(frozen=True)
class SourceInfo:
    """ROS source package path."""

    path: Path

    def __post_init__(self) -> None:
        """Convert path to a Path object after dataclass construction."""
        object.__setattr__(self, "path", Path(self.path))


@dataclass(frozen=True)
class InputsInfo:
    """Input data paths."""

    dataset: Path

    def __post_init__(self) -> None:
        """Convert dataset to a Path object after dataclass construction."""
        object.__setattr__(self, "dataset", Path(self.dataset))


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


def _variation_folder_name(sensor_model: str, num_particles: int) -> str:
    """Build the per-variation folder name."""
    return f"{sensor_model}_p{num_particles}"


def _iteration_folder_name(iteration: int) -> str:
    """Build the per-iteration folder name."""
    return f"iter_{iteration}"


class Context:
    """Carries all namespaced information for one benchmark variation.

    Parameters
    ----------
    variation:
        Dict with at least ``sensor_model`` and ``num_particles``.
    iteration:
        Current iteration index (0-based).
    source_path:
        Filesystem path to the ROS source package under test.
    dataset_path:
        Path to the rosbag file used as input.
    base_output_dir:
        Root directory where all benchmark results are written.
    options:
        Dict with ``clock``, ``qos_option_path``, and ``rate``.

    """

    def __init__(
        self,
        variation: dict[str, Any],
        iteration: int,
        source_path: Path | str,
        dataset_path: Path | str,
        output_dir: Path | str,
        options: dict[str, Any],
    ) -> None:
        """Initialize a Context for one (variation, iteration) benchmark run.

        Builds all namespaced sub-objects (variation, source, inputs, option,
        output) from the given parameters and automatically creates the
        required output folders on disk.

        Parameters
        ----------
        variation : dict
            Algorithm parameters for this run. Must contain:
            - ``sensor_model`` (str): e.g. ``"beam"`` or ``"likelihood_field"``.
            - ``num_particles`` (int): number of particles for the algorithm.
        iteration : int
            Zero-based index of the current repetition within this variation.
            Determines the ``iter_<N>`` subfolder under the variation directory.
        source_path : Path or str
            Filesystem path to the ROS source package under test
            (e.g. ``/opt/ros/overlay/amcl``).
        dataset_path : Path or str
            Path to the rosbag file used as input by the algorithm
            (e.g. ``data/rosbags/run_01.bag``).
        output_dir : Path or str
            Root directory where all benchmark results are written.
            The variation and iteration subfolders are created inside it.
        option : dict
            ROS runtime options. Recognised keys:
            - ``clock`` (bool or str): use sim clock or a clock topic name.
            - ``qos_option`` (str): ROS QoS profile (e.g. ``"sensor_data"``).
            - ``rate`` (float): rate multiplier.
        """
        # ctx.variation
        self.variation = VariationInfo(
            sensor_model=variation["sensor_model"],
            num_particles=variation["num_particles"],
        )

        # ctx.options
        self.options = OptionsInfo(
            clock=options.get("clock", False),
            qos_option=options.get("qos_option", "system_default"),
            rate=float(options.get("rate", 1.0)),
        )

        # ctx.source — may be overridden by @nomida.input
        self.source = SourceInfo(path=Path(source_path) if source_path else Path())

        # ctx.inputs — may be overridden by @nomida.input
        self.inputs = InputsInfo(dataset=Path(dataset_path) if dataset_path else Path())

        # ctx.iteration
        self.iteration = iteration

        # ctx.output
        base = Path(output_dir)
        variation_dir = base / _variation_folder_name(
            self.variation.sensor_model,
            self.variation.num_particles,
        )
        iteration_dir = variation_dir / _iteration_folder_name(iteration)
        self.output = OutputInfo(
            variation_dir=variation_dir,
            iteration_dir=iteration_dir,
        )

        self._setup_directories()

        # ctx.shell
        self.shell = Shell(self)

    def add_variation(self, variation: dict) -> None:
        """Set variation parameters from a dict onto ctx.variation."""
        for key, value in variation.items():
            setattr(self.variation, key, value)

    def add_options(self, options: dict) -> None:
        """Set option parameters from a dict onto ctx.options."""
        for key, value in options.items():
            setattr(self.options, key, value)

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
            f"  variation    = {asdict(self.variation)},\n"
            f"  iteration    = {self.iteration},\n"
            f"  source       = {source.path if source else 'not set'},\n"
            f"  inputs       = {inputs.dataset if inputs else 'not set'},\n"
            f"  options      = {asdict(self.options)},\n"
            f"  variation_dir= {self.output.variation_dir},\n"
            f"  iteration_dir= {self.output.iteration_dir}\n"
            f")"
        )
