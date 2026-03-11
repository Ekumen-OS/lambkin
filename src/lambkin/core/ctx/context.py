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
from typing import Any


@dataclass(frozen=True)
class VariationInfo:
    """Algorithm parameters for this benchmark variation."""

    sensor_model: str
    num_particles: int


# Source path TO DO
# Dataset path TO DO
# Base Output Dir TO DO
# Options TO DO


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
        Dict with ``clock``, ``qos_option``, and ``rate``.

    """

    def __init__(
        self,
        variation: dict[str, Any],
        iteration: int,
        source_path: Path | str,
        dataset_path: Path | str,
        base_output_dir: Path | str,
        options: dict[str, Any],
    ) -> None:
        """Initialize a Context for one (variation, iteration) benchmark run.

        Builds all namespaced sub-objects (variation, source, inputs, options,
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
        base_output_dir : Path or str
            Root directory where all benchmark results are written.
            The variation and iteration subfolders are created inside it.
        options : dict
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

        self.iteration = iteration

    def add_variation(self, variation: dict) -> None:
        """Set variation parameters from a dict onto ctx.variation.

        Examples:
        --------
        ctx.add_variation({"sensor_model": "beam", "num_particles": 10})
        ctx.variation.sensor_model ==> 'beam'
        """
        for key, value in variation.items():
            setattr(self.variation, key, value)
