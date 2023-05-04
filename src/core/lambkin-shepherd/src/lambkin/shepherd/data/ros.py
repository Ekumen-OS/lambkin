# Copyright 2023 Ekumen, Inc.
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

"""This module provides APIs to access ROS output in benchmarks."""

import warnings

from importlib import import_module
from collections.abc import Mapping
from typing import Any, Iterable, Optional, Protocol, Tuple, Union

from numpy.typing import ArrayLike

from lambkin.shepherd.data import access
from lambkin.shepherd.data.access import Location
from lambkin.shepherd.utilities import enforce_nonempty

import numpy as np
import yaml


def messages(
    topic_names: Optional[Union[Iterable[str], str]] = None,
    target_iterations: Optional[Iterable[Location]] = None, /,
    bag_name: str = 'output.bag'
) -> Iterable[Tuple[Mapping, Any]]:
    """
    Yield ROS messages for the given topics per benchmark iteration.

    A ROS bag must have been recorded during benchmarks. This will be the case whenever
    working with ROS based benchmarks.

    :param topic_names: names of the recorded ROS topics of interest. If none is specified,
      messages for all topics in each ROS bag are returned.
    :param target_iterations: locations of benchmark case iterations to target. If not provided,
      it defaults to all locations as returned by py:func:`lambkin.data.access.iterations()`.
    :param bag_name: name of the ROS bags in benchmarks' output. Matches the name used in
      ROS based benchmarks by default.
    :returns: an iterable over tuples of benchmark iteration metadata, as a semi-structured
      mapping, and ROS bag record ie. a py:mod:`rosbag` specific tuple type bearing topic name,
      message timestamp, and message instance.
    """
    if topic_names is not None:
        if isinstance(topic_names, str):
            topic_names = [topic_names]
    else:
        topic_names = []
    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')

    for path, metadata in target_iterations:
        bag_path = path / bag_name
        if not bag_path.exists():
            warnings.warn(f'{bag_path} is missing')
            continue
        rosbag = import_module('rosbag')
        with rosbag.Bag(bag_path, 'r') as bag:
            for record in bag.read_messages(topic_names):
                yield metadata, record


class OccupancyGridType(Protocol):
    """py:class:`nav_msgs.msg.OccupancyGrid` message type stub."""

    data: ArrayLike


def occupancy_grids(
    map_name: str = 'map', target_iterations: Optional[Iterable[Location]] = None
) -> Iterable[Tuple[Mapping, OccupancyGridType]]:
    """
    Yield ROS occupancy grid maps per benchmark iteration.

    A ROS bag must have been recorded during benchmarks. This will be the case
    whenever working with ROS 2D SLAM style benchmarks.

    :param map_name: ROS map files' basename. By default, it matches the name used
      in ROS 2D SLAM style benchmarks by default.
    :param target_iterations: locations of benchmark case iterations to target.
      If not provided, it defaults to all locations as returned by
      py:func:`lambkin.data.access.iterations()`.
    :returns: an iterable over tuples of benchmark iteration metadata,
      as a semi-structured mapping, and an occupancy grid.
    """
    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')

    for path, metadata in target_iterations:
        map_metadata_path = path / f'{map_name}.yaml'
        if not map_metadata_path.exists():
            warnings.warn(f'{map_metadata_path} is missing')
            continue
        with map_metadata_path.open('r') as map_metadata_file:
            map_metadata = yaml.safe_load(map_metadata_file)
        map_image_path = path / map_metadata['image']
        if not map_image_path.exists():
            warnings.warn(f'{map_image_path} is missing')
            continue
        nav_msgs = import_module('nav_msgs.msg')
        Image = import_module('PIL.Image')
        with Image.open(map_image_path) as map_image:
            grid = nav_msgs.OccupancyGrid()
            grid.info.width = map_image.width
            grid.info.height = map_image.height
            grid.info.resolution = map_metadata['resolution']
            grid.info.origin.position.x = map_metadata['origin'][0]
            grid.info.origin.position.y = map_metadata['origin'][1]
            grid.info.origin.position.z = map_metadata['origin'][2]
            grid.data = (
                np.asarray(map_image) * 100. / 255
            ).flatten().astype(np.uint8)

            yield metadata, grid
