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

"""This module provides APIs to access ROS 2 output in benchmarks."""

import warnings

from importlib import import_module
from collections.abc import Mapping
from typing import Any, Iterable, Optional, Tuple, Union

from lambkin.shepherd.data import access
from lambkin.shepherd.data.access import Location
from lambkin.shepherd.utilities import enforce_nonempty

from lambkin.shepherd.data.ros import OccupancyGridType
from lambkin.shepherd.data.ros import occupancy_grids


def messages(
    topic_names: Optional[Union[Iterable[str], str]] = None,
    target_iterations: Optional[Iterable[Location]] = None,
    bag_name: str = 'output_bag'
) -> Iterable[Tuple[Mapping, Tuple[str, Any, int]]]:
    """
    Yield ROS 2 messages for the given topics per benchmark iteration.

    A ROS 2 bag must have been recorded during benchmarks. This will be the case whenever
    working with ROS 2 based benchmarks.

    :param topic_names: names of the recorded ROS 2 opics of interest. If none is specified,
      messages for all topics in each ROS 2 bag are returned.
    :param target_iterations: locations of benchmark case iterations to target. If not provided,
      it defaults to all locations as returned by py:func:`lambkin.data.access.iterations()`.
    :param bag_name: name of the ROS 2 bags in benchmarks' output. Matches the name used in
      ROS 2 based benchmarks by default.
    :returns: an iterable over tuples of benchmark iteration metadata, as a semi-structured
      mapping, and triplets bearing topic name, message instance, and message timestamp.
    """
    if topic_names is not None:
        if isinstance(topic_names, str):
            topic_names = [topic_names]
        else:
            topic_names = list(topic_names)

    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')
    assert target_iterations is not None

    rosbag2 = import_module('rosbag2_py')
    serialization = import_module('rclpy.serialization')
    rosidl_runtime = import_module('rosidl_runtime_py.utilities')
    for path, metadata in target_iterations:
        bag_path = path / bag_name

        if not bag_path.exists():
            warnings.warn(f'{bag_path} is missing')
            continue

        reader = rosbag2.SequentialReader()
        storage_options = rosbag2.StorageOptions(str(bag_path))
        converter_options = rosbag2.ConverterOptions('', '')
        reader.open(storage_options, converter_options)

        message_types = {
            entry.name: rosidl_runtime.get_message(entry.type)
            for entry in reader.get_all_topics_and_types()
        }

        if topic_names is not None:
            message_types = {
                topic_name: message_type for topic_name, message_type
                in message_types.items() if topic_name in topic_names}

            if len(message_types) != len(topic_names):
                missing_topic_names = set(topic_names) - set(message_types)
                warnings.warn(f'{missing_topic_names} missing from {bag_path}')

            storage_filter = rosbag2.StorageFilter(topics=topic_names)
            reader.set_filter(storage_filter)

        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            msg = serialization.deserialize_message(
                data, message_types[topic_name]
            )
            yield metadata, (topic_name, msg, timestamp)


__all__ = ['OccupancyGridType', 'occupancy_grids', 'messages']
