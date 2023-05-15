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

"""This module supplements RobotFramework ROS 2 library resource."""

from robot.api import logger
from robot.api.deco import keyword

from importlib import import_module


@keyword('Warn If ROS 2 Bag Is Compressed')
def warn_if_ros_2_bag_is_compressed(bag_path: str) -> bool:
    """
    Log a warning to console if the given ROS 2 bag is compressed.

    :param bag_path: path to ROS 2 bag to be checked.
    """
    rosbags = import_module('rosbags.rosbag2')
    reader = rosbags.Reader(bag_path)
    compression_mode = reader.compression_mode
    if compression_mode != "":
        logger.console(
            f'\n[!] {bag_path} has {compression_mode}'
            ' compression, performance may be degraded')
        return True
    return False
