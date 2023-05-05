# Copyright 2022 Ekumen, Inc.
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

"""This module supplements RobotFramework ROS library resource."""

import time

import importlib

from robot.api import logger
from robot.api.deco import keyword

from typing import Optional, Union


inf = float('+inf')


@keyword('Wait For ROS Master')
def wait_for_ros_master(timeout: Union[float, str] = inf) -> bool:
    """
    Wait for the ROS master to be running.

    ROS master details are picked from the environment.

    :param timeout: optional timeout, either as a number of seconds or
      as a RobotFramework time string. Waits indefinitely by default.
    :return: whether the ROS master is running or not.
    """
    if isinstance(timeout, str):
        from robot.utils import timestr_to_secs
        timeout = timestr_to_secs(timeout)
    timeout = float(timeout)
    deadline = time.time() + timeout
    roslaunch = importlib.import_module('roslaunch')
    master = roslaunch.core.Master()
    while time.time() < deadline:
        if master.is_running():
            return True
        time.sleep(min(0.1, timeout))
    return master.is_running()


@keyword('ROS Master Should Be Running')
def ros_master_should_be_running(msg: Optional[str] = None):
    """
    Assert that a ROS master is running.

    :param msg: optional assertion message.
    """
    roslaunch = importlib.import_module('roslaunch')
    master = roslaunch.core.Master()
    if not master.is_running():
        if msg is None:
            msg = 'ROS master is not running'
        raise AssertionError(msg)


@keyword('Warn If Bagfile Is Compressed')
def warn_if_bagfile_is_compressed(bagfile_path: str) -> bool:
    """
    Log a warning to console if the given ROS bag is compressed.

    :param bagfile_path: path to ROS bag to be checked.
    """
    rosbag = importlib.import_module('rosbag')
    with rosbag.Bag(bagfile_path) as bagfile:
        compression_info = bagfile.get_compression_info()
        compression_type = compression_info.compression
        if compression_type != rosbag.Compression.NONE:
            logger.console(
                f'\n[!] {bagfile_path} has {compression_type}'
                ' compression, performance may be degraded')
            logger.console(
                f'Consider decompressing it: rosbag decompress {bagfile_path}')
            return True
        return False
