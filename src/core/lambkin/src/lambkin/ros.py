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

import time

import rosbag
import roslaunch.core

from robot.api import logger
from robot.api.deco import keyword
from robot.utils import timestr_to_secs

inf = float('+inf')

@keyword('Wait for ROS Master')
def wait_for_ros_master(timeout=inf):
    if isinstance(timeout, str):
        timeout = timestr_to_secs(timeout)
    deadline = time.time() + timeout
    master = roslaunch.core.Master()
    while time.time() < deadline:
        if master.is_running():
            return True
        time.sleep(min(0.1, timeout))
    return master.is_running()


@keyword('ROS Master Should Be Running')
def ros_master_should_be_running(msg=None):
    master = roslaunch.core.Master()
    if not master.is_running():
        if msg is None:
            msg = 'ROS master is not running'
        raise AssertionError(msg)


@keyword('Warn If Bagfile Is Compressed')
def warn_if_bagfile_is_compressed(bagfile_path):
    with rosbag.Bag(bagfile_path) as bagfile:
        compression_info = bagfile.get_compression_info()
        if compression_info.compression != rosbag.Compression.NONE:
            warning = '[!] {} has {} compression, performance may be degraded'.format(
                bagfile_path, compression_info.compression
            )
            hint = 'Consider decompressing it: rosbag decompress {}'.format(bagfile_path)
            logger.console(warning)
            logger.console(hint)
