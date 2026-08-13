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

"""Launch file for the robot_localization EKF odometry baseline.

Launches robot_localization's ekf_node alone (no map, no lifecycle
manager) fusing wheel odometry and IMU data to produce a filtered
odometry estimate, for comparison against the raw /odom baseline.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_dir = get_package_share_directory("ekf_ros2")

default_yaml_path = os.path.join(pkg_dir, "params", "ekf.ros2.yaml")


def generate_launch_description():
    """Generate the launch description for the EKF odometry baseline.

    Declares a params_file launch argument and configures
    robot_localization's ekf_node, remapping its default output topic to
    a flat name so it can be evaluated the same way as other pose topics
    in this project.

    Returns:
        LaunchDescription: The complete ROS 2 launch description object.
    """
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_yaml_path,
        description="Absolute YAML file path",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("odometry/filtered", "odom_filtered")],
    )

    return LaunchDescription([
        params_file_arg,
        ekf_node,
    ])
