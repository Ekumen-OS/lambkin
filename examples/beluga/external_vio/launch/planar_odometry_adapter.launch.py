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
"""Launch the external VIO planar odometry adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the adapter launch description."""
    config = os.path.join(
        get_package_share_directory("external_vio_adapter"),
        "config",
        "planar_odometry_adapter.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_topic",
            default_value="/ov_msckf/odomimu",
            description="External nav_msgs/Odometry topic",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/beluga/vio/odometry",
            description="Planar nav_msgs/Odometry topic consumed by Beluga",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description=(
                "Robot base frame used to transform the external odometry "
                "before projection"
            ),
        ),
        DeclareLaunchArgument(
            "use_base_frame_transform",
            default_value="true",
            description="Apply the body-to-base TF before planar projection",
        ),
        Node(
            package="external_vio_adapter",
            executable="planar_odometry_adapter",
            name="planar_odometry_adapter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_base_frame_transform")),
            parameters=[
                config,
                {
                    "input_topic": LaunchConfiguration("input_topic"),
                    "output_topic": LaunchConfiguration("output_topic"),
                    "base_frame": LaunchConfiguration("base_frame"),
                },
            ],
        ),
        Node(
            package="external_vio_adapter",
            executable="planar_odometry_adapter",
            name="planar_odometry_adapter",
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("use_base_frame_transform")),
            parameters=[
                config,
                {
                    "input_topic": LaunchConfiguration("input_topic"),
                    "output_topic": LaunchConfiguration("output_topic"),
                    "base_frame": "",
                },
            ],
        ),
    ])
