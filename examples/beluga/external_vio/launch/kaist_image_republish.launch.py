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
"""Republish KAIST image topics as raw ROS 2 images."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _republisher(name, input_topic, output_topic, input_transport):
    input_remap = "in" if input_transport == "raw" else "in/compressed"
    return Node(
        package="image_transport",
        executable="republish",
        name=name,
        output="screen",
        arguments=[input_transport, "raw"],
        remappings=[
            (input_remap, input_topic),
            ("out", output_topic),
        ],
    )


def _setup(context):
    input_transport = LaunchConfiguration("input_transport").perform(context)
    return [
        _republisher(
            "kaist_mono_image_republish",
            LaunchConfiguration("mono_image_topic").perform(context),
            LaunchConfiguration("mono_output_topic").perform(context),
            input_transport,
        ),
        _republisher(
            "kaist_left_image_republish",
            LaunchConfiguration("left_image_topic").perform(context),
            LaunchConfiguration("left_output_topic").perform(context),
            input_transport,
        ),
        _republisher(
            "kaist_right_image_republish",
            LaunchConfiguration("right_image_topic").perform(context),
            LaunchConfiguration("right_output_topic").perform(context),
            input_transport,
        ),
    ]


def generate_launch_description():
    """Generate republishers for the mono and stereo KAIST image topics."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "mono_image_topic",
            default_value="/camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "left_image_topic",
            default_value="/camera/infra1/image_rect_raw",
        ),
        DeclareLaunchArgument(
            "right_image_topic",
            default_value="/camera/infra2/image_rect_raw",
        ),
        DeclareLaunchArgument(
            "input_transport",
            default_value="raw",
            description="Transport of the recorded KAIST image topics",
        ),
        DeclareLaunchArgument(
            "mono_output_topic", default_value="/kaist/mono/image_raw"
        ),
        DeclareLaunchArgument(
            "left_output_topic", default_value="/kaist/cam0/image_raw"
        ),
        DeclareLaunchArgument(
            "right_output_topic", default_value="/kaist/cam1/image_raw"
        ),
        OpaqueFunction(function=_setup),
    ])
