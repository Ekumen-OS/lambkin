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
"""Launch OpenVINS and the external planar odometry adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the OpenVINS and adapter launch description."""
    adapter_config = os.path.join(
        get_package_share_directory("external_vio_adapter"),
        "config",
        "planar_odometry_adapter.yaml",
    )

    openvins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ov_msckf"),
                "launch",
                "subscribe.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("launch_openvins")),
        launch_arguments={
            "namespace": LaunchConfiguration("openvins_namespace"),
            "ov_enable": LaunchConfiguration("launch_openvins"),
            "config": LaunchConfiguration("openvins_config"),
            "config_path": LaunchConfiguration("openvins_config_path"),
            "max_cameras": LaunchConfiguration("max_cameras"),
            "use_stereo": LaunchConfiguration("use_stereo"),
            "rviz_enable": LaunchConfiguration("rviz_enable"),
        }.items(),
    )

    kaist_images = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("external_vio_adapter"),
                "launch",
                "kaist_image_republish.launch.py",
            ])
        ),
        launch_arguments={
            "mono_image_topic": LaunchConfiguration("mono_image_topic"),
            "left_image_topic": LaunchConfiguration("left_image_topic"),
            "right_image_topic": LaunchConfiguration("right_image_topic"),
            "input_transport": LaunchConfiguration("input_transport"),
        }.items(),
    )

    openvins_with_kaist_remaps = GroupAction([
        SetRemap(src="/imu0", dst=LaunchConfiguration("imu_topic")),
        SetRemap(src="/cam0/image_raw", dst=LaunchConfiguration("camera0_topic")),
        SetRemap(src="/cam1/image_raw", dst="/kaist/cam1/image_raw"),
        # The KAIST OpenVINS calibration stores the original topic names in
        # its YAML file. Keep these remaps as a second protection for the
        # external node if its topic parameters are not overridden.
        SetRemap(src="/mavros/imu/data", dst=LaunchConfiguration("imu_topic")),
        SetRemap(
            src="/camera/infra1/image_rect_raw",
            dst=LaunchConfiguration("camera0_topic"),
        ),
        SetRemap(src="/camera/infra2/image_rect_raw", dst="/kaist/cam1/image_raw"),
        openvins_launch,
    ])

    adapter_parameters = [
        adapter_config,
        {
            "input_topic": LaunchConfiguration("openvins_odom_topic"),
            "output_topic": LaunchConfiguration("beluga_odom_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
        },
    ]
    adapter = Node(
        package="external_vio_adapter",
        executable="planar_odometry_adapter",
        name="planar_odometry_adapter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_base_frame_transform")),
        parameters=adapter_parameters,
    )
    adapter_without_tf = Node(
        package="external_vio_adapter",
        executable="planar_odometry_adapter",
        name="planar_odometry_adapter",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_base_frame_transform")),
        parameters=[
            adapter_config,
            {
                "input_topic": LaunchConfiguration("openvins_odom_topic"),
                "output_topic": LaunchConfiguration("beluga_odom_topic"),
                "base_frame": "",
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_openvins",
            default_value="true",
            description="Start the external OpenVINS node",
        ),
        DeclareLaunchArgument(
            "openvins_namespace",
            default_value="ov_msckf",
            description="Namespace used by OpenVINS",
        ),
        DeclareLaunchArgument(
            "openvins_config",
            default_value="euroc_mav",
            description="OpenVINS installed configuration name",
        ),
        DeclareLaunchArgument(
            "openvins_config_path",
            default_value="",
            description="Path to a calibrated OpenVINS estimator_config.yaml",
        ),
        DeclareLaunchArgument(
            "openvins_odom_topic",
            default_value="/ov_msckf/odomimu",
            description="OpenVINS nav_msgs/Odometry output topic",
        ),
        DeclareLaunchArgument(
            "beluga_odom_topic",
            default_value="/beluga/vio/odometry",
            description="Planar odometry topic consumed by Beluga",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Robot base frame used to transform the OpenVINS body frame",
        ),
        DeclareLaunchArgument(
            "use_base_frame_transform",
            default_value="true",
            description="Apply the body-to-base TF before planar projection",
        ),
        DeclareLaunchArgument("max_cameras", default_value="1"),
        DeclareLaunchArgument("use_stereo", default_value="false"),
        DeclareLaunchArgument("imu_topic", default_value="/mavros/imu/data"),
        DeclareLaunchArgument("camera0_topic", default_value="/kaist/cam0/image_raw"),
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
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        kaist_images,
        openvins_with_kaist_remaps,
        adapter,
        adapter_without_tf,
    ])
