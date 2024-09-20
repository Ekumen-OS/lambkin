#!/usr/bin/env -S shepherd robot -f

# Copyright 2022 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


*** Settings ***
Documentation       Nominal Beluga AMCL vs Nav2 AMCL benchmark using 2D datasets.

Resource            lambkin/shepherd/robot/resources/all.resource

Suite Setup         Setup Beluga vs Nav2 benchmark suite
Suite Teardown      Teardown Beluga vs Nav2 benchmark suite
Test Template       Run Beluga vs Nav2 benchmark case for each ${dataset} ${basedir} ${laser_model} ${odom_frame} ${map_frame} ${base_frame} ${scan_topic} ${iterations} ${initial_pose_x} ${initial_pose_y} ${initial_pose_yaw} ${robot_model_type}


*** Variables ***

@{LASER_MODELS}             likelihood_field
...                         beam

${TORWIC_SLAM_DIR}          torwic_slam_dataset_ros2_localization_only
@{TORWIC_SLAM_BAGS}         2022-06-15_aisle_ccw_run_1
# ...                         2022-06-15_aisle_ccw_run_2
# ...                         2022-06-15_aisle_cw_run_1
# ...                         2022-06-15_aisle_cw_run_2
# ...                         2022-06-15_hallway_full_cw_parts_1_and_2
# ...                         2022-06-15_hallway_full_ccw_parts_1_and_2
# ...                         2022-06-15_hallway_straight_ccw_parts_1_and_2
# ...                         2022-06-23_aisle_ccw_run_1
# ...                         2022-06-23_aisle_ccw_run_2
# ...                         2022-06-23_aisle_cw_run_1
# ...                         2022-06-23_aisle_cw_run_2
# ...                         2022-06-23_hallway_full_cw_parts_1_and_2
# ...                         2022-06-23_hallway_straight_ccw_parts_1_and_2
# ...                         2022-06-23_hallway_straight_cw_parts_1_and_2
# ...                         2022-10-12_aisle_ccw
# ...                         2022-10-12_aisle_cw
# ...                         2022-10-12_hallway_full_cw_run1
# ...                         2022-10-12_hallway_full_cw_run2
# ...                         2022-10-12_hallway_straight_ccw
# ...                         2022-10-12_hallway_straight_cw


*** Test Cases ***        DATASET                         BASEDIR                   LASER_MODEL           ODOM_FRAME     MAP_FRAME  BASE_FRAME      SCAN_TOPIC     ITERATION      INITIAL_POSE_X  INITIAL_POSE_Y  INITIAL_POSE_YAW    ROBOT_MODEL_TYPE 
TorWIC SLAM               ${{TORWIC_SLAM_BAGS}}           ${{TORWIC_SLAM_DIR}}      ${{LASER_MODELS}}     odom           map        base_link       /front/scan    1              0.0             0.0             0.0                nav2_amcl::DifferentialMotionModel

*** Keywords ***
Beluga vs Nav2 benchmark suite
    Extends ROS 2 system benchmark suite
    Extends ROS 2 2D SLAM system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from nominal_report template in beluga_vs_nav2_multi_dataset ROS 2 package

Beluga vs Nav2 benchmark case
    Extends generic resource usage benchmark case
    Extends ROS 2 system benchmark case
    Extends ROS 2 2D SLAM system benchmark case
    # Setup benchmark inputs
    ${bagfile_path} =  Set Variable  ${EXECDIR}/${basedir}/bagfiles/${dataset}
    ${artifacts_path} =  Set Variable  ${EXECDIR}/${basedir}/artifacts/${dataset}
    Uses ${bagfile_path} at 1x as input to ROS 2 system
    ${package_share_path} =  Find ROS 2 Package  beluga_vs_nav2_multi_dataset  share=yes
    ${qos_override_path} =  Join Path  ${package_share_path}  config  qos_override.yml
    Configures QoS overrides from ${qos_override_path} for input to ROS 2 system
    # Setup benchmark rig
    Uses beluga_vs_nav2_multi_dataset.launch in beluga_vs_nav2_multi_dataset ROS package as rig
    Sets map_filename launch argument to ${artifacts_path}/map.yaml
    Sets laser_model_type launch argument to ${laser_model}
    Sets global_frame_id launch argument to ${map_frame}
    Sets odom_frame_id launch argument to ${odom_frame}
    Sets base_frame_id launch argument to ${base_frame}
    Sets scan_topic launch argument to ${scan_topic}
    Sets initial_pose_x launch argument to ${initial_pose_x}
    Sets initial_pose_y launch argument to ${initial_pose_y}
    Sets initial_pose_yaw launch argument to ${initial_pose_yaw}
    Sets robot_model_type launch argument to ${robot_model_type}
    Sets use_sim_time launch argument to true
    # Setup benchmark profiling
    Uses timemory-timem to sample beluga_amcl performance
    Uses timemory-timem to sample nav2_amcl performance
    # Setup benchmark analysis
    Tracks /nav2_amcl/pose /beluga_amcl/pose trajectories
    Uses ${artifacts_path}/groundtruth.tum as trajectory groundtruth
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}
    Uses ${iterations} iterations
