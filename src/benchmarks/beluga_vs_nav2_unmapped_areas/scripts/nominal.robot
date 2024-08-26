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
Test Template       Run Beluga vs Nav2 benchmark case for each ${dataset} ${basedir} ${map} ${laser_model} ${odom_frame} ${map_frame} ${base_frame} ${scan_topic} ${iterations} ${initial_pose_x} ${initial_pose_y} ${initial_pose_yaw} ${robot_model_type}


*** Variables ***

@{LASER_MODELS}        beam  likelihood_field

@{TEST_DATA_DIR}       bookstore_visit

@{TEST_DATA_BAGS}        bookstore_visit

@{BASELINE_MAPS}       map_baseline
@{PROBLEM_MAPS}        map_with_problems

*** Test Cases ***                       DATASET                BASEDIR                MAP                  LASER_MODEL           ODOM_FRAME     MAP_FRAME  BASE_FRAME      SCAN_TOPIC     ITERATION      INITIAL_POSE_X  INITIAL_POSE_Y  INITIAL_POSE_YAW    ROBOT_MODEL_TYPE
Bookstore visit Baseline                 ${{TEST_DATA_BAGS}}    ${{TEST_DATA_DIR}}     map_baseline         ${{LASER_MODELS}}     odom           map        base_link       /scan           5              3.2             9.0             0.7                 nav2_amcl::OmniMotionModel
Bookstore visit With Bad Start           ${{TEST_DATA_BAGS}}    ${{TEST_DATA_DIR}}     bad_patches_start    ${{LASER_MODELS}}     odom           map        base_link       /scan           5              3.2             9.0             0.7                 nav2_amcl::OmniMotionModel
Bookstore visit With Bad Patches         ${{TEST_DATA_BAGS}}    ${{TEST_DATA_DIR}}     bad_patches_along    ${{LASER_MODELS}}     odom           map        base_link       /scan           5              3.2             9.0             0.7                 nav2_amcl::OmniMotionModel

*** Keywords ***
Beluga vs Nav2 benchmark suite
    Extends ROS 2 system benchmark suite
    Extends ROS 2 2D SLAM system benchmark suite
    Generates latexpdf report from nominal_report template in beluga_vs_nav2_unmapped_areas ROS 2 package

Beluga vs Nav2 benchmark case
    Extends ROS 2 system benchmark case
    Extends ROS 2 2D SLAM system benchmark case
    # Setup benchmark inputs
    ${bagfile_path} =  Set Variable  ${EXECDIR}/${basedir}/bagfiles/${dataset}
    ${artifacts_path} =  Set Variable  ${EXECDIR}/${basedir}
    Uses ${bagfile_path} at 5x as input to ROS 2 system
    ${package_share_path} =  Find ROS 2 Package  beluga_vs_nav2_unmapped_areas  share=yes
    ${qos_override_path} =  Join Path  ${package_share_path}  config  qos_override.yml
    Configures QoS overrides from ${qos_override_path} for input to ROS 2 system
    # Setup benchmark rig
    Uses beluga_vs_nav2_unmapped_areas.launch in beluga_vs_nav2_unmapped_areas ROS package as rig
    Sets map_filename launch argument to ${artifacts_path}/maps/${map}/map.yaml
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
    # Setup benchmark analysis
    Tracks /nav2_amcl/pose /beluga_amcl/pose trajectories
    Uses ${artifacts_path}/groundtruth.tum as trajectory groundtruth
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}
    Uses ${iterations} iterations
