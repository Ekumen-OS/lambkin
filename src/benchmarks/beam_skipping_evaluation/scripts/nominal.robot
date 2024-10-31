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
Documentation       Nav2 beam skipping feature evaluation.

Resource            lambkin/shepherd/robot/resources/all.resource

Suite Setup         Setup Nav2 Beam Skipping Evaluation benchmark suite
Suite Teardown      Teardown Nav2 Beam Skipping Evaluation benchmark suite
Test Template       Run Nav2 Beam Skipping Evaluation benchmark case for each ${dataset} ${basedir} ${odom_frame} ${map_frame} ${base_frame} ${scan_topic} ${iterations} ${initial_pose_x} ${initial_pose_y} ${initial_pose_yaw} ${robot_model_type}


*** Variables ***

@{DATASETS_DIR}             simulated_datasets
@{DIFF_DRIVE_SIM_BAGS}      simulated_hq_kobuki
@{OMNI_DRIVE_SIM_BAGS}      simulated_bookstore_robomaster

*** Test Cases ***        DATASET                         BASEDIR                ODOM_FRAME     MAP_FRAME  BASE_FRAME      SCAN_TOPIC     ITERATION      INITIAL_POSE_X  INITIAL_POSE_Y  INITIAL_POSE_YAW    ROBOT_MODEL_TYPE
Diff Drive Sim 60m        ${{DIFF_DRIVE_SIM_BAGS}}        ${{DATASETS_DIR}}      odom           map        base_link       /scan          1              0.0             2.0             0.0                 nav2_amcl::DifferentialMotionModel
Omni Drive Sim 30m        ${{OMNI_DRIVE_SIM_BAGS}}        ${{DATASETS_DIR}}      odom           map        base_link       /scan          1              3.2             9.0             0.7                 nav2_amcl::OmniMotionModel

*** Keywords ***
Nav2 Beam Skipping Evaluation benchmark suite
    Extends ROS 2 system benchmark suite
    Extends ROS 2 2D SLAM system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from nominal_report template in beam_skipping_evaluation ROS 2 package

Nav2 Beam Skipping Evaluation benchmark case
    Extends generic resource usage benchmark case
    Extends ROS 2 system benchmark case
    Extends ROS 2 2D SLAM system benchmark case
    # Setup benchmark inputs
    ${bagfile_path} =  Set Variable  ${EXECDIR}/${basedir}/bagfiles/${dataset}
    ${artifacts_path} =  Set Variable  ${EXECDIR}/${basedir}/artifacts/${dataset}
    Uses ${bagfile_path} at 1x as input to ROS 2 system
    ${package_share_path} =  Find ROS 2 Package  beam_skipping_evaluation  share=yes
    ${qos_override_path} =  Join Path  ${package_share_path}  config  qos_override.yml
    Configures QoS overrides from ${qos_override_path} for input to ROS 2 system
    # Setup benchmark rig
    Uses beam_skipping_evaluation.launch in beam_skipping_evaluation ROS package as rig
    Sets map_filename launch argument to ${artifacts_path}/map.yaml
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
    Uses timemory-timem to sample nav2_amcl_likelihood performance
    Uses timemory-timem to sample nav2_amcl_likelihood_prob performance
    Uses timemory-timem to sample nav2_amcl_likelihood_beam_skip performance
    # Setup tracking
    Tracks /nav2_amcl_likelihood/pose trajectories
    Tracks /nav2_amcl_likelihood_prob/pose trajectories
    Tracks /nav2_amcl_likelihood_beam_skip/pose trajectories
    # Setup benchmark analysis
    Uses ${artifacts_path}/groundtruth.tum as trajectory groundtruth
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}
    Uses ${iterations} iterations
