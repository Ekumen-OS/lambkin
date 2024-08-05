#!/usr/bin/env -S shepherd robot --skip-all -f

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

@{MAGAZINO_DIR}             magazzino_ros2_localization_only
@{MAGAZINO_BAGS}            hallway_localization
...                         hallway_return

@{OPENLORIS_DIR}            openloris_office_ros2_localization_only
@{OPENLORIS_OFFICE_BAGS}    office1-1
...                         office1-2
...                         office1-3
...                         office1-4
...                         office1-5
...                         office1-6
...                         office1-7

${TORWIC_MAPPING_DIR}       torwic_mapping_dataset_ros2_localization_only
@{TORWIC_MAPPING_BAGS}      baseline
...                         1-1_top_row_shift
...                         1-2_larger_shift
...                         1-3_longitudinal_shift
...                         1-4_all_boxes_shift
...                         1-5_all_boxes_rotate
...                         1-6_inner_shrink
...                         1-7_all_shrink
...                         1-8_alternating_in_and_out
...                         1-9_stepping_outward
...                         2-1_remove_top_row
...                         2-2_remove_corners
...                         2-3_remove_all_boxes
...                         2-4_remove_fences
...                         3-1_top_row_shift_with_fences
...                         3-2_top_row_shift_with_sides
...                         3-3_fences_move_outward
...                         4-1_cover_the_fences_with_boxes

${TORWIC_SLAM_DIR}          torwic_slam_dataset_ros2_localization_only
@{TORWIC_SLAM_BAGS}         2022-06-15_aisle_ccw_run_1
...                         2022-06-15_aisle_ccw_run_2
...                         2022-06-15_aisle_cw_run_1
...                         2022-06-15_aisle_cw_run_2
...                         2022-06-15_hallway_full_cw_parts_1_and_2
...                         2022-06-15_hallway_full_ccw_parts_1_and_2
...                         2022-06-15_hallway_straight_ccw_parts_1_and_2
...                         2022-06-23_aisle_ccw_run_1
...                         2022-06-23_aisle_ccw_run_2
...                         2022-06-23_aisle_cw_run_1
...                         2022-06-23_aisle_cw_run_2
...                         2022-06-23_hallway_full_cw_parts_1_and_2
...                         2022-06-23_hallway_straight_ccw_parts_1_and_2
...                         2022-06-23_hallway_straight_cw_parts_1_and_2
...                         2022-10-12_aisle_ccw
...                         2022-10-12_aisle_cw
...                         2022-10-12_hallway_full_cw_run1
...                         2022-10-12_hallway_full_cw_run2
...                         2022-10-12_hallway_straight_ccw
...                         2022-10-12_hallway_straight_cw

${WILLOW_DSET_DIR}           willow_garage_dataset_ros2_localization_only
# # 2011-08-03-16-16-43 is not in the list because it's missing the scan topic
@{WILLOW_DSET_BAGS}          2011-08-03-20-03-22
...                          2011-08-04-12-16-23
...                          2011-08-04-14-27-40
...                          2011-08-04-23-46-28
...                          2011-08-05-09-27-53
...                          2011-08-05-12-58-41
...                          2011-08-05-23-19-43
...                          2011-08-08-09-48-17
...                          2011-08-08-14-26-55
...                          2011-08-08-23-29-37
# we are not testing the full dataset because it's too big and because all bags
# are basically the same environment with different starting positions and trajectories
#...                          2011-08-09-08-49-52
#...                          2011-08-09-14-32-35
#...                          2011-08-09-22-31-30
#...                          2011-08-10-09-36-26
#...                          2011-08-10-14-48-32
#...                          2011-08-11-01-31-15
#...                          2011-08-11-08-36-01
#...                          2011-08-11-14-27-41
#...                          2011-08-11-22-03-37
#...                          2011-08-12-09-06-48
#...                          2011-08-12-16-39-48
#...                          2011-08-12-22-46-34
#...                          2011-08-15-17-22-26
#...                          2011-08-15-21-26-26
#...                          2011-08-16-09-20-08
#...                          2011-08-16-18-40-52
#...                          2011-08-16-20-59-00
#...                          2011-08-17-15-51-51
#...                          2011-08-17-21-17-05
#...                          2011-08-18-20-33-16
#...                          2011-08-18-20-52-30
#...                          2011-08-19-10-12-20
#...                          2011-08-19-14-17-55
#...                          2011-08-19-21-35-17
#...                          2011-08-22-10-02-27
#...                          2011-08-22-14-53-33
#...                          2011-08-23-01-11-53
#...                          2011-08-23-09-21-17
#...                          2011-08-24-09-52-14
#...                          2011-08-24-15-01-39
#...                          2011-08-24-19-47-10
#...                          2011-08-25-09-31-05
#...                          2011-08-25-20-14-56
#...                          2011-08-25-20-38-39
#...                          2011-08-26-09-58-19
#...                          2011-08-29-15-48-07
#...                          2011-08-29-21-14-07
#...                          2011-08-30-08-55-28
#...                          2011-08-30-20-49-42
#...                          2011-08-30-21-17-56
#...                          2011-08-31-20-29-19
#...                          2011-08-31-20-44-19
#...                          2011-09-01-08-21-33
#...                          2011-09-02-09-20-25
#...                          2011-09-06-09-04-41
#...                          2011-09-06-13-20-36
#...                          2011-09-08-13-14-39
#...                          2011-09-09-13-22-57
#...                          2011-09-11-07-34-22
#...                          2011-09-11-09-43-46
#...                          2011-09-12-14-18-56
#...                          2011-09-12-14-47-01
#...                          2011-09-13-10-23-31
#...                          2011-09-13-13-44-21
#...                          2011-09-14-10-19-20
#...                          2011-09-15-08-32-46

@{LONG_DURATION_DIR}          long_duration_bags_ros2_localization_only
@{DIFF_DRIVE_SIM_BAGS}         simulated_hq_kobuki_25hs
@{OMNI_DRIVE_SIM_BAGS}         simulated_bookstore_robomaster_24hs

*** Test Cases ***        DATASET                         BASEDIR                   LASER_MODEL           ODOM_FRAME     MAP_FRAME  BASE_FRAME      SCAN_TOPIC     ITERATION      INITIAL_POSE_X  INITIAL_POSE_Y  INITIAL_POSE_YAW    ROBOT_MODEL_TYPE 
Magazino Datasets         ${{MAGAZINO_BAGS}}              ${{MAGAZINO_DIR}}         ${{LASER_MODELS}}     odom           map        base_footprint  /scan_front    5              0.0             0.0             0.0                nav2_amcl::DifferentialMotionModel
Openloris Office          ${{OPENLORIS_OFFICE_BAGS}}      ${{OPENLORIS_DIR}}        ${{LASER_MODELS}}     base_odom      map        base_link       /scan          5              0.0             0.0             0.0                nav2_amcl::DifferentialMotionModel
TorWIC Mapping            ${{TORWIC_MAPPING_BAGS}}        ${{TORWIC_MAPPING_DIR}}   ${{LASER_MODELS}}     odom           map        base_link       /front/scan    5              0.0             0.0             0.0                nav2_amcl::DifferentialMotionModel
TorWIC SLAM               ${{TORWIC_SLAM_BAGS}}           ${{TORWIC_SLAM_DIR}}      ${{LASER_MODELS}}     odom           map        base_link       /front/scan    5              0.0             0.0             0.0                nav2_amcl::DifferentialMotionModel
Willow Garage             ${{WILLOW_DSET_BAGS}}           ${{WILLOW_DSET_DIR}}      ${{LASER_MODELS}}     odom_combined  map        base_footprint  /base_scan     5              0.0             0.0             0.0                nav2_amcl::DifferentialMotionModel
Diff Drive Sim 24hs       ${{DIFF_DRIVE_SIM_BAGS}}        ${{LONG_DURATION_DIR}}    ${{LASER_MODELS}}     odom           map        base_link       /scan          1              0.0             2.0             0.0                nav2_amcl::DifferentialMotionModel
Omni Drive Sim 24hs       ${{OMNI_DRIVE_SIM_BAGS}}        ${{LONG_DURATION_DIR}}    ${{LASER_MODELS}}     odom           map        base_link       /scan          1              3.2             9.0             0.7                 nav2_amcl::OmniMotionModel

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
