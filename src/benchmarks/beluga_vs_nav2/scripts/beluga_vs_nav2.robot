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
Documentation       Multi solution benchmark for 2D localization datasets.

Resource            misc.resource

Suite Setup         Setup Beluga vs Nav2 benchmark suite
Suite Teardown      Teardown Beluga vs Nav2 benchmark suite
Test Template       Run Beluga vs Nav2 benchmark case for each ${dataset} ${launch_file} ${reference} ${scan_topic} ${initial_pose_x} ${initial_pose_y} ${initial_pose_yaw}


*** Variables ***
${BASE_DATASETS_PATH}=                   ${EXECDIR}/beluga-datasets
${BENCHMARK.OUTPUT.ROS.BAG}     output_bag


*** Test Cases ***    DATASET    LAUNCH_FILE    REFERENCE    SCAN_TOPIC    INITIAL_POSE_X    INITIAL_POSE_Y    INITIAL_POSE_YAW
Hallway Localization Nav2AMCL    hallway_localization    nav2_amcl.launch    ${True}    scan_front    74.15    -8.22    2.25
Hallway Localization Beluga    hallway_localization    beluga_amcl.launch    ${False}    scan_front    74.15    -8.22    2.25
Hallway Return Nav2AMCL    hallway_return    nav2_amcl.launch    ${True}    scan_front    0.0    0.0    -0.17
Hallway Return Beluga    hallway_return    beluga_amcl.launch    ${False}    scan_front    0.0    0.0    -0.17


*** Keywords ***
Beluga vs Nav2 benchmark suite
    Extends ROS 2 system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from beluga_vs_nav2_report template in beluga_vs_nav2 ROS 2 package

Beluga vs Nav2 benchmark case
    Extends generic resource usage benchmark case
    Extends ROS 2 system benchmark case
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}
    Uses /amcl_pose as trajectory groundtruth
    Tracks /amcl_pose trajectory
    Set Test Variable    $BENCHMARK.CASE.ITERATION.MAP_PATH    ${BASE_DATASETS_PATH}/${dataset}/${dataset}.yaml
    Set Test Variable    $IS_REFERENCE    ${reference}
    Set Test Variable    $DATASET    ${dataset}
    Uses timemory-timem to sample amcl performance
    Uses ${BASE_DATASETS_PATH}/${dataset}/ROS2 at 3x as input to ROS 2 system
    Uses ${launch_file} in multi_solution_benchmarks ROS package as rig
    Sets initial_pose_x launch argument to ${initial_pose_x}
    Sets initial_pose_y launch argument to ${initial_pose_y}
    Sets initial_pose_yaw launch argument to ${initial_pose_yaw}
    Sets yaml_filename launch argument to ${BENCHMARK.CASE.ITERATION.MAP_PATH}
    Sets scan_topic launch argument to ${scan_topic}
    Log To Console    Running case ${dataset} with launch file ${launch_file}
    Uses 20 iterations
