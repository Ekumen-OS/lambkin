#!/usr/bin/env -S lambkin robot -f
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

*** Settings ***
Documentation   SLAM Toolbox 2D SLAM benchmark using Rawseeds datasets
Resource        lambkin/robot/resources/all.resource

Suite Setup     Setup Cartographer ROS 2D SLAM benchmark suite
Suite Teardown  Teardown Cartographer ROS 2D SLAM benchmark suite
Test Template   Run Cartographer ROS 2D SLAM benchmark case for each ${dataset}


*** Test Cases ***                    DATASET
Indoors with static illumination      Bicocca_2009-02-25b_Static_Lamps.bag


*** Keywords ***
Cartographer ROS 2D SLAM benchmark suite
    Extends ROS 2D SLAM system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from rawseeds_report template in cartographer_ros_benchmark ROS package

Cartographer ROS 2D SLAM benchmark case
    Extends ROS 2D SLAM system benchmark case
    Extends generic resource usage benchmark case
    Uses 2 minutes of ${dataset} at 10x as input
    Uses rawseeds_benchmark.launch in cartographer_ros_benchmark ROS package as rig
    Uses timemory-timem to sample cartographer_node performance
    Tracks /tf:odom.base_link /tf:map.base_link trajectories
    Uses /gt as trajectory groundtruth
    Uses 10 iterations
