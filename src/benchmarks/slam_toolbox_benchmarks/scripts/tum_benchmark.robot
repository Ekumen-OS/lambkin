#!/usr/bin/env -S shepherd robot -f
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
Documentation   SLAM Toolbox 2D SLAM benchmark using TUM datasets
Resource        lambkin/shepherd/robot/resources/all.resource

Suite Setup     Setup SLAM Toolbox 2D SLAM benchmark suite
Suite Teardown  Teardown SLAM Toolbox 2D SLAM benchmark suite
Test Template   Run SLAM Toolbox 2D SLAM benchmark case for each ${dataset}


*** Test Cases ***        DATASET
Freiburg2 Pioneer 360     rgbd_dataset_freiburg2_pioneer_360.bag
Freiburg2 Pioneer SLAM 1  rgbd_dataset_freiburg2_pioneer_slam.bag
Freiburg2 Pioneer SLAM 2  rgbd_dataset_freiburg2_pioneer_slam2.bag
Freiburg2 Pioneer SLAM 3  rgbd_dataset_freiburg2_pioneer_slam3.bag


*** Keywords ***
SLAM Toolbox 2D SLAM benchmark suite
    Extends ROS 2D SLAM system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from tum_report template in slam_toolbox_benchmarks ROS package

SLAM Toolbox 2D SLAM benchmark case
    Extends ROS 2D SLAM system benchmark case
    Extends generic resource usage benchmark case
    Uses /tf /scan data in ${dataset} at 10x as input
    Uses tum_benchmark.launch in slam_toolbox_benchmarks ROS package as rig
    Uses timemory-timem to sample sync_slam_toolbox_node performance
    Tracks /tf:odom.base_link /tf:map.base_link trajectories
    Uses /tf:world.kinect as trajectory groundtruth
    Uses 10 iterations
