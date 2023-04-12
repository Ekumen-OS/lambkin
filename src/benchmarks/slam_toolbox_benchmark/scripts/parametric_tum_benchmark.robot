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
Documentation   SLAM Toolbox 2D SLAM benchmark using TUM datasets
Resource        lambkin/robot/resources/all.resource

Suite Setup     Setup SLAM Toolbox 2D SLAM benchmark suite
Suite Teardown  Teardown SLAM Toolbox 2D SLAM benchmark suite
Test Template   Run SLAM Toolbox 2D SLAM benchmark case for each ${dataset} ${map_resolution} ${search_resolution}


*** Test Cases ***        DATASET                                   MAP RESOLUTION                     SEARCH RESOLUTION
Freiburg2 Pioneer 360     rgbd_dataset_freiburg2_pioneer_360.bag    ${{np.linspace(0.025, 0.125, 5)}}  ${{[0.025, 0.050]}}
Freiburg2 Pioneer SLAM 1  rgbd_dataset_freiburg2_pioneer_slam.bag   ${{np.linspace(0.025, 0.125, 5)}}  ${{[0.025, 0.050]}}
Freiburg2 Pioneer SLAM 2  rgbd_dataset_freiburg2_pioneer_slam2.bag  ${{np.linspace(0.025, 0.125, 5)}}  ${{[0.025, 0.050]}}
Freiburg2 Pioneer SLAM 3  rgbd_dataset_freiburg2_pioneer_slam3.bag  ${{np.linspace(0.025, 0.125, 5)}}  ${{[0.025, 0.050]}}


*** Keywords ***
SLAM Toolbox 2D SLAM benchmark suite
    Extends ROS 2D SLAM system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from parametric_report template in slam_toolbox_benchmark ROS package

SLAM Toolbox 2D SLAM benchmark case
    Extends ROS 2D SLAM system benchmark case
    Extends generic resource usage benchmark case
    Uses /tf /scan data in ${dataset} at 10x as input
    Uses parametric_benchmark.launch in slam_toolbox_benchmark ROS package as rig
    Sets map_resolution launch argument to ${map_resolution}
    Sets search_resolution launch argument to ${search_resolution}
    Uses timemory-timem to sample sync_slam_toolbox_node performance
    Tracks /tf:map.base_link /tf:odom.base_link trajectories
    Uses /tf:world.kinect as trajectory groundtruth
    Uses 10 iterations
