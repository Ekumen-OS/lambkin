#!/usr/bin/env -S lambkin robot -f
# Copyright 2023 Ekumen, Inc.
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
Test Template   Run SLAM Toolbox 2D SLAM benchmark case for each ${dataset}


*** Test Cases ***        DATASET

Freiburg3 Pioneer SLAM 3  rgbd_dataset_freiburg3_long_office_household.bag


*** Keywords ***
SLAM Toolbox 2D SLAM benchmark suite
    Extends ROS 2D SLAM system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from tum_report template in rtabmap_benchmark ROS package

SLAM Toolbox 2D SLAM benchmark case
    Extends ROS 2D SLAM system benchmark case
    Extends generic resource usage benchmark case
    Uses /tf /camera/depth/camera_info /camera/depth/image /camera/rgb/camera_info /camera/rgb/image_color data in ${dataset} at 1x as input
    Uses tum_benchmark.launch in rtabmap_benchmark ROS package as rig
    Uses timemory-timem to sample rtabmap performance
    Tracks /tf:odom.kinect /tf:map.kinect trajectories
    Uses /tf:world.kinect_gt as trajectory groundtruth
    Uses 1 iterations
