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
Resource          lambkin.resource

Test Template     Benchmark Cartographer ROS 2D SLAM
Suite Setup       Lambkin Setup
Suite Teardown    Benchmark Teardown

*** Test Cases ***         DATASET
Freiburg2 Pioneer 360      rgbd_dataset_freiburg2_pioneer_360.bag
Freiburg2 Pioneer SLAM 1   rgbd_dataset_freiburg2_pioneer_slam.bag
Freiburg2 Pioneer SLAM 2   rgbd_dataset_freiburg2_pioneer_slam2.bag
Freiburg2 Pioneer SLAM 3   rgbd_dataset_freiburg2_pioneer_slam3.bag

*** Keywords ***
Benchmark Cartographer ROS 2D SLAM
    [Arguments]  ${dataset}
    Register Parameters  dataset=${dataset}
    Use /tf /scan /pose data in ${dataset} as input
    Track /tf:world.kinect /tf:map.base_link trajectories
    And save the resulting map
    Use tum_benchmark.launch in cartographer_ros_benchmark package to launch
    Use a sampling rate of 20 Hz to track computational performance
    Benchmark Cartographer ROS for 10 iterations

Benchmark Teardown
    Lambkin Teardown
    Run Keyword If All Tests Passed
    ...  Generate report using reports/tum_report in cartographer_ros_benchmark package
