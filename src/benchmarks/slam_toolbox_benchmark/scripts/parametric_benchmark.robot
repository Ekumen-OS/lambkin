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
Library           lambkin.TestMacros

Test Template     Benchmark SLAM Toolbox 2D SLAM
Suite Setup       Lambkin Setup
Suite Teardown    Benchmark Teardown

*** Test Cases ***         DATASET                                      RESOLUTION                  SEARCH_RESOLUTION
Freiburg2 Pioneer 360      rgbd_dataset_freiburg2_pioneer_360.bag       LINSPACE(0.025, 0.125, 5)   SEQUENCE(0.025, 0.050)
Freiburg2 Pioneer SLAM 1   rgbd_dataset_freiburg2_pioneer_slam.bag      LINSPACE(0.025, 0.125, 5)   SEQUENCE(0.025, 0.050)
Freiburg2 Pioneer SLAM 2   rgbd_dataset_freiburg2_pioneer_slam2.bag     LINSPACE(0.025, 0.125, 5)   SEQUENCE(0.025, 0.050)
Freiburg2 Pioneer SLAM 3   rgbd_dataset_freiburg2_pioneer_slam3.bag     LINSPACE(0.025, 0.125, 5)   SEQUENCE(0.025, 0.050)

*** Keywords ***
Benchmark SLAM Toolbox 2D SLAM
    [Arguments]  ${dataset}  ${resolution}  ${search_resolution}
    Register parameters
    ...  dataset=${dataset}
    ...  resolution=${resolution}
    ...  search_resolution=${search_resolution}
    Use /tf /scan data in ${dataset} as input
    Track /tf:world.kinect /tf:map.base_link trajectories
    And save the resulting map
    Use parametric_benchmark.launch in slam_toolbox_benchmark package to launch
    Use a sampling rate of 20 Hz to track computational performance
    Benchmark SLAM Toolbox for 5 iterations

Benchmark Teardown
    Lambkin Teardown
    Run Keyword If All Tests Passed
    ...  Generate report using parametric_report in slam_toolbox_benchmark package
