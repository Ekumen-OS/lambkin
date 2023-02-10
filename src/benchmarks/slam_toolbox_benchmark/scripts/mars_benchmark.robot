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

Test Template     Benchmark SLAM Toolbox 2D SLAM
Suite Setup       Lambkin Setup
Suite Teardown    Benchmark Teardown

*** Test Cases ***       DATASET
Indoor loop              MARS_Loop_1.bag MARS_Loop_2.bag MARS_Loop_3.bag

*** Keywords ***
Benchmark SLAM Toolbox 2D SLAM
    [Arguments]  ${dataset}
    Register Parameters  dataset=${dataset}
    Use /tf /vertical_velodyne/velodyne_points data in ${dataset} at 10x as input
    Track /tf:odom.base_link /tf:map.base_link trajectories
    And save the resulting map
    Use mars_benchmark.launch in slam_toolbox_benchmark package to launch
    Use a sampling rate of 20 Hz to track computational performance
    Benchmark SLAM Toolbox for 10 iterations

Benchmark Teardown
    Lambkin Teardown
    Run Keyword If All Tests Passed
    ...  Generate report using mars_report in slam_toolbox_benchmark package

