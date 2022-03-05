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
Hallway Return           hallway_return.bag

*** Keywords ***
Benchmark SLAM Toolbox 2D SLAM
    [Arguments]  ${dataset}
    Register Parameters  dataset=${dataset}
    Use 2 minutes of all data in ${dataset} as input
    Track /tf:odom.base_link /tf:map.base_link trajectories
    And save the resulting map
    Use magazino_benchmark.launch in slam_toolbox_benchmark package to launch
    Use a sampling rate of 20 Hz to track computational performance
    Benchmark SLAM Toolbox for 10 iterations

Benchmark Teardown
    Lambkin Teardown
    Run Keyword If All Tests Passed
    ...  Generate report using magazino_report in slam_toolbox_benchmark package
