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
Suite Teardown    Run Keyword If All Tests Passed
...               Generate report using rawseeds_report.rst in cartographer_ros_benchmark package

*** Test Cases ***                    DATASET
Indoors with static illumination      Bicocca_2009-02-25b_Static_Lamps.bag

*** Keywords ***
Benchmark Cartographer ROS 2D SLAM
    [Arguments]  ${dataset}
    Use 2 minutes of /tf /odom /scan/front /scan/rear data in ${dataset} as input
    Track /tf:map.base_link /tf:odom.base_link trajectories
    And save the resulting map
    Use rawseeds_benchmark.launch in cartographer_ros_benchmark package to launch
    Use a sampling rate of 20 Hz to track computational performance
    Benchmark Cartographer ROS for 10 iterations
