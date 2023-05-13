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
Documentation  Base ROS 2 system benchmark implementation tests
Resource  lambkin/shepherd/robot/resources/benchmarks/ros2/base.resource

Resource  testing.resource

Suite Setup     Setup basic ROS 2 system benchmark suite
Suite Teardown  Teardown basic ROS 2 system benchmark suite


*** Test Cases ***
ROS 2 system benchmark functionality is correct
    Skip Unless Executable Exists  ros2
    Run basic ROS 2 system benchmark case once


*** Keywords ***
Basic ROS 2 system benchmark suite
    Extends ROS 2 system benchmark suite
    Extends testing benchmark suite

Basic ROS 2 system benchmark case
    Extends ROS 2 system benchmark case
    Uses data/ros2/basic as input
    Uses data/ros2/basic.launch as rig

After basic ROS 2 system benchmark case iteration
    Directory Should Exist  ${BENCHMARK.CASE.ITERATION.PATH}/${BENCHMARK.OUTPUT.ROS.BAG}
    ${result} =  Run Process  ros2  bag  info
    ...  ${BENCHMARK.CASE.ITERATION.PATH}/${BENCHMARK.OUTPUT.ROS.BAG}
    Should Be Equal  ${result.rc}  ${0}
    Should Contain  ${result.stdout}  Topic: /clock
    Should Contain  ${result.stdout}  Topic: /tf
    Should Contain  ${result.stdout}  Topic: /output
