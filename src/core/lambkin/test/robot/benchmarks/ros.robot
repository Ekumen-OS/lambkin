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
Documentation  Base ROS system benchmark implementation tests
Resource  lambkin/robot/resources/benchmarks/ros/base.resource

Resource  testing.resource

Library   yaml

Suite Setup     Setup basic ROS system benchmark suite
Suite Teardown  Teardown basic ROS system benchmark suite


*** Test Cases ***
ROS system benchmark functionality is correct
    Skip Unless Executable Exists  rosbag
    Run basic ROS system benchmark case once


*** Keywords ***
Basic ROS system benchmark suite
    Extends ROS system benchmark suite
    Extends testing benchmark suite

Basic ROS system benchmark case
    Extends ROS system benchmark case
    Uses data/basic.bag as input
    Uses data/basic.launch as rig

After basic ROS system benchmark case iteration
    File Should Exist  ${BENCHMARK.CASE.ITERATION.PATH}/output.bag
    ${result} =  Run Process  rosbag  info  -y
    ...  ${BENCHMARK.CASE.ITERATION.PATH}/output.bag
    Should Be Equal  ${result.rc}  ${0}
    ${info} =  yaml.safe_load  ${result.stdout}
    ${topics} =  Get From Dictionary  ${info}  topics
    ${num_topics} =  Get Length  ${topics}
    Should Be True  ${num_topics} >= ${3}
    Dictionary Should Contain Item  ${topics[0]}  topic  /clock
    Dictionary Should Contain Item  ${topics[1]}  topic  /input
    Dictionary Should Contain Item  ${topics[2]}  topic  /output
