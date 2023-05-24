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
Documentation
...  Common template descriptions for ROS and ROS 2 system benchmarks.
...
...  These descriptions provide data post-processing
...  for performance assessment of ROS and ROS 2 systems.

Library   Collections
Library   String


*** Variables ***
${BENCHMARK.INPUT.ROS.RATE}            ${1}
${BENCHMARK.INPUT.ROS.TOPICS}          ${null}
${BENCHMARK.INPUT.ROS.DURATION}        ${null}
${BENCHMARK.OUTPUT.ROS.BAG}            output.bag
${BENCHMARK.RIG.ROS.LAUNCH.PACKAGE}    ${null}
@{BENCHMARK.RIG.ROS.LAUNCH.ARGUMENTS}  @{EMPTY}
${BENCHMARK.RIG.ROS.SIMULATED}         ${False}


*** Keywords ***
Uses ${launch_file} as rig
    [Documentation]
    ...  Specifies a launch file for the benchmark case rig.
    ...  Paths are resolved relative to the benchmark suite.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.RIG.ROS.LAUNCH.FILE
    ...  ROS launch file already set to ${BENCHMARK.RIG.ROS.LAUNCH_FILE}
    ${launch_file} =  Resolve File Path or Fail  ${launch_file}
    Set Test Variable  $BENCHMARK.RIG.ROS.LAUNCH.FILE  ${launch_file}

Uses ${launch_file} in ${package_name} ROS package as rig
    [Documentation]
    ...  Specifies a launch file for the benchmark case rig.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.RIG.ROS.LAUNCH.FILE
    ...  ROS launch file already set to ${BENCHMARK.RIG.ROS.LAUNCH_FILE}
    Set Test Variable  $BENCHMARK.RIG.ROS.LAUNCH.PACKAGE  ${package_name}
    Set Test Variable  $BENCHMARK.RIG.ROS.LAUNCH.FILE  ${launch_file}

Sets ${name} launch argument to ${value}
    [Documentation]
    ...  Specifies an argument for the benchmark case rig launch file.
    [Tags]  declarative
    ${args} =  Set Variable  ${BENCHMARK.RIG.ROS.LAUNCH.ARGUMENTS}
    Append To List  ${args}  ${name}:=${value}
    Set Test Variable  $BENCHMARK.RIG.ROS.LAUNCH.ARGUMENTS  ${args}
