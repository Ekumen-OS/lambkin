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
...  Template descriptions for ROS 2 system benchmarks.
...
...  These descriptions instrument benchmark rigs for
...  performance assessment of ROS 2 based systems.

Resource  lambkin/shepherd/robot/resources/library/filesystem.resource
Resource  lambkin/shepherd/robot/resources/library/ros2.resource
Resource  lambkin/shepherd/robot/resources/benchmarks/base.resource

Library   Collections
Library   DateTime
Library   OperatingSystem
Library   String


*** Variables ***
${BENCHMARK.INPUT.ROS.RATE}            ${1}
${BENCHMARK.INPUT.ROS.TOPICS}          ${null}
${BENCHMARK.INPUT.ROS.DURATION}        ${null}
${BENCHMARK.OUTPUT.ROS.BAG}            output
${BENCHMARK.RIG.ROS.LAUNCH.PACKAGE}    ${null}
@{BENCHMARK.RIG.ROS.LAUNCH.ARGUMENTS}  @{EMPTY}
${BENCHMARK.RIG.ROS.SIMULATED}         ${False}


*** Keywords ***
ROS 2 system benchmark suite
    [Documentation]
    ...  Generic ROS 2 system benchmark suite description.
    [Tags]  descriptive
    Extends base benchmark suite

ROS 2 system benchmark case
    [Documentation]
    ...  Generic ROS 2 system benchmark case description.
    ...
    ...  This description uses a single launch file for a benchmark case rig.
    ...  Input data is fed via ROS 2 bag playback, output data is collected via
    ...  ROS 2 bag recording.
    [Tags]  descriptive
    Extends base benchmark case

Uses ${bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. The path to ROS bag will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    Uses ${bag} at 1x as input

Uses ${duration} of ${bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    Uses ${duration} of ${bag} at 1x as input

Uses ${topics:/.+} data in ${bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. The path to ROS bag will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    Uses ${topics} data in ${bag} at 1x as input

Uses ${bag} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. The path to ROS bag will be resolved
    ...  relative to the working directory and the benchmark suite
    ...  parent directory.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.INPUT.ROS.BAGS
    ...  ROS bags already set to ${BENCHMARK.INPUT.ROS.BAGS}
    ${paths} =  Create List
    ${path} =  Resolve File Path or Fail  ${bag}
    Append To List  ${paths}  ${path}
    Set Test Variable  $BENCHMARK.INPUT.ROS.BAGS  ${paths}
    ${rate} =  Convert To Number  ${rate}
    Set Test Variable  $BENCHMARK.INPUT.ROS.RATE  ${rate}

Uses ${topics:/.*} data in ${bag} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. The path to ROS bag will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    ${topics} =  Split String  ${topics}
    Set Test Variable  $BENCHMARK.INPUT.ROS.TOPICS  ${topics}
    Uses ${bag} at ${rate}x as input

Uses ${duration} of ${bag} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    ${duration} =  Convert Time  ${duration}
    Set Test Variable  $BENCHMARK.INPUT.ROS.DURATION  ${duration}
    Uses ${bag} at ${rate}x as input

Uses ${duration} of ${topics:/.*} data in ${bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    Uses ${duration} of ${topics} data in ${bag} at 1x as input

Uses ${duration} of ${topics:/.*} data in ${bag} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    ${duration} =  Convert Time  ${duration}
    Set Test Variable  $BENCHMARK.INPUT.ROS.DURATION  ${duration}
    Uses ${topics} data in ${bag} at ${rate}x as input

Uses ${launch_file:.+\.launch} as rig
    [Documentation]
    ...  Specifies a launch file for the benchmark case rig.
    ...  Paths are resolved relative to the benchmark suite.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.RIG.ROS.LAUNCH.FILE
    ...  ROS launch file already set to ${BENCHMARK.RIG.ROS.LAUNCH_FILE}
    ${launch_file} =  Resolve File Path or Fail  ${launch_file}
    Set Test Variable  $BENCHMARK.RIG.ROS.LAUNCH.FILE  ${launch_file}

Uses ${launch_file:.+\.launch} in ${package_name} ROS 2 package as rig
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

Generates ${format} report from ${template_name} template in ${package_name} ROS 2 package
    [Documentation]
    ...  Specifies report generation using Sphinx template projects that are
    ...  to be found in ROS package paths, under the reports subdirectory.
    ...  See `Generates ${format} report from ${template_path} template`
    ...  documentation for further reference.
    [Tags]  declarative
    ${package_path} =  Find ROS Package  ${package_name}
    ${template_path} =  Join Path  ${package_path}  reports  ${template_name}
    Directory Should Exist  ${template_path}
    ...  No ${template_name} template found in ${package_name} ROS package
    Generates ${format} report from ${template_path} template

### Hooks ###

Before ROS 2 system benchmark case iteration
    [Documentation]
    ...  See `Before ${some} benchmark case iteration` documentation.
    [Tags]  declarative  override
    Set Environment Variable  ROS_HOME  ${BENCHMARK.CASE.ITERATION.PATH}/_ros

Before ROS 2 system benchmark case rig bringup
    [Documentation]
    ...  See `Before ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Set ROS 2 Parameter For Every Node  use_sim_time  ${BENCHMARK.RIG.ROS.SIMULATED}
    Start recording ROS 2 bag
    ...  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...  cwd=${BENCHMARK.CASE.ITERATION.PATH}  all=yes

For ROS 2 system benchmark case rig bringup
    [Documentation]
    ...  See `For ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Variable Should Exist  ${BENCHMARK.RIG.ROS.LAUNCH.FILE}
    ...  No ROS launch file specified for benchmark rig
    Launch ROS 2 System  main
    ...  ${BENCHMARK.RIG.ROS.LAUNCH.FILE}
    ...  @{BENCHMARK.RIG.ROS.LAUNCH.ARGUMENTS}
    ...  package=${BENCHMARK.RIG.ROS.LAUNCH.PACKAGE}

On ROS 2 system benchmark case rig bringup
    [Documentation]
    ...  See `On ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    # NOTE(hidmic): wait for roslaunch to set parameters
    Sleep  1s

After ROS 2 system benchmark case rig bringup
    [Documentation]
    ...  See `After ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Dump ROS 2 Parameters  cwd=${BENCHMARK.CASE.ITERATION.PATH}

Per ROS 2 system benchmark case iteration
    [Documentation]
    ...  See `Per ${some} benchmark case shutdown` documentation.
    [Tags]  declarative  override
    ${bags} =  Get Variable Value  $BENCHMARK.INPUT.ROS.BAGS  ${null}
    IF  "${bags}" != "${null}"
        ${clock_rate} =  Evaluate  ${BENCHMARK.INPUT.ROS.RATE} * 100
        ${result} =  Play ROS 2 Bag  @{BENCHMARK.INPUT.ROS.BAGS}
        ...          topics=${BENCHMARK.INPUT.ROS.TOPICS}
        ...          duration=${BENCHMARK.INPUT.ROS.DURATION}
        ...          rate=${BENCHMARK.INPUT.ROS.RATE}
        ...          clock=${clock_rate}  delay=${2}
        Should Be Equal  ${result.rc}  ${0}  msg=Failed to play input ROS bags
    END

For ROS 2 system benchmark case rig shutdown
    [Documentation]
    ...  See `For ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    ${result} =  Shutdown ROS 2 System  main
    Should Be Equal  ${result.rc}  ${0}
    ...  msg=ROS system had issues during benchmark

After ROS 2 system benchmark case rig shutdown
    [Documentation]
    ...  See `After ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    ${result} =  Stop recording ROS 2 bag
    ...  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...  cwd=${BENCHMARK.CASE.ITERATION.PATH}
    Should Be Equal  ${result.rc}  ${0}
    ...  msg=Failed to record output ROS bag
