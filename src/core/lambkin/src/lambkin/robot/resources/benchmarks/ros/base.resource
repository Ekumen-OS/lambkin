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
...  Template descriptions for ROS system benchmarks.
...
...  These descriptions instrument benchmark rigs for
...  performance assessment of ROS based systems.

Resource  lambkin/robot/resources/library/filesystem.resource
Resource  lambkin/robot/resources/library/ros.resource
Resource  lambkin/robot/resources/benchmarks/base.resource

Library   Collections
Library   DateTime
Library   OperatingSystem
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
ROS system benchmark suite
    [Documentation]
    ...  Generic ROS system benchmark suite description.
    [Tags]  descriptive
    Extends base benchmark suite

ROS system benchmark case
    [Documentation]
    ...  Generic ROS system benchmark case description.
    ...
    ...  This description uses a single launch file for a benchmark case rig.
    ...  Input data is fed via ROS bag playback, output data is collected via
    ...  ROS bag recording.
    [Tags]  descriptive
    Extends base benchmark case

Uses ${bags:.+\.bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Paths to ROS bags will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    Uses ${bags} at 1x as input

Uses ${duration} of ${bags:.+\.bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  Paths to ROS bags will be resolved relative to the benchmark suite.
    [Tags]  declarative
    Uses ${duration} of ${bags} at 1x as input

Uses ${topics:/.+} data in ${bags:.+\.bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Paths to ROS bags will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    Uses ${topics} data in ${bags} at 1x as input

Uses ${bags:.+\.bag$} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Paths to ROS bags will be resolved
    ...  relative to the working directory and the benchmark suite
    ...  parent directory.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.INPUT.ROS.BAGS
    ...  ROS bags already set to ${BENCHMARK.INPUT.ROS.BAGS}
    ${paths} =  Create List
    ${bags} =  Split String  ${bags}
    FOR  ${bag}  IN  @{bags}
        ${path} =  Resolve File Path or Fail  ${bag}
        Warn If Bagfile Is Compressed  ${path}
        Append To List  ${paths}  ${path}
    END
    Set Test Variable  $BENCHMARK.INPUT.ROS.BAGS  ${paths}
    ${rate} =  Convert To Number  ${rate}
    Set Test Variable  $BENCHMARK.INPUT.ROS.RATE  ${rate}
    Set Test Variable  $BENCHMARK.RIG.ROS.SIMULATED  ${True}

Uses ${topics:/.*} data in ${bags:.+\.bag} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Paths to ROS bags will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    ${topics} =  Split String  ${topics}
    Set Test Variable  $BENCHMARK.INPUT.ROS.TOPICS  ${topics}
    Uses ${bags} at ${rate}x as input

Uses ${duration} of ${bags:.+\.bag} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  Paths to ROS bags will be resolved relative to the benchmark suite.
    [Tags]  declarative
    ${duration} =  Convert Time  ${duration}
    Set Test Variable  $BENCHMARK.INPUT.ROS.DURATION  ${duration}
    Uses ${bags} at ${rate}x as input

Uses ${duration} of ${topics:/.*} data in ${bags:.+\.bag} as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  Paths to ROS bags will be resolved relative to the benchmark suite.
    [Tags]  declarative
    Uses ${duration} of ${topics} data in ${bags} at 1x as input

Uses ${duration} of ${topics:/.*} data in ${bags:.+\.bag} at ${rate:(\d*\.)?\d+}x as input
    [Documentation]
    ...  Specifies the ROS bag data that some ROS system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  Paths to ROS bags will be resolved relative to the benchmark suite.
    [Tags]  declarative
    ${duration} =  Convert Time  ${duration}
    Set Test Variable  $BENCHMARK.INPUT.ROS.DURATION  ${duration}
    Uses ${topics} data in ${bags} at ${rate}x as input

Uses ${launch_file:.+\.launch} as rig
    [Documentation]
    ...  Specifies a launch file for the benchmark case rig.
    ...  Paths are resolved relative to the benchmark suite.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.RIG.ROS.LAUNCH.FILE
    ...  ROS launch file already set to ${BENCHMARK.RIG.ROS.LAUNCH_FILE}
    ${launch_file} =  Resolve File Path or Fail  ${launch_file}
    Set Test Variable  $BENCHMARK.RIG.ROS.LAUNCH.FILE  ${launch_file}

Uses ${launch_file:.+\.launch} in ${package_name} ROS package as rig
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

Generates ${format} report from ${template_name} template in ${package_name} ROS package
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

Before ROS system benchmark case iteration
    [Documentation]
    ...  See `Before ${some} benchmark case iteration` documentation.
    [Tags]  declarative  override
    Set Environment Variable  ROS_HOME  ${BENCHMARK.CASE.ITERATION.PATH}/_ros
    Bring Up ROS Core

Before ROS system benchmark case rig bringup
    [Documentation]
    ...  See `Before ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Set ROS Parameter  use_sim_time  ${BENCHMARK.RIG.ROS.SIMULATED}
    Start recording ROS bag
    ...  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...  cwd=${BENCHMARK.CASE.ITERATION.PATH}  all=yes

For ROS system benchmark case rig bringup
    [Documentation]
    ...  See `For ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Variable Should Exist  ${BENCHMARK.RIG.ROS.LAUNCH.FILE}
    ...  No ROS launch file specified for benchmark rig
    Launch ROS System  main
    ...  ${BENCHMARK.RIG.ROS.LAUNCH.FILE}
    ...  @{BENCHMARK.RIG.ROS.LAUNCH.ARGUMENTS}
    ...  package=${BENCHMARK.RIG.ROS.LAUNCH.PACKAGE}

On ROS system benchmark case rig bringup
    [Documentation]
    ...  See `On ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    # NOTE(hidmic): wait for roslaunch to set parameters
    Sleep  1s

After ROS system benchmark case rig bringup
    [Documentation]
    ...  See `After ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Dump ROS Parameters  cwd=${BENCHMARK.CASE.ITERATION.PATH}

Per ROS system benchmark case iteration
    [Documentation]
    ...  See `Per ${some} benchmark case shutdown` documentation.
    [Tags]  declarative  override
    ${bags} =  Get Variable Value  $BENCHMARK.INPUT.ROS.BAGS  ${null}
    IF  "${bags}" != "${null}"
        ${clock_rate} =  Evaluate  ${BENCHMARK.INPUT.ROS.RATE} * 100
        ${result} =  Play ROS Bags  @{BENCHMARK.INPUT.ROS.BAGS}
        ...          topics=${BENCHMARK.INPUT.ROS.TOPICS}
        ...          duration=${BENCHMARK.INPUT.ROS.DURATION}
        ...          rate=${BENCHMARK.INPUT.ROS.RATE}  clock=yes
        ...          hz=${clock_rate}  delay=${2}  quiet=yes
        Should Be Equal  ${result.rc}  ${0}  msg=Failed to play input ROS bags
    END

For ROS system benchmark case rig shutdown
    [Documentation]
    ...  See `For ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    ${result} =  Shutdown ROS System  main
    Should Be Equal  ${result.rc}  ${0}
    ...  msg=ROS system had issues during benchmark

After ROS system benchmark case rig shutdown
    [Documentation]
    ...  See `After ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    ${result} =  Stop recording ROS bag
    ...  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...  cwd=${BENCHMARK.CASE.ITERATION.PATH}
    Should Be Equal  ${result.rc}  ${0}
    ...  msg=Failed to record output ROS bag

After ROS system benchmark case iteration
    [Documentation]
    ...  See `After ${some} benchmark case iteration` documentation.
    [Tags]  declarative  override
    ${result} =  Shutdown ROS Core
    Should Be Equal  ${result.rc}  ${0}
    ...  msg=ROS master had issues during benchmark
