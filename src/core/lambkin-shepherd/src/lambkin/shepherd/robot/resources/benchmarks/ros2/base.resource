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
Resource  lambkin/shepherd/robot/resources/benchmarks/ros_base.resource

Library   Collections
Library   DateTime
Library   OperatingSystem
Library   String


*** Keywords ***
ROS 2 system benchmark suite
    [Documentation]
    ...  Generic ROS 2 system benchmark suite description.
    [Tags]  descriptive
    Extends base benchmark suite
    Extends ROS base benchmark suite

ROS 2 system benchmark case
    [Documentation]
    ...  Generic ROS 2 system benchmark case description.
    ...
    ...  This description uses a single launch file for a benchmark case rig.
    ...  Input data is fed via ROS 2 bag playback, output data is collected via
    ...  ROS 2 bag recording.
    [Tags]  descriptive
    Extends base benchmark case
    Extends ROS base benchmark case

Uses ${bag} as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. The path to ROS 2 bag will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    Uses ${bag} at 1x as input to ROS 2 system

Uses ${duration} of ${bag} as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS 2 bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    Uses ${duration} of ${bag} at 1x as input to ROS 2 system

Uses ${topics:/.+} data in ${bag} as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. The path to ROS 2 bag will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    Uses ${topics} data in ${bag} at 1x as input to ROS 2 system

Uses ${bag} at ${rate:(\d*\.)?\d+}x as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. The path to ROS 2 bag will be resolved
    ...  relative to the working directory and the benchmark suite
    ...  parent directory.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.INPUT.ROS.BAGS
    ...  ROS bags already set to ${BENCHMARK.INPUT.ROS.BAGS}
    ${paths} =  Create List
    ${path} =  Resolve File Path or Fail  ${bag}
    Warn If ROS 2 Bag Is Compressed  ${path}
    Append To List  ${paths}  ${path}
    Set Test Variable  $BENCHMARK.INPUT.ROS.BAGS  ${paths}
    ${rate} =  Convert To Number  ${rate}
    Set Test Variable  $BENCHMARK.INPUT.ROS.RATE  ${rate}
    Set Test Variable  $BENCHMARK.RIG.ROS.SIMULATED  ${True}

Uses ${topics:/.*} data in ${bag} at ${rate:(\d*\.)?\d+}x as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. The path to ROS 2 bag will be resolved
    ...  relative to the benchmark suite.
    [Tags]  declarative
    ${topics} =  Split String  ${topics}
    Set Test Variable  $BENCHMARK.INPUT.ROS.TOPICS  ${topics}
    Uses ${bag} at ${rate}x as input to ROS 2 system

Uses ${duration} of ${bag} at ${rate:(\d*\.)?\d+}x as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS 2 bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    ${duration} =  Convert Time  ${duration}
    Set Test Variable  $BENCHMARK.INPUT.ROS.DURATION  ${duration}
    Uses ${bag} at ${rate}x as input to ROS 2 system

Uses ${duration} of ${topics:/.*} data in ${bag} as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS 2 bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    Uses ${duration} of ${topics} data in ${bag} at 1x as input to ROS 2 system

Uses ${duration} of ${topics:/.*} data in ${bag} at ${rate:(\d*\.)?\d+}x as input to ROS 2 system
    [Documentation]
    ...  Specifies the ROS 2 bag data that some ROS 2 system benchmark case
    ...  will use for data input. Durations are specified as time strings.
    ...  The path to ROS 2 bag will be resolved relative to the benchmark suite.
    [Tags]  declarative
    ${duration} =  Convert Time  ${duration}
    Set Test Variable  $BENCHMARK.INPUT.ROS.DURATION  ${duration}
    Uses ${topics} data in ${bag} at ${rate}x as input to ROS 2 system

Configures QoS overrides from ${qos_overrides_path} for input to ROS 2 system
    [Documentation]
    ...  Specifies the QoS overrides from a YAML file that some ROS 2 system
    ...  benchmark case will use for ROS 2 bag data input. The path to the
    ...  QoS overrides file will be resolved relative to the benchmark suite.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.INPUT.ROS.QOS_OVERRIDES
    ...  QoS overrides already set to ${$BENCHMARK.INPUT.ROS.QOS_OVERRIDES}
    ${path} =  Resolve File Path or Fail  ${qos_overrides_path}
    Set Test Variable  $BENCHMARK.INPUT.ROS.QOS_OVERRIDES  ${path}

Generates ${format} report from ${template_name} template in ${package_name} ROS 2 package
    [Documentation]
    ...  Specifies report generation using Sphinx template projects that are
    ...  to be found in ROS 2 package paths, under the reports subdirectory.
    ...  See `Generates ${format} report from ${template_path} template`
    ...  documentation for further reference.
    [Tags]  declarative
    ${package_path} =  Find ROS 2 Package  ${package_name}
    ${template_path} =  Join Path  ${package_path}  share    ${package_name}    reports    ${template_name}
    Directory Should Exist  ${template_path}
    ...  No ${template_name} template found in ${package_name} ROS 2 package
    Generates ${format} report from ${template_path} template

### Hooks ###

For ROS 2 system benchmark suite setup
    [Documentation]
    ...  See `For ${some} benchmark suite setup` documentation.
    [Tags]  declarative  override
    Set Suite Variable  $BENCHMARK.OUTPUT.ROS.BAG  output_bag

Before ROS 2 system benchmark case iteration
    [Documentation]
    ...  See `Before ${some} benchmark case iteration` documentation.
    [Tags]  declarative  override
    Set Environment Variable  ROS_LOG_DIR  ${BENCHMARK.CASE.ITERATION.PATH}/_ros

Before ROS 2 system benchmark case rig bringup
    [Documentation]
    ...  See `Before ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Start ROS 2 Daemon
    Start recording ROS 2 bag
    ...  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...  cwd=${BENCHMARK.CASE.ITERATION.PATH}
    ...  all=yes  use-sim-time=true

For ROS 2 system benchmark case rig bringup
    [Documentation]
    ...  See `For ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    Variable Should Exist  ${BENCHMARK.RIG.ROS.LAUNCH.FILE}
    ...  No ROS 2 launch file specified for benchmark rig
    Launch ROS 2 System  main
    ...  ${BENCHMARK.RIG.ROS.LAUNCH.FILE}
    ...  @{BENCHMARK.RIG.ROS.LAUNCH.ARGUMENTS}
    ...  package=${BENCHMARK.RIG.ROS.LAUNCH.PACKAGE}

After ROS 2 system benchmark case rig bringup
    [Documentation]
    ...  See `After ${some} benchmark case rig bringup` documentation.
    [Tags]  declarative  override
    IF  ${BENCHMARK.RIG.ROS.SIMULATED}
        Set ROS 2 Parameter  use_sim_time  ${BENCHMARK.RIG.ROS.SIMULATED}
        Warn If ROS 2 Nodes Are Not Using Sim Time
    END
    Dump ROS 2 Parameters  cwd=${BENCHMARK.CASE.ITERATION.PATH}

Per ROS 2 system benchmark case iteration
    [Documentation]
    ...  See `Per ${some} benchmark case shutdown` documentation.
    [Tags]  declarative  override
    ${bags} =  Get Variable Value  $BENCHMARK.INPUT.ROS.BAGS
    IF  "${bags}" != "${null}"
        ${clock_rate} =  Evaluate  ${BENCHMARK.INPUT.ROS.RATE} * 100
        ${arguments} =  Create Dictionary
        ...    topics=${BENCHMARK.INPUT.ROS.TOPICS}
        ...    duration=${BENCHMARK.INPUT.ROS.DURATION}
        ...    rate=${BENCHMARK.INPUT.ROS.RATE}
        ...    clock=${clock_rate}  delay=${3}
        ${qos_overrides_path} =  Get Variable Value  $BENCHMARK.INPUT.ROS.QOS_OVERRIDES
        IF    "${qos_overrides_path}" != "${null}"
            Set To Dictionary  ${arguments}  qos-profile-overrides-path=${qos_overrides_path}
        END
        ${result} =  Play ROS 2 Bag  @{BENCHMARK.INPUT.ROS.BAGS}  &{arguments}
        Should Be Equal  ${result.rc}  ${0}  msg=Failed to play input ROS 2 bags
    END

For ROS 2 system benchmark case rig shutdown
    [Documentation]
    ...  See `For ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    ${result} =  Shutdown ROS 2 System  main
    Should Be Equal  ${result.rc}  ${0}
    ...  msg=ROS 2 system had issues during benchmark

After ROS 2 system benchmark case rig shutdown
    [Documentation]
    ...  See `After ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    Stop ROS 2 Daemon
    ${result} =  Stop recording ROS 2 bag
    ...  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...  cwd=${BENCHMARK.CASE.ITERATION.PATH}
    Should Be Equal  ${result.rc}  ${0}
    ...  msg=Failed to record output ROS 2 bag
