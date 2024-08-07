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
Documentation     ROS 2 APIs and utilities.

Resource          lambkin/shepherd/robot/resources/library/process.resource
Resource          lambkin/shepherd/robot/resources/library/utilities.resource

Library           DateTime
Library           lambkin.shepherd.robot.api.ros2


*** Keywords ***
Find ROS 2 Package
    [Documentation]  Finds a ROS 2 package path.
    [Arguments]  ${package_name}  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Process  ros2  pkg  prefix  @{options}  ${package_name}
    IF  ${result.rc} != 0  RETURN  ${None}
    RETURN  ${result.stdout}

Skip Unless ROS 2 Package Exists
    [Documentation]  Skips test case unless ROS 2 package can be found.
    [Arguments]  ${package_name}  ${msg}=${null}
    [Tags]  imperative
    Skip Unless Executable Exists  ros2
    ${msg} =  Set Variable If  ${msg} == ${null}
    ...  ${package_name} package not found
    ${path} =  Find ROS 2 Package  ${package_name}
    Skip If  "${path}" == "${None}"  msg=${msg}

Get ROS 2 Node List
    [Documentation]  Returns a list of available ROS 2 nodes.
    [Arguments]  &{kwargs}
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  ros2  node  list  @{options}
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    @{nodes} =  Split To Lines  ${result.stdout}
    RETURN  @{nodes}

Get ROS 2 Parameter
    [Documentation]  Gets a ROS 2 parameter value from the given node.
    [Arguments]  ${name}  ${node}
    ${result} =  Run Process  ros2  param  get  ${node}  ${name}  --hide-type
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    ${value} =  Evaluate  ${result.stdout}
    RETURN  ${value}

Start ROS 2 Daemon
    [Documentation]  Stars the ROS 2 daemon.
    [Arguments]  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  ros2  daemon  start  @{options}
    RETURN  ${result}

Stop ROS 2 Daemon
    [Documentation]  Stops the ROS 2 daemon.
    [Arguments]  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  ros2  daemon  stop  @{options}
    RETURN  ${result}

Launch ROS 2 System
    [Documentation]  Launches a ROS 2 system via ros2 launch.
    [Arguments]  ${name}  ${launch_file}  @{arguments}  ${package}=${null}  &{kwargs}
    [Tags]  imperative
    Reverse List  ${arguments}
    Append To List  ${arguments}  ${launch_file}
    IF  "${package}" != "${null}"  Append To List  ${arguments}  ${package}
    Reverse List  ${arguments}
    ${process} =  Start Managed Process  ros2  launch
    ...           @{arguments}  &{kwargs}  alias=${name}
    ...           env:OVERRIDE_LAUNCH_PROCESS_OUTPUT=log
    Process Should Be Running  ${name}  msg=${name} ROS system is not running
    RETURN  ${process}

Shutdown ROS 2 System
    [Documentation]  Shuts down a ROS 2 system, fails if not running.
    [Arguments]  ${name}
    [Tags]  imperative
    ${result} =  Terminate Managed Process  ${name}
    Process Should Be Stopped  ${name}
    RETURN  ${result}

Start Recording ROS 2 Bag
    [Documentation]  Starts recording a ROS 2 bag in the background.
    [Arguments]  ${bag}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  output=${bag}  &{kwargs}
    ${process} =  Start Managed Process  ros2  bag  record  @{options}  alias=RECORD:${bag}  cwd=${cwd}
    Process Should Be Running  RECORD:${bag}  msg=${bag} recording process is not running
    RETURN  ${process}

Play ROS 2 Bag
    [Documentation]  Plays one ROS 2 bag.
    [Arguments]  ${bag}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  ros2  bag  play  @{options}  ${bag}  cwd=${cwd}
    RETURN  ${result}

Stop Recording ROS 2 Bag
    [Documentation]  Stops recording a ROS 2 bag, fails if it is not.
    [Arguments]  ${bag}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    Process Should Be Running  RECORD:${bag}
    ${result} =  Terminate Managed Process  RECORD:${bag}
    Process Should Be Stopped  RECORD:${bag}
    ${path} =  Join Path  ${cwd}  ${bag}
    Wait Until Created  ${path}  &{kwargs}
    RETURN  ${result}

Save ROS 2 Occupancy Grid Map
    [Documentation]
    ...  Saves a ROS occupancy grid map using `map_server` utilities.
    [Arguments]  ${map}=map  ${topic}=${null}  ${timeout}=${-1}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${args} =  Create List  -f  ${map}
    IF  "${topic}" != "${null}"  Append To List  ${args}  -t  ${topic}
    Start Managed Process  ros2  run  nav2_map_server  map_saver_cli  @{args}
    ...                    cwd=${cwd}  alias=SAVE:${map}  &{kwargs}
    ${result} =  Wait For Managed Process  SAVE:${map}  timeout=${timeout}
    IF  "${result}" == "${null}"
        ${result} =  Terminate Managed Process  SAVE:${map}
    END
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    Wait Until Created  ${cwd}/${map}.pgm  timeout=${timeout}
    File Should Exist  ${cwd}/${map}.yaml
