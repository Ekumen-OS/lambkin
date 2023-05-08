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
Documentation     ROS APIs and utilities.

Resource          lambkin/shepherd/robot/resources/library/process.resource
Resource          lambkin/shepherd/robot/resources/library/utilities.resource

Library           DateTime
Library           lambkin.shepherd.robot.api.ros


*** Keywords ***
Find ROS Package
    [Documentation]  Finds a ROS package path.
    [Arguments]  ${package_name}
    [Tags]  imperative
    ${result} =  Run Process  rospack  find  ${package_name}
    IF  ${result.rc} != 0  RETURN  ${None}
    RETURN  ${result.stdout}

Skip Unless ROS Package Exists
    [Documentation]  Skips test case unless ROS package can be found.
    [Arguments]  ${package_name}  ${msg}=${null}
    [Tags]  imperative
    Skip Unless Executable Exists  rospack
    ${msg} =  Set Variable If  ${msg} == ${null}
    ...  ${package_name} package not found
    ${path} =  Find ROS Package  ${package_name}
    Skip If  "${path}" == "${None}"  msg=${msg}

Bring Up ROS Core
    [Documentation]  Brings up ROS core nodes, fails if already running.
    [Tags]  imperative
    Start Managed Process  roscore  alias=ROSCORE
    Process Should Be Running  ROSCORE
    Wait for ROS Master
    ROS Master Should Be Running

Shutdown ROS Core
    [Documentation]  Shuts down ROS core nodes, fails if not running.
    [Tags]  imperative
    Process Should Be Running  ROSCORE
    ${result} =  Terminate Managed Process  ROSCORE
    Process Should Be Stopped  ROSCORE
    RETURN  ${result}

Set ROS Parameter
    [Documentation]  Sets a ROS parameter.
    [Arguments]  ${name}  ${value}
    [Tags]  imperative
    ${result} =  Run Managed Process  rosparam  set  ${name}  ${value}
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}

Dump ROS Parameters
    [Documentation]  Writes all ROS parameters to file.
    [Arguments]  ${file}=ros_param_dump.yaml  &{kwargs}
    [Tags]  imperative
    ${result} =  Run Managed Process  rosparam  dump  ${file}  -v  &{kwargs}
    Should Be Equal As Integers  ${result.rc}  0

Launch ROS System
    [Documentation]  Launches a ROS system via roslaunch.
    [Arguments]  ${name}  ${launch_file}  @{arguments}  ${package}=${null}  &{kwargs}
    [Tags]  imperative
    Reverse List  ${arguments}
    Append To List  ${arguments}  ${launch_file}
    IF  "${package}" != "${null}"  Append To List  ${arguments}  ${package}
    Reverse List  ${arguments}
    ${process} =  Start Managed Process  roslaunch  --wait  --log
    ...           --sigint-timeout  20  --sigterm-timeout  20
    ...           @{arguments}  &{kwargs}  alias=${name}
    Process Should Be Running  ${name}  msg=${name} ROS system is not running
    RETURN  ${process}

Shutdown ROS System
    [Documentation]  Shuts down a ROS system, fails if not running.
    [Arguments]  ${name}
    [Tags]  imperative
    ${result} =  Terminate Managed Process  ${name}
    Process Should Be Stopped  ${name}
    RETURN  ${result}

Start Recording ROS Bag
    [Documentation]  Starts recording a ROS bag in the background.
    [Arguments]  ${bag}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  output-name=${bag}  &{kwargs}
    ${process} =  Start Managed Process  rosbag  record  @{options}  alias=RECORD:${bag}  cwd=${cwd}
    Process Should Be Running  RECORD:${bag}  msg=${bag} recording process is not running
    RETURN  ${process}

Play ROS Bags
    [Documentation]  Plays one or more ROS bags while clocking.
    [Arguments]  @{bags}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  rosbag  play  @{options}  @{bags}  cwd=${cwd}
    RETURN  ${result}

Stop Recording ROS Bag
    [Documentation]  Stops recording a ROS bag, fails if it is not.
    [Arguments]  ${bag}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    Process Should Be Running  RECORD:${bag}
    ${result} =  Terminate Managed Process  RECORD:${bag}
    Process Should Be Stopped  RECORD:${bag}
    ${path} =  Join Path  ${cwd}  ${bag}
    Wait Until Created  ${path}  &{kwargs}
    RETURN  ${result}

Save ROS Occupancy Grid Map
    [Documentation]
    ...  Saves a ROS occupancy grid map using `map_server` utilities.
    [Arguments]  ${map}=map  ${topic}=${null}  ${timeout}=${-1}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${args} =  Create List  -f  ${map}
    IF  "${topic}" != "${null}"  Append To List  ${args}  map:=${topic}
    Start Managed Process  rosrun  map_server  map_saver  @{args}
    ...                    cwd=${cwd}  alias=SAVE:${map}  &{kwargs}
    ${result} =  Wait For Managed Process  SAVE:${map}  timeout=${timeout}
    IF  "${result}" == "${null}"
        ${result} =  Terminate Managed Process  SAVE:${map}
    END
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    Wait Until Created  ${cwd}/${map}.pgm  timeout=${timeout}
    File Should Exist  ${cwd}/${map}.yaml
