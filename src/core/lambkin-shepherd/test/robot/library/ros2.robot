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
Documentation  ROS 2 interoperability library tests
Resource  lambkin/shepherd/robot/resources/library/ros2.resource

Resource  lambkin/shepherd/robot/resources/library/filesystem.resource

Resource  default.resource

Library   Collections
Library   Process
Library   String


*** Test Cases ***
ROS 2 package can be found
    Skip Unless Executable Exists  ros2
    ${path} =  Find ROS 2 Package  launch
    Directory Should Exist  ${path}

Missing ROS 2 package is handled gracefully
    Skip Unless Executable Exists  ros2
    ${path} =  Find ROS 2 Package  not_a_real_package
    Should Be Equal  ${path}  ${None}

ROS 2 parameters can be dumped
    Skip Unless ROS 2 Package Exists  topic_tools
    Start Process  ros2  run  topic_tools  relay  /tf
    Dump ROS 2 Parameters  ${TEST_TEMPDIR}
    Terminate Process
    File Should Exist  ${TEST_TEMPDIR}/relay_params.yaml
    ${config} =  Read YAML File  ${TEST_TEMPDIR}/relay_params.yaml
    ${node} =  Get From Dictionary  ${config}  /relay
    ${params} =  Get From Dictionary  ${node}  ros__parameters
    Dictionary Should Contain Item  ${params}  use_sim_time  ${False}

ROS 2 parameters can be set
    Skip Unless ROS 2 Package Exists  topic_tools
    Start Process  ros2  run  topic_tools  relay  /tf
    Set ROS 2 Parameter  use_sim_time  true  /relay
    Dump ROS 2 Parameters  ${TEST_TEMPDIR}
    Terminate Process
    File Should Exist  ${TEST_TEMPDIR}/relay_params.yaml
    ${config} =  Read YAML File  ${TEST_TEMPDIR}/relay_params.yaml
    ${node} =  Get From Dictionary  ${config}  /relay
    ${params} =  Get From Dictionary  ${node}  ros__parameters
    Dictionary Should Contain Item  ${params}  use_sim_time  ${True}

ROS 2 occupancy grids can be saved
    Skip Unless ROS 2 Package Exists  nav2_map_server
    ${message} =  Get File  ${CURDIR}/data/ros2/occupancy_grid.yaml
    Start Managed Process  ros2  topic  pub
    ...  map  nav_msgs/OccupancyGrid  ${message}
    Save ROS 2 Occupancy Grid Map  empty_map  cwd=${TEST_TEMPDIR}
    Terminate Managed Process
    File Should Exist  ${TEST_TEMPDIR}/empty_map.yaml
    ${metadata} =  Read YAML File  ${TEST_TEMPDIR}/empty_map.yaml
    Dictionary Should Contain Item  ${metadata}  resolution  ${20}
    File Should Exist  ${TEST_TEMPDIR}/empty_map.pgm
    ${image} =  Get Binary File  ${TEST_TEMPDIR}/empty_map.pgm
    ${dimensions} =  Get Line  ${image}  1
    Should Be Equal  "${dimensions}"  "2 2"
    ${data} =  Get Line  ${image}  -1
    Should Be Equal  "${data}"  "\x00\x00\x00\x00"
