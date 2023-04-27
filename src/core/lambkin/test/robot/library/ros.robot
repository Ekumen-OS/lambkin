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
Documentation  ROS interoperability library tests
Resource  lambkin/robot/resources/library/ros.resource

Resource  lambkin/robot/resources/library/filesystem.resource

Resource  default.resource

Library   Collections
Library   Process
Library   String


*** Test Cases ***
Missing ROS master can be detected
    Skip Unless Executable Exists  roscore
    ${running} =  Wait For ROS Master  timeout=2s
    Should Not Be True  ${running}
    Run Keyword And Expect Error  *
    ...  ROS Master Should Be Running

Running ROS master can be detected
    Skip Unless Executable Exists  roscore
    Start Process  roscore
    ${running} =  Wait For ROS Master  timeout=2s
    Should Be True  ${running}
    ROS Master Should Be Running

ROS package can be found
    Skip Unless Executable Exists  rospack
    ${path} =  Find ROS Package  roslaunch
    Directory Should Exist  ${path}

Missing ROS package is handled gracefully
    Skip Unless Executable Exists  rospack
    ${path} =  Find ROS Package  not_a_real_package
    Should Be Equal  ${path}  ${None}

ROS parameters can be set
    Skip Unless Executables Exist  roscore  rosparam
    Bring Up ROS Core
    Set ROS Parameter  use_sim_time  true
    Dump ROS Parameters  ${TEST_TEMPDIR}/params.yaml
    Shutdown ROS Core
    File Should Exist  ${TEST_TEMPDIR}/params.yaml
    ${params} =  Read YAML File  ${TEST_TEMPDIR}/params.yaml
    Dictionary Should Contain Item  ${params}  use_sim_time  ${True}

ROS occupancy grids can be saved
    Skip Unless ROS Package Exists  map_server
    Bring Up ROS Core
    Start Managed Process  rostopic  pub
    ...  -f  ${CURDIR}/data/occupancy_grid.yaml
    ...  --latch  /map  nav_msgs/OccupancyGrid
    Save ROS Occupancy Grid Map  empty_map  cwd=${TEST_TEMPDIR}
    Terminate Managed Process
    Shutdown ROS Core
    File Should Exist  ${TEST_TEMPDIR}/empty_map.yaml
    ${metadata} =  Read YAML File  ${TEST_TEMPDIR}/empty_map.yaml
    Dictionary Should Contain Item  ${metadata}  resolution  ${20}
    File Should Exist  ${TEST_TEMPDIR}/empty_map.pgm
    ${image} =  Get Binary File  ${TEST_TEMPDIR}/empty_map.pgm
    ${dimensions} =  Get Line  ${image}  2
    Should Be Equal  "${dimensions}"  "2 2"
    ${data} =  Get Line  ${image}  -1
    Should Be Equal  "${data}"  "\x00\x00\x00\x00"

Compressed bagfiles can be detected
    Skip Unless Executables Exist  roscore  rostopic  rosbag
    Bring Up ROS Core
    Start Managed Process  rostopic  pub  -r  10  /foo  std_msgs/String  bar
    ${result} =  Run Managed Process  rosbag  record
    ...  -O  ${TEST_TEMPDIR}/empty.bag  --duration  2  --lz4  -a
    ${compressed} =  Warn If Bagfile Is Compressed  ${TEST_TEMPDIR}/empty.bag
    Should Be True  ${compressed}
    Terminate Managed Process
    Shutdown ROS Core
