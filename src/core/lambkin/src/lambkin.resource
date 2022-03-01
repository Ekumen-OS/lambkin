# Copyright 2022 Ekumen, Inc.
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
Documentation     Localization And Mapping Benchmarking Toolkit

Library           Collections
Library           OperatingSystem
Library           Process
Library           String
Library           DateTime
Library           json

Library           lambkin
Library           lambkin.Hooks

*** Variables ***
${INPUT_DIR}      %{INPUT_DIR=.}

*** Keywords ***
# TODO(hidmic): spread out content in different files

Create Json File
    [Arguments]  ${path}  &{dict}
    ${json} =  Json.Dumps  ${dict}  indent=${2}
    Create File  ${path}  ${json}

Log ROS Parameters
    [Arguments]  ${cwd}=.
    ${result} =  Run Process  rosparam  dump  ros_param_dump.yaml  -v
    ...          cwd=${cwd}
    Log  ${result.stdout}
    Log  ${result.stderr}

Find ROS Package
    [Arguments]  ${package}
    ${result} =  Run Process  rospack  find  ${package}
    [Return]  ${result.stdout}

Bring Up ROS Core
    Start Process  roscore  alias=ROS_MASTER
    Wait for ROS Master  timeout=10s
    ROS Master Should Be Running
    Process Should Be Running  ROS_MASTER
    # TODO(hidmic): make it optional in case data is
    #               generated or gathered on the fly
    Run Process  rosparam  set  use_sim_time  true

Shutdown ROS Core
    Terminate Process  ROS_MASTER
    Process Should Be Stopped  ROS_MASTER

Bring Up ROS System
    [Arguments]  ${package}  ${launch_file}  &{kwargs}
    Start Process  roslaunch  --wait
    ...            --sigint-timeout  20
    ...            --sigterm-timeout  20
    ...            ${package}  ${launch_file}  &{kwargs}
    ...            alias=ROS_SYSTEM  stdout=DEVNULL
    Process Should Be Running  ROS_SYSTEM  msg=ROS system is not running

Shutdown ROS System
    Terminate Process  ROS_SYSTEM
    Process Should Be Stopped  ROS_SYSTEM
    # TODO(hidmic): log errors coming the ROS system somehow

Start Recording ROS Bag
    [Arguments]  ${bag}  &{kwargs}
    Start Process  rosbag  record  --quiet  --all  --output-name  ${bag}
    ...            alias=${bag}  stdout=DEVNULL  stderr=DEVNULL  &{kwargs}
    Process Should Be Running  ${bag}  msg=${bag} recording process is not running

Play ROS Bags
    [Arguments]  ${bags}  ${topics}=${null}  ${start}=${null}  ${duration}=${null}
    ...          ${rate}=${null}  &{kwargs}
    ${bags} =  Split String  ${bags}
    FOR  ${bag}  IN  @{bags}
        File Should Exist  ${bag}
        # TODO(nahuel): warning if ROS bag is compressed
    END
    @{options} =  Create List  --quiet  --clock
    IF  '${topics}' != '${null}' and '${topics}' != 'all'
        @{topics} =  Split String  ${topics}
        Append To List  ${options}  --topics  @{topics}
    END
    IF  '${start}' != '${null}'
        ${start} =  Convert Time  ${start}
        Append To List  ${options}  --start  ${start}
    END
    IF  '${duration}' != '${null}'
        ${duration} =  Convert Time  ${duration}
        Append To List  ${options}  --duration  ${duration}
    END
    IF  '${rate}' != '${null}'
        Append To List  ${options}  --rate  ${rate}
    END
    Run Process  rosbag  play  @{bags}  @{options}
    ...          stdout=DEVNULL  stderr=DEVNULL  &{kwargs}

Stop Recording ROS Bag
    [Arguments]  ${bag}  ${cwd}=.
    Terminate Process  ${bag}
    ${bag}  Join Path  ${cwd}  ${bag}
    Wait Until Created  ${bag}

Save 2D Map
    [Arguments]  ${map}  ${timeout}=5s  ${cwd}=.  &{kwargs}
    Process Should Be Running  ROS_SYSTEM
    Run Process  rosrun  map_server  map_saver  -f  ${map}
    ...          cwd=${cwd}  timeout=${timeout}  &{kwargs}
    Wait Until Created  ${cwd}/${map}.pgm
    File Should Exist  ${cwd}/${map}.yaml

Plot 2D Trajectories
    [Arguments]  ${bag}  ${reference}  @{trajectories}
    ...          ${map}=${null}  ${cwd}=.
    ${bag}  Join Path  ${cwd}  ${bag}
    File Should Exist  ${bag}
    @{options} =  Create List
    IF  '${map}' != '${null}'
        ${map}  Join Path  ${cwd}  ${map}.yaml
        File Should Exist  ${map}
        Append To List  ${options}  --ros_map_yaml  ${map}
    END
    ${result} =  Run Process  evo_traj  bag  ${bag}  @{trajectories}
    ...          --ref  ${reference}
    ...          --no_warnings
    ...          --plot_mode  xy
    ...          --t_max_diff  0.1
    ...          --align
    ...          --save_plot  traj.svg
    ...          @{options}
    ...          stdout=DEVNULL  stderr=DEVNULL  cwd=${cwd}
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    Wait Until Created  ${cwd}/traj_trajectories.svg

Compute APE
    # TODO(nahuel): compute error between the reference and a list of
    #               trajectories.
    [Arguments]  ${bag}  ${reference}  ${estimate}  ${cwd}=.
    ${bag}  Join Path  ${cwd}  ${bag}
    File Should Exist  ${bag}
    Run Process  evo_config  set  save_traj_in_zip
    ${result} =  Run Process  evo_ape  bag  ${bag}
    ...          ${reference}  ${estimate}
    ...          --no_warnings
    ...          --t_max_diff  0.1
    ...          --plot_mode  xy
    ...          --align
    ...          --save_plot  ape.svg
    ...          --save_results  ape.zip
    ...          stdout=DEVNULL  stderr=DEVNULL  cwd=${cwd}
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    Wait Until Created  ${cwd}/ape.zip

Compute RPE
    # TODO(nahuel): compute error between the reference and a list of
    #               trajectories.
    [Arguments]  ${bag}  ${reference}  ${estimate}  ${cwd}=.
    ${bag}  Join Path  ${cwd}  ${bag}
    File Should Exist  ${bag}
    Run Process  evo_config  set  save_traj_in_zip
    ${result} =  Run Process  evo_rpe  bag  ${bag}
    ...          ${reference}  ${estimate}
    ...          --no_warnings
    ...          --t_max_diff  0.1
    ...          --plot_mode  xy
    ...          --align
    ...          --save_plot  rpe.svg
    ...          --save_results  rpe.zip
    ...          stdout=DEVNULL  stderr=DEVNULL  cwd=${cwd}
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    Wait Until Created  ${cwd}/rpe.zip

Aggregate Metrics
    [Arguments]  @{metrics}  ${cwd}=.
    ${runs} =  List Directories In Directory  ${cwd}/runs  absolute=True
    ${result} =  Run Process  lambkin  aggregate  @{runs}
    ...          --metrics  @{metrics}
    ...          --functions  median  mean  std
    ...          cwd=${cwd}
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}

Run Keyword In Run Directory
    [Arguments]  ${keyword}  @{arguments}
    Run Keyword  ${keyword}  @{arguments}  cwd=${RUN_DIR}

Run Keyword In Test Directory
    [Arguments]  ${keyword}  @{arguments}
    Run Keyword  ${keyword}  @{arguments}  cwd=${TEST_DIR}

Use ${duration} of ${topics:all|/.*} data in ${bags:.+\.bag$} at ${rate:(\d*\.)?\d+}x as input, but start ${start} in
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}  start=${start}  duration=${duration}  rate=${rate}

Use ${duration} of ${topics:all|/.*} data in ${bags:.+\.bag$} at ${rate:(\d*\.)?\d+}x as input
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}  duration=${duration}  rate=${rate}

Use ${duration} of ${topics:all|/.*} data in ${bags:.+\.bag$} as input, but start ${start} in
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}  start=${start}  duration=${duration}

Use ${duration} of ${topics:all|/.*} data in ${bags:.+\.bag$} as input
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}  duration=${duration}

Use ${topics:all|/.*} data in ${bags:.+\.bag$} at ${rate:(\d*\.)?\d+}x as input, but start ${start} in
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}  start=${start}  rate=${rate}

Use ${topics:all|/.*} data in ${bags:.+\.bag$} at ${rate:(\d*\.)?\d+}x as input
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}  rate=${rate}

Use ${topics:all|/.*} data in ${bags:.+\.bag$} as input, but start ${start} in
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}  start=${start}

Use ${topics:all|/.*} data in ${bags:.+\.bag$} as input
    Register Keyword  After system bring up
    ...               Play ROS Bags  ${bags}  ${topics}

Record output data in ${bag}
    Register Keyword  Before system bring up
    ...               Run Keyword In Run Directory  Start recording ROS bag  ${bag}
    Register Keyword  After system shutdown
    ...               Run Keyword In Run Directory  Stop recording ROS bag  ${bag}

Track ${trajectories} trajectories
    @{trajectories} =  Split String  ${trajectories}
    ${length} =  Get Length  ${trajectories}
    Should Be True  ${length} > 1
    Record output data in outcome.bag
    Register Keyword  After system shutdown
    ...               Run Keyword In Run Directory  Plot 2D trajectories  outcome.bag  @{trajectories}
    Register Keyword  After system shutdown
    ...               Run Keyword In Run Directory  Compute APE  outcome.bag  @{trajectories}
    Register Keyword  After system shutdown
    ...               Run Keyword In Run Directory  Compute RPE  outcome.bag  @{trajectories}
    Register Keyword  After finishing benchmark
    ...               Run Keyword In Test Directory  Aggregate metrics  ape  rpe

Save the resulting map
    Register Keyword  Before system shutdown
    ...               Run Keyword In Run Directory  Save 2D Map  map

Set timem output path
    Set Environment Variable  TIMEM_OUTPUT  ${RUN_DIR}/timem_output

Wait for timem output
    Wait Until Created  ${RUN_DIR}/timem_output.json

Use a sampling rate of ${freq} Hz to track computational performance
    Set Environment Variable  LAMBKIN_MAIN_NODE_PREFIX  timem --forward-signal 2 15 -f ${freq} --
    Register Keyword  Before system bring up  Set timem output path
    Register Keyword  After system shutdown  Wait for timem output
    Register Keyword  After finishing benchmark
    ...               Run Keyword In Test Directory  Aggregate metrics  cpu_usage  peak_rss  virtual_memory

Use ${launch_file} in ${package} package to launch
    Register Keyword  Before starting iteration
    ...               Bring Up ROS Core
    Register Keyword  System bring up
    ...               Bring Up ROS System  ${package}  ${launch_file}
    Register Keyword  After system bring up
    ...               Run Keyword In Run Directory  Log ROS Parameters
    Register Keyword  System shutdown
    ...               Shutdown ROS System
    Register Keyword  After finishing iteration
    ...               Shutdown ROS Core

Generate report using ${report} in ${package} package
    # TODO(nahuel): have a declarative way of running this on Suite Teardown
    ${package_path} =  Find ROS Package  ${package}
    Copy Files  ${package_path}/config/${report}  ${SUITE_SOURCE}  ${OUTPUT_DIR}
    ${result} =  Run Process  rst2pdf  ${report}  -o  report.pdf
    ...          cwd=${OUTPUT_DIR}
    ${_}  ${basename} =  Split Path  ${SUITE_SOURCE}
    Remove Files  ${OUTPUT_DIR}/${report}  ${OUTPUT_DIR}/${basename}
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}

Wait for ${system} system
    Sleep  3 seconds

Before starting ${system} benchmark
    Run Registered Keywords  Before starting benchmark

After finishing ${system} benchmark
    Run Registered Keywords  After finishing benchmark

Before starting ${system} iteration
    Run Registered Keywords  Before starting iteration

After finishing ${system} iteration
    Run Registered Keywords  After finishing iteration

Before ${system} system bring up
    Run Registered Keywords  Before system bring up

After ${system} system bring up
    Run Registered Keywords  After system bring up

Before ${system} system shutdown
    Run Registered Keywords  Before system shutdown

After ${system} system shutdown
    Run Registered Keywords  After system shutdown

Convert To Directory Name
    [Arguments]  ${name}
    ${filename} =  Convert To Lower Case  ${name}
    ${filename} =  Replace String  ${filename}  ${SPACE}  _
    [Return]  ${filename}

Benchmark ${system} for ${iterations} iterations
    [Teardown]  Terminate All Processes
    ${prefix} =  Convert To Directory Name  ${TEST_NAME}
    ${prefix} =  Join Path  ${OUTPUT_DIR}  tests  ${prefix}
    Create Directory  ${prefix}
    Empty Directory  ${prefix}
    Set Test Variable  ${TEST_DIR}  ${prefix}
    ${timestamp} =  Get Current Date
    ${metadata_path} =  Join Path  ${prefix}  metadata.json
    Create Json File  ${metadata_path}
    ...  name=${TEST_NAME}
    ...  timestamp=${timestamp}
    ...  iterations=${iterations}
    ...  system=${system}
    Run Keyword  Before starting ${system} benchmark
    Log To Console  ${\n}
    FOR  ${index}  IN RANGE  1  ${iterations} + 1
        Log To Console  Benchmarking ${system}... ${index}/${iterations}
        ${run_prefix} =  Join Path  ${prefix}  runs/${index}
        Create Directory  ${run_prefix}
        Empty Directory  ${run_prefix}
        Set Test Variable  ${RUN_DIR}  ${run_prefix}
        Run Keyword  Before starting ${system} iteration
        Run Keyword  Before ${system} system bring up
        Run Registered Keywords  System bring up
        Run Keyword  Wait for ${system} system
        Run Keyword  After ${system} system bring up
        Run Keyword  Before ${system} system shutdown
        Run Registered Keywords  System shutdown
        Run Keyword  After ${system} system shutdown
        Run Keyword  After finishing ${system} iteration
    END
    Run Keyword  After finishing ${system} benchmark
    All Registered Keywords Should Have Run

Benchmark ${system} with default settings
    Benchmark ${system} for 100 iterations

Lambkin Setup
    ${dir} =  Join Path  ${OUTPUT_DIR}  tests
    Create Directory  ${dir}
    Empty Directory  ${dir}
