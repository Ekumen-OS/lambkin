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
...  Template descriptions for ROS 2 SLAM benchmarks.
...
...  These descriptions provide data post-processing
...  for performance assessment of ROS 2 SLAM systems.

Resource    lambkin/shepherd/robot/resources/benchmarks/ros2/base.resource
Resource    lambkin/shepherd/robot/resources/benchmarks/slam.resource

Resource    lambkin/shepherd/robot/resources/library/evo.resource

Library     Collections


*** Keywords ***
ROS 2 2D SLAM system benchmark suite
    [Documentation]  ROS 2 2D SLAM system benchmark suite description.
    [Tags]  descriptive
    Extends ROS 2 system benchmark suite
    Extends SLAM system benchmark suite

ROS 2 2D SLAM system benchmark case
    [Documentation]  ROS 2 2D SLAM system benchmark case description.
    [Tags]  descriptive
    Extends ROS 2 system benchmark case
    Extends SLAM system benchmark case
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}

### Hooks ###

Before ROS 2 2D SLAM system benchmark case rig shutdown
    [Documentation]
    ...  See `Before ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    Save ROS 2 Occupancy Grid Map
    ...  map  timeout=10s  cwd=${BENCHMARK.CASE.ITERATION.PATH}
    Set Test Variable  $BENCHMARK.OUTPUT.MAP.PATH
    ...  ${BENCHMARK.CASE.ITERATION.PATH}/map.yaml

After ROS 2 2D SLAM system benchmark case iteration
    [Documentation]
    ...  See `After ${some} benchmark case iteration` documentation.
    [Tags]  declarative  override
    Variable Should Exist  $BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH
    ...                    No ground truth trajectory specified
    Variable Should Exist  $BENCHMARK.OUTPUT.SLAM.TRAJECTORIES
    ...                    No output trajectories specified
    List Should Not Contain Value  ${BENCHMARK.OUTPUT.SLAM.TRAJECTORIES}
    ...                            ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
    ...                            Ground truth trajectory already tracked

    # NOTE(nahuel): TF trajectories are not supported in the current version of evo.
    # See:
    # - https://github.com/MichaelGrupp/evo/issues/443.
    # - https://github.com/MichaelGrupp/evo/blob/v1.22.0/evo/tools/file_interface.py#L279-L280

    ${result} =  Manipulate Trajectories  bag2  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...                                   @{BENCHMARK.OUTPUT.SLAM.TRAJECTORIES}
    ...                                   no_warnings=yes  save_as_tum=yes
    ...                                   cwd=${BENCHMARK.CASE.ITERATION.PATH}
    Should Be Equal  ${result.rc}  ${0}  msg=${result.stderr}

    @{trajectories} =  Create List
    FOR  ${trajectory}  IN  @{BENCHMARK.OUTPUT.SLAM.TRAJECTORIES}
        ${filestem} =  Convert To EVO Filestem  ${trajectory}  name_format=ros
        Append To List  ${trajectories}  ${filestem}.tum
    END

    IF  ${BENCHMARK.OUTPUT.SLAM.USE_GROUNDTRUTH_FROM_FILE}
        ${groundtruth_format} =  Infer Trajectory File Format  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
        IF  "${groundtruth_format}" != "tum"
            ${result} =  Manipulate Trajectories
            ...    ${groundtruth_format}  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
            ...    no_warnings=yes  save_as_tum=yes  cwd=${BENCHMARK.CASE.ITERATION.PATH}
            Should Be Equal  ${result.rc}  ${0}  msg=${result.stderr}
            ${filestem} =  Convert To EVO Filestem  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}  name_format=path
            ${groundtruth} =  Set Variable  ${filestem}.tum
        ELSE
            ${groundtruth} =  Set Variable  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
        END
    ELSE
        ${result} =  Manipulate Trajectories
        ...    bag2  ${BENCHMARK.OUTPUT.ROS.BAG}  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
        ...    no_warnings=yes  save_as_tum=yes  cwd=${BENCHMARK.CASE.ITERATION.PATH}
        Should Be Equal  ${result.rc}  ${0}  msg=${result.stderr}
        ${filestem} =  Convert To EVO Filestem  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}  name_format=ros
        ${groundtruth} =  Set Variable  ${filestem}.tum
    END

    Perform TUM Trajectory Analysis
    ...    @{trajectories}  ref=${groundtruth}  map=${BENCHMARK.OUTPUT.MAP.PATH}
    ...    &{BENCHMARK.OUTPUT.SLAM.CORRECTIONS}  cwd=${BENCHMARK.CASE.ITERATION.PATH}
