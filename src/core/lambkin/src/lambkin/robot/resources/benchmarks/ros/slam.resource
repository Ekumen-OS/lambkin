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
...  Template descriptions for ROS SLAM benchmarks.
...
...  These descriptions provide data post-processing
...  for performance assessment of ROS SLAM systems.
Resource    lambkin/robot/resources/benchmarks/ros/base.resource
Resource    lambkin/robot/resources/benchmarks/slam.resource

Resource    lambkin/robot/resources/library/evo.resource

Library     Collections


*** Keywords ***
ROS 2D SLAM system benchmark suite
    [Documentation]  ROS 2D SLAM system benchmark suite description.
    [Tags]  descriptive
    Extends ROS system benchmark suite
    Extends SLAM system benchmark suite

ROS 2D SLAM system benchmark case
    [Documentation]  ROS 2D SLAM system benchmark case description.
    [Tags]  descriptive
    Extends ROS system benchmark case
    Extends SLAM system benchmark case
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}

### Hooks ###

Before ROS 2D SLAM system benchmark case rig shutdown
    [Documentation]
    ...  See `Before ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    Save ROS Occupancy Grid Map
    ...  map  timeout=10s  cwd=${BENCHMARK.CASE.ITERATION.PATH}
    Set Test Variable  $BENCHMARK.OUTPUT.MAP.PATH
    ...  ${BENCHMARK.CASE.ITERATION.PATH}/map.yaml

After ROS 2D SLAM system benchmark case iteration
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

    ${result} =  Perform trajectory analysis  bag  ${BENCHMARK.OUTPUT.ROS.BAG}
    ...                                       @{BENCHMARK.OUTPUT.SLAM.TRAJECTORIES}
    ...                                       ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
    ...                                       ref=${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
    ...                                       ros_map_yaml=${BENCHMARK.OUTPUT.MAP.PATH}
    ...                                       plot_mode=xy  save_plot=all.png
    ...                                       no_warnings=yes  save_as_tum=yes
    ...                                       &{BENCHMARK.OUTPUT.SLAM.CORRECTIONS}
    ...                                       cwd=${BENCHMARK.CASE.ITERATION.PATH}
    Should Be Equal  ${result.rc}  ${0}
    Wait Until Created  ${BENCHMARK.CASE.ITERATION.PATH}/all_trajectories.png  timeout=10s
    File Should Exist  ${BENCHMARK.CASE.ITERATION.PATH}/all_trajectories.png

    FOR  ${trajectory}  IN  @{BENCHMARK.OUTPUT.SLAM.TRAJECTORIES}
        ${stem} =  Convert To EVO Filestem  ${trajectory}  name_format=ros
        ${result} =  Estimate APE  bag  ${BENCHMARK.OUTPUT.ROS.BAG}
        ...          ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}  ${trajectory}
        ...          save_plot=${stem}.ape.png  save_results=${stem}.ape.zip
        ...          plot_mode=xy  no_warnings=yes  &{BENCHMARK.OUTPUT.SLAM.CORRECTIONS}
        ...          cwd=${BENCHMARK.CASE.ITERATION.PATH}
        Should Be Equal  ${result.rc}  ${0}
        Wait Until Created  ${BENCHMARK.CASE.ITERATION.PATH}/${stem}.ape.zip  timeout=10s
        File Should Exist  ${BENCHMARK.CASE.ITERATION.PATH}/${stem}.ape.zip
        ${result} =  Estimate RPE  bag  ${BENCHMARK.OUTPUT.ROS.BAG}  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
        ...          ${trajectory}  save_plot=${stem}.rpe.png  save_results=${stem}.rpe.zip
        ...          plot_mode=xy  no_warnings=yes  &{BENCHMARK.OUTPUT.SLAM.CORRECTIONS}
        ...          cwd=${BENCHMARK.CASE.ITERATION.PATH}
        Should Be Equal  ${result.rc}  ${0}
        Wait Until Created  ${BENCHMARK.CASE.ITERATION.PATH}/${stem}.rpe.zip  10s
        File Should Exist  ${BENCHMARK.CASE.ITERATION.PATH}/${stem}.rpe.zip
    END
