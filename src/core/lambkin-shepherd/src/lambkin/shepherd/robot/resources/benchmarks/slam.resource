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
...  Template descriptions for SLAM system benchmarks.
...
...  These descriptions provide data post-processing
...  for performance assessment of SLAM systems.

Resource  lambkin/shepherd/robot/resources/benchmarks/base.resource

Library   Collections
Library   String


*** Variables ***
&{BENCHMARK.OUTPUT.SLAM.CORRECTIONS}  &{EMPTY}
${BENCHMARK.OUTPUT.SLAM.USE_GROUNDTRUTH_FROM_FILE}  ${False}


*** Keywords ***
SLAM system benchmark suite
    [Documentation]  Generic SLAM system benchmark suite description.
    [Tags]  descriptive
    Extends base benchmark suite

SLAM system benchmark case
    [Documentation]  Generic SLAM system benchmark case description.
    [Tags]  descriptive
    Extends base benchmark case

Tracks ${trajectory} trajectory
    [Documentation]
    ...  States that some SLAM system benchmark will track the given trajectory
    ...  and its error w.r.t. some groundtruth. See ``evo`` documentation for
    ...  reference on trajectory name patterns.
    [Tags]  declarative
    ${list} =  Get Variable Value  $BENCHMARK.OUTPUT.SLAM.TRAJECTORIES  @{EMPTY}
    List Should Not Contain Value
    ...  ${list}  ${trajectory}  msg=${trajectory} is already tracked
    Append To List  ${list}  ${trajectory}
    Set Non Local Variable  ${BENCHMARK.CASE.SCOPE}  @BENCHMARK.OUTPUT.SLAM.TRAJECTORIES  @{list}

Tracks ${trajectories} trajectories
    [Documentation]
    ...  States that some SLAM system benchmark will track the given trajectories
    ...  and their errors w.r.t. some groundtruth. See ``evo`` documentation for
    ...  reference on trajectory name patterns.
    [Tags]  declarative
    ${trajectories} =  Split String  ${trajectories}
    FOR  ${trajectory}  IN  @{trajectories}
        Tracks ${trajectory} trajectory
    END

Performs trajectory corrections
    [Documentation]
    ...  States that some SLAM system benchmark will correct trajectories being
    ...  tracked before computing any errors w.r.t. some groundtruth. See ``evo``
    ...  documentation for reference on trajectory corrections.
    [Arguments]  &{kwargs}
    [Tags]  declarative
    Set Non Local Variable  ${BENCHMARK.CASE.SCOPE}  $BENCHMARK.OUTPUT.SLAM.CORRECTIONS  ${kwargs}

Uses ${trajectory} as trajectory groundtruth
    [Documentation]
    ...  States that some SLAM system benchmark will use the given trajectory
    ...  as groundtruth when computing trajectory errors.
    ...  See ``evo`` documentation for reference on trajectory name patterns.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH
    ...  Ground truth trajectory is already set to ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}
    ${trajectory_path} =  Resolve File Path  ${trajectory}
    IF  $trajectory_path != $null
        Set Non Local Variable  ${BENCHMARK.CASE.SCOPE}  $BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH  ${trajectory_path}
        Set Non Local Variable  ${BENCHMARK.CASE.SCOPE}  $BENCHMARK.OUTPUT.SLAM.USE_GROUNDTRUTH_FROM_FILE  ${True}
    ELSE
        Set Non Local Variable  ${BENCHMARK.CASE.SCOPE}  $BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH  ${trajectory}
    END

### Hooks ###

Before SLAM system benchmark case rig shutdown
    [Documentation]
    ...  See `Before ${some} benchmark case rig shutdown` documentation.
    [Tags]  declarative  override
    IF  ${BENCHMARK.OUTPUT.SLAM.USE_GROUNDTRUTH_FROM_FILE}
        Copy File  ${BENCHMARK.OUTPUT.SLAM.GROUNDTRUTH}  ${BENCHMARK.CASE.ITERATION.PATH}/.
    END
