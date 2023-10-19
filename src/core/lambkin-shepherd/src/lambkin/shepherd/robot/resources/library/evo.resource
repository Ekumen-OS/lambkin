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
Documentation  Wrappers for evo CLI applications.

Resource  lambkin/shepherd/robot/resources/library/process.resource
Resource  lambkin/shepherd/robot/resources/library/utilities.resource

Library   String
Library   lambkin.shepherd.robot.api.evo


*** Keywords ***
Manipulate trajectories
    [Documentation]  Invokes evo_traj with arguments and options.
    [Arguments]  @{args}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  evo_traj  @{args}  @{options}  cwd=${cwd}
    RETURN  ${result}

Estimate APE
    [Documentation]  Invokes evo_ape with arguments and options.
    [Arguments]  @{args}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  evo_ape  @{args}  @{options}  cwd=${cwd}
    RETURN  ${result}

Estimate RPE
    [Documentation]  Invokes evo_rpe with arguments and options.
    [Arguments]  @{args}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options  &{kwargs}
    ${result} =  Run Managed Process  evo_rpe  @{args}  @{options}  cwd=${cwd}
    RETURN  ${result}

Perform TUM Trajectory Analysis
    [Documentation]  Performs basic trajectory analysis on a set of TUM formatted trajectories.
    [Arguments]  @{trajectories}  ${ref}  ${map}  ${cwd}=.  &{kwargs}
    [Tags]  imperative
    ${result} =  Manipulate Trajectories  tum  @{trajectories}  ref=${ref}  ros_map_yaml=${map}
    ...                                   plot_mode=xy  save_plot=all.png  no_warnings=yes
    ...                                   cwd=${cwd}  &{kwargs}
    Should Be Equal  ${result.rc}  ${0}  msg=${result.stderr}
    Wait Until Created  ${cwd}/all_trajectories.png  timeout=10s
    File Should Exist  ${cwd}/all_trajectories.png

    FOR  ${trajectory}  IN  @{trajectories}
        ${filestem} =  Convert To EVO Filestem  ${trajectory}  name_format=path
        ${result} =  Estimate APE  tum  ${ref}  ${trajectory}  save_results=${filestem}.ape.zip
        ...          save_plot=${filestem}.ape.png  plot_mode=xy  no_warnings=yes  cwd=${cwd}
        ...          &{kwargs}
        Should Be Equal  ${result.rc}  ${0}  msg=${result.stderr}
        Wait Until Created  ${cwd}/${filestem}.ape.zip  timeout=10s
        File Should Exist  ${cwd}/${filestem}.ape.zip
        ${result} =  Estimate RPE  tum  ${ref}  ${trajectory}  save_results=${filestem}.rpe.zip
        ...          save_plot=${filestem}.rpe.png  plot_mode=xy  no_warnings=yes  cwd=${cwd}
        ...          &{kwargs}
        Should Be Equal  ${result.rc}  ${0}  msg=${result.stderr}
        Wait Until Created  ${cwd}/${filestem}.rpe.zip  10s
        File Should Exist  ${cwd}/${filestem}.rpe.zip
    END
