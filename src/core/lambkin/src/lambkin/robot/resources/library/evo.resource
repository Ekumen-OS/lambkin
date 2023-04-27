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

Resource  lambkin/robot/resources/library/process.resource
Resource  lambkin/robot/resources/library/utilities.resource

Library   String
Library   lambkin.robot.api.evo


*** Keywords ***
Perform trajectory analysis
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
