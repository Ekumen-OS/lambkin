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
Documentation  Wrappers for timemory CLI applications.

Resource  lambkin/robot/resources/library/utilities.resource

Library   OperatingSystem


*** Variables ***
@{SIGNALS}  2  15


*** Keywords ***
Setup timemory-timem
    [Documentation]
    ...  Setup timemory's ``timem`` to sample a process.
    ...
    ...  All configuration is provided through a ${process}_PREFIX
    ...  environment variable, to be used as prefix on process
    ...  execution. It is up to the caller to ensure that is the case.
    [Arguments]  ${process_name}  ${dir}=${OUTPUT_DIR}
    [Tags]  imperative
    ${options} =  Convert To Command Line Options
    ...   output=${dir}/${process_name}.timem.json
    ...   sample-freq=${20}  forward-signal=${SIGNALS}
    ${value} =  Catenate  timem  @{options}  --
    Set Environment Variable  ${process_name}_PREFIX  ${value}

Wait for timemory-timem output
    [Documentation]  Wait for timem output to be created.
    [Arguments]  ${process_name}  ${dir}=${OUTPUT_DIR}  &{kwargs}
    [Tags]  imperative
    Wait Until Created  ${dir}/${process_name}.timem.json  &{kwargs}
