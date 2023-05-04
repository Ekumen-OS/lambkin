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
Documentation    Wrappers for Sphinx CLI applications.

Resource         lambkin/shepherd/robot/resources/library/process.resource


*** Keywords ***
Build Sphinx Documentation
    [Documentation]  Invokes sphinx-build on input to build on output.
    [Arguments]  ${output}=build  ${input}=.  ${builder}=latexpdf
    [Tags]  imperative
    ${result} =  Run Managed Process
    ...  sphinx-build  -M  ${builder}  ${input}  ${output}  -W
    RETURN  ${result}
