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
Documentation  Framework utilities

Library  lambkin.robot.api.utilities


*** Keywords ***
Get Parent Scope
    [Documentation]  Retrieve caller parent scope.
    [Tags]  imperative
    ${name} =  Get Variable Value  $TEST_NAME
    IF  "${name}" == "${null}"  RETURN  Suite
    RETURN  Test

Should Be In Test Scope
    [Documentation]  Ensure caller is in test scope.
    [Arguments]  ${msg}=Not in test scope
    [Tags]  imperative
    Variable Should Exist  $TEST_NAME  msg=${msg}

Should Not Be In Test Scope
    [Documentation]  Ensure caller is not in test scope
    ...  e.g. in test suite setup or teardown scope.
    [Arguments]  ${msg}=In test scope
    [Tags]  imperative
    Variable Should Not Exist  $TEST_NAME  msg=${msg}

Set Variable In Parent Scope
    [Documentation]
    ...  Set variable in parent scope (test or suite).
    [Arguments]  ${name}  ${value}
    [Tags]  imperative
    ${scope} =  Get Parent Scope
    Run Keyword  Set ${scope} Variable  ${name}  ${value}

Increment Variable In Parent Scope
    [Documentation]
    ...  Increment integer variable in parent scope (test or suite).
    ...
    ...  If variable is not defined, it is initialized to 0.
    ...  Its value before the increment operation is returned.
    [Arguments]  ${variable}  ${initial}=${0}
    [Tags]  imperative
    # robocop: disable=non-local-variables-should-be-uppercase
    ${prevalue} =  Get Variable Value  ${variable}  ${initial}
    ${postvalue} =  Evaluate  int($prevalue) + 1
    Set Variable In Parent Scope  ${variable}  ${postvalue}
    RETURN  ${prevalue}
