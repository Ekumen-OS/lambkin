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
Documentation  Hooks library tests
Resource  lambkin/shepherd/robot/resources/library/hooks.resource

Resource  default.resource

Suite Setup  Setup hooks suite


*** Test Cases ***
Suite scope hooks behave as intended
    Should Not Be True  ${FLAG}
    Run Keywords Registered On Current Suite  To be run on demand
    Should Be True  ${FLAG}
    Set Suite Variable  $FLAG  ${False}
    Clear Keywords Registered On Current Suite  To be run on demand
    Run Keywords Registered On Current Suite  To be run on demand
    Should Not Be True  ${FLAG}

Test scope hooks behave as intended
    Set Test Variable  $FLAG  ${False}
    Register Keyword On Current Test  To be run on demand
    ...  Set Test Variable  $FLAG  ${True}
    Run Keywords Registered On Current Test  To be run on demand
    Should Be True  ${FLAG}
    Set Test Variable  $FLAG  ${False}
    Clear Keywords Registered On Current Test  To be run on demand
    Run Keywords Registered On Current Test  To be run on demand
    Should Not Be True  ${FLAG}


*** Keywords ***
Setup hooks suite
    Set Suite Variable  $FLAG  ${False}
    Register Keyword On Current Suite
    ...  To be run on demand
    ...  Set Suite Variable  $FLAG  ${True}
