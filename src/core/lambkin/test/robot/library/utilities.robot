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
Documentation  Utilities library
Resource  lambkin/robot/resources/library/utilities.resource

Resource  default.resource

Library  Collections


*** Test Cases ***
Can convert to snake case
    ${value} =  Convert To Snake Case  Some non-snake case string
    Should Be Equal  ${value}  some_non_snake_case_string

Can convert keyword arguments to options
    ${patterns} =  Create List  *.robot  *.resource
    ${options} =  Convert To Command Line Options
    ...  output=some_file.txt  config=/etc/some.ini
    ...  exclude=${patterns}  verbose=yes  quiet=no
    ${expected_options} =  Create List
    ...  --output  some_file.txt
    ...  --config  /etc/some.ini
    ...  --exclude  *.robot  *.resource
    ...  --verbose
    Should Be Equal  ${options}  ${expected_options}

Can detect testing scope
    Run Keyword And Expect Error  *
    ...  Should Not Be In Test Scope
    Should Be In Test Scope

Can increment variables
    Variable Should Not Exist  $COUNTER
    ${prevalue} =  Increment Variable In Parent Scope  $COUNTER
    Variable Should Exist  $COUNTER
    Should Be Equal As Integers  ${prevalue}  0
    Should Be Equal As Integers  ${COUNTER}  1
