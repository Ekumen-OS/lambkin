# Copyright 2022 Ekumen, Inc.
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
Documentation  Process library tests
Resource  lambkin/shepherd/robot/resources/library/process.resource

Resource  default.resource

Library   OperatingSystem


*** Test Cases ***
Foreground process should log stdout
    Reset Process Logs  ${TEST_TEMPDIR}/logs
    ${result} =  Run Managed Process  python3  -u  -c  print('foo')
    Should Be Equal As Integers  ${result.rc}  0
    File Should Exist  ${result.stdout_path}
    ${stdout} =  Get File  ${result.stdout_path}
    Should Be Equal  ${stdout}  foo${\n}

Foreground process should log stderr
    Reset Process Logs  ${TEST_TEMPDIR}/logs
    ${result} =  Run Managed Process  python3  -u  -c
    ...  import sys; print('bar', file\=sys.stderr)
    Should Be Equal As Integers  ${result.rc}  0
    File Should Exist  ${result.stderr_path}
    ${stderr} =  Get File  ${result.stderr_path}
    Should Be Equal  ${stderr}  bar${\n}

Background process should log stderr
    Reset Process Logs  ${TEST_TEMPDIR}/logs
    Start Managed Process  python3  -u  -c
    ...  import sys; print('foo', file\=sys.stderr)
    ${result} =  Wait For Managed Process
    Should Be Equal As Integers  ${result.rc}  0
    File Should Exist  ${result.stderr_path}
    ${stderr} =  Get File  ${result.stderr_path}
    Should Be Equal  ${stderr}  foo${\n}
