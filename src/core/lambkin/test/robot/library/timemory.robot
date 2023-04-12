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
Documentation  Timemory interoperability library tests
Resource  lambkin/robot/resources/library/timemory.resource

Resource  lambkin/robot/resources/library/filesystem.resource

Resource  default.resource

Library   OperatingSystem
Library   Process


*** Test Cases ***
Timem can be hooked up
    Skip Unless Executable Exists  timem
    Setup timemory-timem  python3  dir=${TEST_TEMPDIR}
    Environment Variable Should Be Set  python3_PREFIX
    ${result} =  Run Process
    ...  $python3_PREFIX python3 -c 'import time; time.sleep(1)'  shell=yes
    Should Be Equal As Integers  ${result.rc}  0  msg=${result.stderr}
    Wait for timemory-timem output  python3  dir=${TEST_TEMPDIR}
    File Should Exist  ${TEST_TEMPDIR}/python3.timem.json
