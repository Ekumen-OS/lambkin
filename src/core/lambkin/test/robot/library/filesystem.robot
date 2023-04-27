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
Documentation  Filesystem library tests
Resource  lambkin/robot/resources/library/filesystem.resource

Resource  default.resource

Library   Collections


*** Test Cases ***
Path to some executable can be found
    ${path} =  Find Executable Path  python3
    Should Not Be Equal  ${path}  ${None}

Nonexistent directory can be reset
    Directory Should Not Exist  ${TEST_TEMPDIR}/non-existent-dir
    Reset Directory  ${TEST_TEMPDIR}/non-existent-dir
    Directory Should Exist  ${TEST_TEMPDIR}/non-existent-dir
    Directory Should Be Empty  ${TEST_TEMPDIR}/non-existent-dir

Pre-existent directory can be reset
    Create File  ${TEST_TEMPDIR}/existent-dir/empty-file  ""
    Directory Should Exist  ${TEST_TEMPDIR}/existent-dir
    Directory Should Not Be Empty  ${TEST_TEMPDIR}/existent-dir
    Reset Directory  ${TEST_TEMPDIR}/existent-dir
    Directory Should Exist  ${TEST_TEMPDIR}/existent-dir
    Directory Should Be Empty  ${TEST_TEMPDIR}/existent-dir

JSON file is correctly written
    Write JSON File  ${TEST_TEMPDIR}/test.json
    ...  foo=bar  num=${0}  indent=${None}
    File Should Exist  ${TEST_TEMPDIR}/test.json
    ${content} =  Get File  ${TEST_TEMPDIR}/test.json
    Should Be Equal  "${content}"  "{\"foo\": \"bar\", \"num\": 0}"

YAML file is correctly read
    Create File  ${TEST_TEMPDIR}/test.yaml  {foo: bar, num: 0}
    ${data} =  Read YAML File  ${TEST_TEMPDIR}/test.yaml
    Dictionary Should Contain Item  ${data}  foo  bar
    Dictionary Should Contain Item  ${data}  num  ${0}
