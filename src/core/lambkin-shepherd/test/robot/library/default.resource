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
Documentation  Support for default test settings

Library   Process
Library   OperatingSystem
Library   tempfile

Library   lambkin.shepherd.robot.listeners.TracebackSupport


*** Keywords ***
Default Test Setup
    [Documentation]  Allocate test-specific temporary resources.
    ${TEST_TEMPDIR} =  tempfile.mkdtemp  dir=${TEMPDIR}
    Set Test Variable  $TEST_TEMPDIR  ${TEST_TEMPDIR}

Default Test Teardown
    [Documentation]  Clean up test-specific temporary resources.
    Terminate All Processes
    Run Keyword If Test Passed
    ...  Remove Directory  ${TEST_TEMPDIR}  recursive=yes
