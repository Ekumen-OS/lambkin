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
Documentation  An augmented version of the builtin Process library.

Library   OperatingSystem
Library   Process

Resource  lambkin/robot/resources/library/utilities.resource


*** Keywords ***
Reset Process Logs
    [Documentation]  Resets processes' logging location.
    [Arguments]  ${dir}
    [Tags]  imperative
    Create Directory  ${dir}
    Set Variable In Parent Scope  $PROCESS_LOG_DIR  ${dir}
    Set Variable In Parent Scope  $PROCESS_COUNTER  ${0}

Start Managed Process
    [Documentation]
    ...  Starts a process, logging its output.
    ...  Builds on the `Start Process` keyword.
    ...  Check its documentation for further reference.
    [Arguments]  ${command}  @{args}  &{kwargs}
    [Tags]  imperative
    ${log_dir} =  Get Variable Value  $PROCESS_LOG_DIR  ${TEMPDIR}
    ${id} =  Increment Variable In Parent Scope  $PROCESS_COUNTER
    ${process} =  Start Process  ${command}  @{args}
    ...           stdout=${log_dir}/${command}-${id}-stdout.log
    ...           stderr=${log_dir}/${command}-${id}-stderr.log
    ...           &{kwargs}
    RETURN  ${process}

Wait For Managed Process
    [Documentation]
    ...  Wait for a process to complete.
    ...  Builds on the `Wait For Process` keyword.
    ...  Check its documentation for further reference.
    [Arguments]  ${handle}=${null}  &{kwargs}
    [Tags]  imperative
    ${process} =  Get Process Object  ${handle}
    ${result} =  Wait For Process  ${handle}  &{kwargs}
    IF  '${result}' != '${null}'
        IF  ${result.rc} != 0
            Log To Console  ${process.args}[0] process exit with return code ${result.rc}  stream=STDERR
            Log To Console  You may check its output in ${result.stdout_path} and ${result.stderr_path}  stream=STDERR
        END
    END
    RETURN  ${result}

Terminate Managed Process
    [Documentation]
    ...  Terminates a process and checks its outcome, by escalating from
    ...  interrupt to termination to kill, 30 seconds apart each.
    ...  Builds on `Wait For Managed Process` and `Terminate Process` keywords.
    ...  Check their documentation for further reference.
    [Arguments]  ${handle}=${null}  &{kwargs}
    [Tags]  imperative
    ${process} =  Get Process Object  ${handle}
    Send Signal To Process  INT  ${handle}
    ${result} =  Wait For Managed Process  ${handle}  timeout=30s
    IF  '${result}' == '${null}'
        ${result} =  Terminate Process  ${handle}  &{kwargs}
        IF  ${result.rc} != 0
            Log To Console  ${process.args}[0] process exit with return code ${result.rc}  stream=STDERR
            Log To Console  You may check its output in ${result.stdout_path} and ${result.stderr_path}  stream=STDERR
        END
    END
    RETURN  ${result}

Run Managed Process
    [Documentation]
    ...  Runs a process, logging its output and checking its outcome.
    ...  Builds on the `Run Process` keyword.
    ...  Check its documentation for further reference.
    [Arguments]  ${command}  @{args}  &{kwargs}
    [Tags]  imperative
    ${log_dir} =  Get Variable Value  $PROCESS_LOG_DIR  ${TEMPDIR}
    ${id} =  Increment Variable In Parent Scope  $PROCESS_COUNTER
    ${result} =  Run Process  ${command}  @{args}
    ...          stdout=${log_dir}/${command}-${id}-stdout.log
    ...          stderr=${log_dir}/${command}-${id}-stderr.log  &{kwargs}
    IF  ${result.rc} != 0
        Log To Console  ${\n}  stream=STDERR
        Log To Console  ${command} process exit with return code ${result.rc}  stream=STDERR
        Log To Console  You may check its output in ${result.stdout_path} and ${result.stderr_path}  stream=STDERR
    END
    RETURN  ${result}
