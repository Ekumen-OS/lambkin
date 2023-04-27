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
Documentation  Utilities to work with files and directories.

Library  OperatingSystem

Library  json
Library  yaml

Library  lambkin.robot.api.filesystem


*** Keywords ***
Skip Unless Executable Exists
    [Documentation]  Skips test case unless executable can be found in $PATH.
    [Arguments]  ${executable}  ${msg}=${null}
    [Tags]  imperative
    ${msg} =  Set Variable If  ${msg} == ${null}  ${executable} not found
    ${path} =  Find Executable Path  ${executable}
    Skip If  "${path}" == "${None}"  msg=${msg}

Skip Unless Executables Exist
    [Documentation]  Skips test case unless executables can be found in $PATH.
    [Arguments]  @{executables}  ${msg}=${null}
    [Tags]  imperative
    FOR  ${executable}  IN  @{executables}
        Skip Unless Executable Exists  ${executable}  ${msg}
    END

Reset Directory
    [Documentation]  Ensures directory exists and it is empty.
    ...  Intermediate directories are created as needed.
    [Arguments]  ${dir}
    [Tags]  imperative
    Create Directory  ${dir}
    Empty Directory  ${dir}

Write JSON File
    [Documentation]
    ...  Writes a JSON file using keyword arguments as key-value pairs.
    [Arguments]  ${path}  ${indent}=${2}  &{dict}
    [Tags]  imperative
    ${content} =  json.dumps  ${dict}  indent=${indent}
    Create File  ${path}  ${content}

Read YAML File
    [Documentation]  Reads a YAML file as a dictionary.
    [Arguments]  ${path}
    [Tags]  imperative
    ${content} =  Get File  ${path}
    ${object} =  yaml.safe_load  ${content}
    RETURN  ${object}
