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
Documentation
...    Base template descriptions for most (if not all) benchmarks.
...
...    These descriptions provide baseline functionality and conventions
...    for all benchmarks, from report generation to consistent filesystem
...    layouts, and thus are likely to be extended by most (if not all).

Library    DateTime

Resource   lambkin/shepherd/robot/resources/library/core.resource
Resource   lambkin/shepherd/robot/resources/library/filesystem.resource
Resource   lambkin/shepherd/robot/resources/library/sphinx.resource


*** Variables ***
${BENCHMARK.SOURCE}  ${SUITE_SOURCE}
${BENCHMARK.ROOT}      ${OUTPUT_DIR}
${BENCHMARK.CASE.DELAY}      ${null}
${BENCHMARK.CASE.ITERATIONS}    ${1}


*** Keywords ***
Generates ${format} report from ${template_path} template
    [Documentation]
    ...  Specifies the Sphinx project template and output format
    ...  that some benchmark  will use to generate a report.
    ...  Relative template project paths will be resolved at the
    ...  working directory. See ``Sphinx`` documentation for a
    ...  list of supported output formats.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.REPORT.TEMPLATE
    ...  Report template already set to ${BENCHMARK.REPORT.TEMPLATE}
    Variable Should Not Exist  $BENCHMARK.REPORT.FORMAT
    ...  Report type already set to ${BENCHMARK.REPORT.FORMAT}
    Variable Should Not Exist  $BENCHMARK.REPORT.PATH
    ...  Report path already set to ${BENCHMARK.REPORT.PATH}
    Set Suite Variable  $BENCHMARK.REPORT.TEMPLATE  ${template_path}
    Set Suite Variable  $BENCHMARK.REPORT.FORMAT  ${format}
    ${path} =  Join Path  ${BENCHMARK.ROOT}  report
    Set Suite Variable  $BENCHMARK.REPORT.PATH  ${path}
    Reset Directory  ${path}

Uses ${count} iterations
    [Documentation]  States the number of iterations for a benchmark case.
    [Tags]  declarative
    ${count} =  Convert To Integer  ${count}
    Should Be True  ${count} > 0
    ...  Number of iterations must be a positive number (got ${count}).
    Set Test Variable  $BENCHMARK.CASE.ITERATIONS  ${count}

Holds ${delay}
    [Documentation]  States the number of iterations for a benchmark case.
    [Tags]  declarative
    ${delay} =  Convert Time  ${delay}
    Set Test Variable  $BENCHMARK.CASE.DELAY  ${delay}

### Hooks ###

Before base benchmark suite setup
    [Documentation]
    ...  See `Before ${some} benchmark suite setup` documentation.
    [Tags]  declarative  override
    ${path} =  Join Path  ${BENCHMARK.ROOT}  logs  setup
    Reset Process Logs  ${path}

For base benchmark suite setup
    [Documentation]
    ...  See `For ${some} benchmark suite setup` documentation.
    [Tags]  declarative  override
    Set Environment Variable  BENCHMARK_ROOT  ${BENCHMARK.ROOT}
    Set Environment Variable  BENCHMARK_SOURCE  ${BENCHMARK.SOURCE}
    ${path} =  Join Path  ${BENCHMARK.ROOT}  cases
    Set Suite Variable  $BENCHMARK.CASE.PATH  ${path}
    Reset Directory  ${path}

Before base benchmark suite teardown
    [Documentation]
    ...  See `Before ${some} benchmark suite teardown` documentation.
    [Tags]  declarative  override
    ${path} =  Join Path  ${BENCHMARK.ROOT}  logs  teardown
    Reset Process Logs  ${path}

For base benchmark suite teardown
    [Documentation]
    ...  See `For ${some} benchmark suite teardown` documentation.
    [Tags]  declarative  override
    ${reporting} =  Get Variable Value  $BENCHMARK.REPORT.TEMPLATE
    IF  "${reporting}" != "${null}"
        Build Sphinx Documentation
        ...  ${BENCHMARK.REPORT.PATH}  ${BENCHMARK.REPORT.TEMPLATE}
        ...  builder=${BENCHMARK.REPORT.FORMAT}
    END

Before base benchmark case execution
    [Documentation]
    ...  See `Before ${some} benchmark case execution` documentation.
    [Tags]  declarative  override

    Log To Console  message=
    ${parameters} =  Get Variable Value  $BENCHMARK.CASE.PARAMETERS  &{EMPTY}
    IF  ${parameters} != &{EMPTY}
        Log To Console  ...running ${BENCHMARK.CASE.TYPE} benchmark case for
        FOR  ${name}  ${value}  IN  &{BENCHMARK.CASE.PARAMETERS}
            Log To Console  message=\ \ \ \ ${name} \= ${value}
        END
    ELSE
        Log To Console  ...running ${BENCHMARK.CASE.TYPE} benchmark case
    END

    ${now} =  Get Current Date

    ${name} =  Convert To Snake Case  ${BENCHMARK.CASE.NAME}
    ${root} =  Join Path  ${BENCHMARK.CASE.PATH}  ${name}
    IF  ${BENCHMARK.CASE.VARIATION} == ${1}
      Reset Directory  ${root}

      ${path} =  Join Path  ${root}  metadata.json
      Write JSON File  ${path}  name=${BENCHMARK.CASE.NAME}
      ...  type=${BENCHMARK.CASE.TYPE}  starttime=${now}
    END

    ${variation} =  Convert To String  ${BENCHMARK.CASE.VARIATION}
    ${root} =  Join Path  ${root}  variations  ${variation}
    Set Test Variable  $BENCHMARK.CASE.VARIATION.PATH  ${root}
    Reset Directory  ${root}

    ${path} =  Join Path  ${root}  metadata.json
    ${parameters} =  Get Variable Value  $BENCHMARK.CASE.PARAMETERS  &{EMPTY}
    Write JSON File  ${path}  parameters=${parameters}  starttime=${now}

Before base benchmark case iteration
    [Documentation]
    ...  See `Before ${some} benchmark case iteration` documentation.
    [Tags]  override

    Log To Console  .  no_newline=yes
    ${now} =  Get Current Date

    ${iteration} =  Convert To String  ${BENCHMARK.CASE.ITERATION}
    ${path} =  Join Path
    ...  ${BENCHMARK.CASE.VARIATION.PATH}  iterations  ${iteration}
    Set Test Variable  $BENCHMARK.CASE.ITERATION.PATH  ${path}
    Reset Directory  ${path}

    ${path} =  Join Path  ${BENCHMARK.CASE.ITERATION.PATH}  metadata.json
    Write JSON File  ${path}  starttime=${now}

    ${path} =  Join Path  ${BENCHMARK.CASE.ITERATION.PATH}  logs
    Reset Process Logs  ${path}

Per base benchmark case iteration
    [Documentation]
    ...  See `Per ${some} benchmark case iteration` documentation.
    [Tags]  declarative  override
    IF  "${BENCHMARK.CASE.DELAY}" != "${null}"  Sleep  ${BENCHMARK.CASE.DELAY}
