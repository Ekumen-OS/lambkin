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
Documentation  Core APIs for benchmark description and instrumentation.

Resource  lambkin/robot/resources/library/hooks.resource
Resource  lambkin/robot/resources/library/process.resource

Library   lambkin.robot.listeners.ParameterizationSupport
Library   lambkin.robot.listeners.TracebackSupport


*** Keywords ***
Configure ${some} benchmark suite
    [Documentation]
    ...  Configures some benchmark suite based on its description.
    [Tags]  imperative
    ${definition} =  Set Variable  ${some} benchmark suite
    Run Keyword  ${definition}

Configure ${some} benchmark case
    [Documentation]
    ...  Configures some benchmark case based on its description.
    [Tags]  imperative
    ${definition} =  Set Variable  ${some} benchmark case
    Run Keyword  ${definition}

Extends ${some} benchmark suite
    [Documentation]
    ...  States that some benchmark suite description
    ...  extends (and thus reuses) another.
    [Tags]  declarative
    ${extensions} =  Get Variable Value  $BENCHMARK.SUITE.EXTENSIONS  @{EMPTY}
    IF  "${some}" not in ${extensions}
        # Avoid the diamond problem, apply extension once
        Configure ${some} benchmark suite
        Configure ${some} benchmark suite hooks

        ${extensions} =  Get Variable Value  $BENCHMARK.SUITE.EXTENSIONS  @{EMPTY}
        Set Suite Variable  @BENCHMARK.SUITE.EXTENSIONS  ${some}  @{extensions}
    END

Extends ${some} benchmark case
    [Documentation]
    ...  States that some benchmark case description
    ...  extends (and thus reuses) another.
    [Tags]  declarative
    ${extensions} =  Get Variable Value  $BENCHMARK.CASE.EXTENSIONS  @{EMPTY}
    IF  "${some}" not in ${extensions}
        # Avoid the diamond problem, apply extension once
        Configure ${some} benchmark case
        Configure ${some} benchmark case hooks

        ${extensions} =  Get Variable Value  $BENCHMARK.CASE.EXTENSIONS  @{EMPTY}
        Set Test Variable  @BENCHMARK.CASE.EXTENSIONS  ${some}  @{extensions}
    END

Setup ${some} benchmark suite
    [Documentation]
    ...  Sets up some benchmark suite as prescribed.
    ...
    ...  Typically used as suite setup.
    [Tags]  imperative
    Configure ${some} benchmark suite
    Configure ${some} benchmark suite hooks
    Run Keywords Registered On Current Suite  To be run before benchmark suite setup
    Run Keywords Registered On Current Suite  To be run for benchmark suite setup
    Run Keywords Registered On Current Suite  To be run after benchmark suite setup

Teardown ${some} benchmark suite
    [Documentation]
    ...  Tears down some benchmark suite as prescribed.
    ...
    ...  Typically used as suite teardown.
    [Tags]  imperative
    Run Keywords Registered On Current Suite  To be run before benchmark suite teardown
    Run Keywords Registered On Current Suite  To be run for benchmark suite teardown
    Run Keywords Registered On Current Suite  To be run after benchmark suite teardown

Execute ${some} benchmark case
    [Documentation]
    ...  Executes some benchmark case as configured.
    ...
    ...  To do this, it relies almost completely on benchmark hook overrides.
    [Tags]  imperative
    Run Keywords Registered On Current Test  To be run before benchmark case execution
    ${iterations} =  Get Variable Value  $BENCHMARK.CASE.ITERATIONS  1
    FOR  ${iteration}  IN RANGE  1  ${iterations} + 1
        Set Test Variable  $BENCHMARK.CASE.ITERATION  ${iteration}
        Run Keywords Registered On Current Test  To be run before benchmark case iteration
        Run Keywords Registered On Current Test  To be run before benchmark case rig bringup
        Run Keywords Registered On Current Test  To be run for benchmark case rig bringup
        Run Keywords Registered On Current Test  To be run on benchmark case rig bringup
        Run Keywords Registered On Current Test  To be run after benchmark case rig bringup
        Run Keywords Registered On Current Test  To be run per benchmark case iteration
        Run Keywords Registered On Current Test  To be run before benchmark case rig shutdown
        Run Keywords Registered On Current Test  To be run for benchmark case rig shutdown
        Run Keywords Registered On Current Test  To be run on benchmark case rig shutdown
        Run Keywords Registered On Current Test  To be run after benchmark case rig shutdown
        Run Keywords Registered On Current Test  To be run after benchmark case iteration
    END
    Run Keywords Registered On Current Test  To be run after benchmark case execution

Run ${some} benchmark case for ${parameterization}
    [Documentation]
    ...  Configures and executes a benchmark case as parameterized.
    ...
    ...  Typically used as a test template, in which case argument names
    ...  may be embedded in the free-form suffix e.g. '${arg1}, ${arg2}...'.
    ...  These arguments will be put in scope for all benchmark hook
    ...  implementations.
    [Tags]  imperative
    Flush Traceback  # split tracebacks for data driven benchmarks
    ${parameters} =  Extract Template Arguments
    ...  Run ${some} benchmark case for ${parameterization}
    Set Test Variable  $BENCHMARK.CASE.PARAMETERS  ${parameters}
    FOR  ${name}  ${value}  IN  &{parameters}
        # robocop: disable=non-local-variables-should-be-uppercase
        Set Test Variable  $${name}  ${value}
    END
    Run ${some} benchmark case once
    Set Test Variable  $BENCHMARK.CASE.PARAMETERS  &{EMPTY}

Run ${some} benchmark case once
    [Documentation]
    ...  Configures and executes a benchmark case.
    ...
    ...  Each invocation within the same test scope will be
    ...  tracked as a separate variation (starting from 0).
    ...  Configuration only takes place on first run for a
    ...  given scope.
    [Teardown]  Terminate All Processes
    [Tags]  imperative
    Increment Variable In Parent Scope  $BENCHMARK.CASE.VARIATION  ${0}
    IF  ${BENCHMARK.CASE.VARIATION} == ${1}
        Set Test Variable  $BENCHMARK.CASE.TYPE  ${some}
        Set Test Variable  $BENCHMARK.CASE.NAME  ${TEST_NAME}
        Configure ${some} benchmark case
        Configure ${some} benchmark case hooks
    END
    Execute ${some} benchmark case
