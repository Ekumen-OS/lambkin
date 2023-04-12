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
Documentation  Benchmark sequencing tests
Resource  lambkin/robot/resources/library/core.resource

Resource  lambkin/robot/resources/library/utilities.resource

Suite Setup     Setup sequenced benchmark suite
Suite Teardown  Teardown sequenced benchmark suite


*** Variables ***
${SETUP}       ${False}
${EXECUTING}   ${False}
${RIG_ONLINE}  ${False}


*** Test Cases ***
Benchmark hooks are correctly sequenced
    Run sequenced benchmark case once


*** Keywords ***
Before sequenced benchmark suite setup
    Should Not Be In Test Scope
    Should Not Be True  ${SETUP}

For sequenced benchmark suite setup
    Should Not Be In Test Scope
    Set Suite Variable  $SETUP  ${True}

After sequenced benchmark suite setup
    Should Not Be In Test Scope
    Should Be True  ${SETUP}
    Should Not Be True  ${EXECUTING}

Before sequenced benchmark suite teardown
    Should Not Be In Test Scope
    Should Not Be True  ${EXECUTING}
    Should Be True  ${SETUP}

For sequenced benchmark suite teardown
    Should Not Be In Test Scope
    Set Suite Variable  $SETUP  ${False}

After sequenced benchmark suite teardown
    Should Not Be In Test Scope
    Should Not Be True  ${SETUP}

Before sequenced benchmark case execution
    Should Be In Test Scope
    Should Not Be True  ${EXECUTING}
    Set Suite Variable  $EXECUTING  ${True}

Before sequenced benchmark case iteration
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Increment Variable In Parent Scope  $ITERATION  ${0}
    Should Be Equal  ${BENCHMARK.CASE.ITERATION}  ${ITERATION}

Before sequenced benchmark case rig bringup
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Not Be True  ${RIG_ONLINE}

For sequenced benchmark case rig bringup
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Set Test Variable  $RIG_ONLINE  ${True}

On sequenced benchmark case rig bringup
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Be True  ${RIG_ONLINE}

After sequenced benchmark case rig bringup
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Be True  ${RIG_ONLINE}

Per sequenced benchmark case iteration
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Be True  ${RIG_ONLINE}

Before sequenced benchmark case rig shutdown
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Be True  ${RIG_ONLINE}

For sequenced benchmark case rig shutdown
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Be True  ${RIG_ONLINE}
    Set Test Variable  $RIG_ONLINE  ${False}

On sequenced benchmark case rig shutdown
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Not Be True  ${RIG_ONLINE}

After sequenced benchmark case rig shutdown
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Not Be True  ${RIG_ONLINE}

After sequenced benchmark case iteration
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Should Not Be True  ${RIG_ONLINE}
    Should Be Equal  ${BENCHMARK.CASE.ITERATION}  ${ITERATION}

After sequenced benchmark case execution
    Should Be In Test Scope
    Should Be True  ${EXECUTING}
    Set Suite Variable  $EXECUTING  ${False}
