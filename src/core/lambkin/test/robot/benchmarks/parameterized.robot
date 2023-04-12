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
Documentation  Benchmark parameterization support tests
Resource  lambkin/robot/resources/library/core.resource

Library   Collections

Suite Setup     Setup parameterized benchmark suite
Suite Teardown  Teardown parameterized benchmark suite
Test Template   Run parameterized benchmark case for each ${parameter}


*** Test Cases ***
Parameterized benchmark using no expressions
    value1
    value2
    value3
    value4
    value5
    value6
    value7
    value8
    value9
    value10

Parameterized benchmark using loop expressions
    FOR  ${index}  IN RANGE  1  10 + 1
        value${index}
    END

Parameterized benchmark using generator expressions  value${{range(1, 10 + 1)}}


*** Keywords ***
Per parameterized benchmark case iteration
    Should Be Equal  ${BENCHMARK.CASE.TYPE}  parameterized
    Dictionary Should Contain Item  ${BENCHMARK.CASE.PARAMETERS}
    ...  parameter  value${BENCHMARK.CASE.VARIATION}
    Should Be Equal  ${parameter}  value${BENCHMARK.CASE.VARIATION}
