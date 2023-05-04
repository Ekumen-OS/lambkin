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
Resource  lambkin/shepherd/robot/resources/library/core.resource

Library   Collections

Suite Setup     Setup parameterized benchmark suite
Suite Teardown  Teardown parameterized benchmark suite
Test Template   Run parameterized benchmark case for each ${a} and ${b} values


*** Test Cases ***
Parameterized benchmark using no expressions
    1  1
    1  2
    1  3
    2  1
    2  2
    2  3
    3  1
    3  2
    3  3


Parameterized benchmark using loop expressions
    FOR  ${i}  IN RANGE  1  3 + 1
        FOR  ${j}  IN RANGE  1  3 + 1
            ${i}  ${j}
        END
    END

Parameterized benchmark using generator expressions
    ${{range(1, 3 + 1)}}  ${{range(1, 3 + 1)}}


*** Keywords ***
Per parameterized benchmark case iteration
    Should Be Equal  ${BENCHMARK.CASE.TYPE}  parameterized
    Dictionary Should Contain Item  ${BENCHMARK.CASE.PARAMETERS}  a  ${a}
    Dictionary Should Contain Item  ${BENCHMARK.CASE.PARAMETERS}  b  ${b}
    ${variation} =  Evaluate  (${a} - 1) * 3 + ${b}
    Should Be Equal  ${variation}  ${BENCHMARK.CASE.VARIATION}
