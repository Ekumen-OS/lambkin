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
Documentation  Base benchmark implementation tests
Resource  lambkin/robot/resources/benchmarks/base.resource

Resource  testing.resource

Suite Setup     Setup basic benchmark suite
Suite Teardown  Teardown basic benchmark suite


*** Test Cases ***
Base benchmark functionality is correct
    Run basic benchmark case once


*** Keywords ***
Basic benchmark suite
    Extends base benchmark suite
    Extends testing benchmark suite

Basic benchmark case
    Extends base benchmark case
    Uses 3 iterations

Per basic benchmark case iteration
    Should Be Equal  ${BENCHMARK.CASE.ITERATIONS}  ${3}
