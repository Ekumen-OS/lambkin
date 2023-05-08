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
Documentation  Resource usage benchmark implementation tests
Resource  lambkin/shepherd/robot/resources/benchmarks/rusage.resource

Resource  testing.resource

Library   OperatingSystem
Library   Process

Suite Setup     Setup performance benchmark suite
Suite Teardown  Teardown performance benchmark suite


*** Test Cases ***
RUsage benchmark functionality is correct
    Run performance benchmark case once


*** Keywords ***
Performance benchmark suite
    Extends testing benchmark suite
    Extends generic resource usage benchmark suite

Performance benchmark case
    Extends generic resource usage benchmark case
    Uses timemory-timem to sample python3 performance
    Uses 3 iterations

For performance benchmark case rig bringup
    Start Process
    ...  $python3_PREFIX python3 -c 'import time; time.sleep(1)'
    ...  shell=yes  alias=PYTHON3

For performance benchmark case rig shutdown
    Wait For Process  PYTHON3

After performance benchmark case iteration
    File Should Not Exist  ${BENCHMARK.CASE.ITERATION.PATH}/.timem.json
    File Should Exist  ${BENCHMARK.CASE.ITERATION.PATH}/python3.timem.json
