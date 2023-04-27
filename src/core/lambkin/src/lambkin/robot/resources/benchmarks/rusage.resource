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
...    Template descriptions for resource usage benchmarks.
...
...    These descriptions ease process instrumentation for
...    resource usage assessments, largely delegating to
...    suitable tools.

Resource    lambkin/robot/resources/benchmarks/base.resource
Resource    lambkin/robot/resources/library/timemory.resource


*** Keywords ***
Generic resource usage benchmark suite
    [Documentation]
    ...  Generic, single process, resource usage benchmark suite description.
    [Tags]  descriptive
    Extends base benchmark suite

Generic resource usage benchmark case
    [Documentation]
    ...  Generic, single process, resource usage benchmark case description.
    [Tags]  descriptive
    Extends base benchmark case

Uses ${profiler} to sample ${process} performance
    [Documentation]
    ...  Specifies profiler and process to be profiled
    ...  by some resource usage benchmark.
    ...
    ...  Note that in many cases this may not be carried out
    ...  automatically and additional steps may have be taken.
    ...  See profiling support documentation for further reference.
    [Tags]  declarative
    Variable Should Not Exist  $BENCHMARK.RIG.PROFILER
    ...  Rig profiler already set to ${BENCHMARK.RIG.PROFILER}
    Variable Should Not Exist  $BENCHMARK.RIG.PROCESS
    ...  Rig process to be profiled already set to ${BENCHMARK.RIG.PROCESS}
    Set Test Variable  $BENCHMARK.RIG.PROFILER  ${profiler}
    Set Test Variable  $BENCHMARK.RIG.PROCESS  ${process}

### Hooks ###

Before generic resource usage benchmark case rig bringup
    [Documentation]
    ...  See `Before ${some} benchmark rig bringup` documentation.
    [Tags]  declarative  override
    Variable Should Exist  $BENCHMARK.RIG.PROFILER  Rig profiler not set
    Variable Should Exist  $BENCHMARK.RIG.PROCESS  Rig process not set
    Run Keyword
    ...  Setup ${BENCHMARK.RIG.PROFILER}  ${BENCHMARK.RIG.PROCESS}
    ...  dir=${BENCHMARK.CASE.ITERATION.PATH}

After generic resource usage benchmark case rig shutdown
    [Documentation]
    ...  See `After ${some} benchmark rig case shutdown` documentation.
    [Tags]  declarative  override
    Variable Should Exist  $BENCHMARK.RIG.PROFILER  Rig profiler not set
    Variable Should Exist  $BENCHMARK.RIG.PROCESS  Rig process not set
    Run Keyword  Wait for ${BENCHMARK.RIG.PROFILER} output
    ...  ${BENCHMARK.RIG.PROCESS}  dir=${BENCHMARK.CASE.ITERATION.PATH}
