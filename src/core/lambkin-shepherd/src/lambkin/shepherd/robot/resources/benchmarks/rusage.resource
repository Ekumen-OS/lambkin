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

Resource    lambkin/shepherd/robot/resources/benchmarks/base.resource
Resource    lambkin/shepherd/robot/resources/library/timemory.resource

Library    Collections


*** Keywords ***
Generic resource usage benchmark suite
    [Documentation]
    ...  Generic, single process, resource usage benchmark suite description.
    [Tags]  descriptive
    Extends base benchmark suite

Generic resource usage benchmark case
    [Documentation]
    ...  Generic, multiple process, resource usage benchmark case description.
    [Tags]  descriptive
    Extends base benchmark case

Uses ${profiler} to sample ${process} performance
    [Documentation]
    ...  Specifies profiler and processes to be profiled
    ...  by some resource usage benchmark.
    ...
    ...  Note that in many cases this may not be carried out
    ...  automatically and additional steps may have be taken.
    ...  See profiling support documentation for further reference.
    [Tags]  declarative
    ${current_profiler} =  Get Variable Value  $BENCHMARK.RIG.PROFILER  ${profiler}
    IF  $current_profiler != $profiler
        Fatal Error  Profiler is already set to ${current_profiler}, it cannot be set to ${profiler}.
    END
    Set Non Local Variable  ${BENCHMARK.CASE.SCOPE}  $BENCHMARK.RIG.PROFILER  ${profiler}

    ${processes} =  Get Variable Value  $BENCHMARK.RIG.PROCESSES  @{EMPTY}
    Append To List  ${processes}  ${process}
    Set Non Local Variable  ${BENCHMARK.CASE.SCOPE}  $BENCHMARK.RIG.PROCESSES  ${processes}

### Hooks ###

Before generic resource usage benchmark case rig bringup
    [Documentation]
    ...  See `Before ${some} benchmark rig bringup` documentation.
    [Tags]  declarative  override
    Variable Should Exist  $BENCHMARK.RIG.PROFILER  Rig profiler not set
    Variable Should Exist  $BENCHMARK.RIG.PROCESSES  Rig processes not set
    FOR    ${process}    IN    @{BENCHMARK.RIG.PROCESSES}
        Run Keyword
        ...  Setup ${BENCHMARK.RIG.PROFILER}  ${process}
        ...  dir=${BENCHMARK.CASE.ITERATION.PATH}
    END

After generic resource usage benchmark case rig shutdown
    [Documentation]
    ...  See `After ${some} benchmark rig case shutdown` documentation.
    [Tags]  declarative  override
    Variable Should Exist  $BENCHMARK.RIG.PROFILER  Rig profiler not set
    Variable Should Exist  $BENCHMARK.RIG.PROCESSES  Rig processes not set
    FOR    ${process}    IN    @{BENCHMARK.RIG.PROCESSES}
        Run Keyword  Wait for ${BENCHMARK.RIG.PROFILER} output
        ...  ${process}  dir=${BENCHMARK.CASE.ITERATION.PATH}
    END
