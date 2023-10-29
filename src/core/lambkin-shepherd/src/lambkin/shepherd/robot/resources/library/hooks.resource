# Copyright 2022 Ekumen, Inc.
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
Documentation     Machinery for benchmark hooks configuration

Library           lambkin.shepherd.robot.api.hooks.SuiteExtensionPoints
Library           lambkin.shepherd.robot.api.hooks.TestExtensionPoints


*** Keywords ***
Configure ${some} benchmark suite hooks
    [Documentation]
    ...  Configures hooks for some benchmark. Stubs will be used for
    ...  hooks that were not overriden for the given benchmark.
    [Tags]  imperative
    Should Not Be In Test Scope
    ...  Cannot configure ${some} benchmark hooks during a test.
    Register Keyword On Current Suite  To be run before benchmark suite setup
    ...                                Before ${some} benchmark suite setup
    Register Keyword On Current Suite  To be run for benchmark suite setup
    ...                                For ${some} benchmark suite setup
    Register Keyword On Current Suite  To be run after benchmark suite setup
    ...                                After ${some} benchmark suite setup
    Register Keyword On Current Suite  To be run before benchmark suite teardown
    ...                                Before ${some} benchmark suite teardown
    Register Keyword On Current Suite  To be run for benchmark suite teardown
    ...                                For ${some} benchmark suite teardown
    Register Keyword On Current Suite  To be run after benchmark suite teardown
    ...                                After ${some} benchmark suite teardown

Configure ${some} benchmark case hooks
    [Documentation]
    ...  Configures hooks for some benchmark case. Stubs will be used
    ...  for hooks that were not overriden for the given benchmark case.
    [Tags]  imperative
    Should Be In Test Scope
    ...  Cannot configure ${some} benchmark case hooks outside a test.
    Register Keyword On Current Test  To be run before benchmark case execution
    ...                               Before ${some} benchmark case execution
    Register Keyword On Current Test  To be run before benchmark case iteration
    ...                               Before ${some} benchmark case iteration
    Register Keyword On Current Test  To be run before benchmark case rig bringup
    ...                               Before ${some} benchmark case rig bringup
    Register Keyword On Current Test  To be run for benchmark case rig bringup
    ...                               For ${some} benchmark case rig bringup
    Register Keyword On Current Test  To be run on benchmark case rig bringup
    ...                               On ${some} benchmark case rig bringup
    Register Keyword On Current Test  To be run after benchmark case rig bringup
    ...                               After ${some} benchmark case rig bringup
    Register Keyword On Current Test  To be run per benchmark case iteration
    ...                               Per ${some} benchmark case iteration
    Register Keyword On Current Test  To be run before benchmark case rig shutdown
    ...                               Before ${some} benchmark case rig shutdown
    Register Keyword On Current Test  To be run for benchmark case rig shutdown
    ...                               For ${some} benchmark case rig shutdown
    Register Keyword On Current Test  To be run on benchmark case rig shutdown
    ...                               On ${some} benchmark case rig shutdown
    Register Keyword On Current Test  To be run after benchmark case rig shutdown
    ...                               After ${some} benchmark case rig shutdown
    Register Keyword On Current Test  To be run after benchmark case iteration
    ...                               After ${some} benchmark case iteration
    Register Keyword On Current Test  To be run after benchmark case execution
    ...                               After ${some} benchmark case execution

Clear ${some} benchmark case hooks
    [Documentation]  Clears previously registered hooks for some benchmark case.
    [Tags]  imperative
    Should Be In Test Scope  Cannot clear ${some} benchmark case hooks outside a test.
    Clear Keywords Registered On Current Test  To be run before benchmark case execution
    Clear Keywords Registered On Current Test  To be run before benchmark case iteration
    Clear Keywords Registered On Current Test  To be run before benchmark case rig bringup
    Clear Keywords Registered On Current Test  To be run for benchmark case rig bringup
    Clear Keywords Registered On Current Test  To be run on benchmark case rig bringup
    Clear Keywords Registered On Current Test  To be run after benchmark case rig bringup
    Clear Keywords Registered On Current Test  To be run per benchmark case iteration
    Clear Keywords Registered On Current Test  To be run before benchmark case rig shutdown
    Clear Keywords Registered On Current Test  To be run for benchmark case rig shutdown
    Clear Keywords Registered On Current Test  To be run on benchmark case rig shutdown
    Clear Keywords Registered On Current Test  To be run after benchmark case rig shutdown
    Clear Keywords Registered On Current Test  To be run after benchmark case iteration
    Clear Keywords Registered On Current Test  To be run after benchmark case execution

### Stubs ###

Before ${some} benchmark suite setup
    [Documentation]
    ...  Stub for hooks that are to be run before benchmark suite setup.
    ...
    ...  Specific benchmark descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

For ${some} benchmark suite setup
    [Documentation]
    ...  Stub for hooks that are to be run for benchmark suite setup.
    ...  That is, to implement the setup functionality.
    ...
    ...  Specific benchmark descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

After ${some} benchmark suite setup
    [Documentation]
    ...  Stub for hooks that are to be run after benchmark setup.
    ...
    ...  Specific benchmark descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

Before ${some} benchmark suite teardown
    [Documentation]
    ...  Stub for hooks that are to be run before benchmark suite teardown.
    ...
    ...  Specific benchmark descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

For ${some} benchmark suite teardown
    [Documentation]
    ...  Stub for hooks that are to be run for benchmark suite teardown. That
    ...  is, to implement the teardown functionality. This occurs once per
    ...  suite, provided the corresponding benchmark suite teardown keyword
    ...  is used.
    ...
    ...  Specific benchmark descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

After ${some} benchmark suite teardown
    [Documentation]
    ...  Stub for hooks that are to be run after benchmark suite teardown. This
    ...  occurs once per suite, provided the corresponding benchmark suite teardown
    ...  keyword is used.
    ...
    ...  Specific benchmark descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

Before ${some} benchmark case execution
    [Documentation]
    ...  Stub for hooks that are to be run before executing each benchmark case.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

Before ${some} benchmark case iteration
    [Documentation]
    ...  Stub for hooks that are to be run before each benchmark case iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

Before ${some} benchmark case rig bringup
    [Documentation]
    ...  Stub for hooks that are to be run before the benchmark case rig
    ...  is brought up, once per iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

For ${some} benchmark case rig bringup
    [Documentation]
    ...  Stub for hooks that are to be run for benchmark case rig bring
    ...  up, once per iteration. That is, to instrument the rig.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

On ${some} benchmark case rig bringup
    [Documentation]
    ...  Stub for hooks that are to be run during benchmark case rig bring
    ...  up, once per iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

After ${some} benchmark case rig bringup
    [Documentation]
    ...  Stub for hooks that are to be run after the benchmark case rig
    ...  has been brought up, once per iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

Per ${some} benchmark case iteration
    [Documentation]
    ...  Stub for hooks that are to be run per benchmark case iteration.
    ...  That is, to implement the benchmark case itself.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

Before ${some} benchmark case rig shutdown
    [Documentation]
    ...  Stub for hooks that are to be run before the benchmark case rig
    ...  is shut down, once per iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

For ${some} benchmark case rig shutdown
    [Documentation]
    ...  Stub for hooks that are to be run for benchmark case rig shutdown,
    ...  once per iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

On ${some} benchmark case rig shutdown
    [Documentation]
    ...  Stub for hooks that are to be run while the benchmark case rig
    ...  is shut down, once per iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

After ${some} benchmark case rig shutdown
    [Documentation]
    ...  Stub for hooks that are to be run after the benchmark case rig
    ...  has been shut down, once per iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

After ${some} benchmark case iteration
    [Documentation]
    ...  Stub for hooks that are to be run after each benchmark case iteration.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

After ${some} benchmark case execution
    [Documentation]
    ...  Stub for hooks that are to be run after each benchmark case execution.
    ...
    ...  Specific benchmark case descriptions may override this stub.
    [Tags]  declarative  stub
    No Operation

${some} benchmark suite
    [Documentation]
    ...  Stub for benchmark suite definition. This is used once per suite for
    ...  configuration, provided the corresponding benchmark setup keyword
    ...  is used.
    [Tags]  declarative  stub
    No Operation

${some} benchmark case
    [Documentation]
    ...  Stub for benchmark case definition. This is used once per test case
    ...  for configuration, provided the corresponding benchmark case keyword
    ...  is used as test template.
    [Tags]  declarative  stub
    No Operation
