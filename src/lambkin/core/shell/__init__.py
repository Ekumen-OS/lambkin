# Copyright 2026 Ekumen, Inc.
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

"""Shell subpackage for the lambkin SDK.

Provides shell abstractions for executing system commands within benchmarks.
Each shell implementation exposes the same interface, allowing benchmarks to
switch between dry-run and real execution without changing any benchmark code.
"""

from lambkin.core.shell.ros.launch import RosLaunchCommand

from .proxy import CommandError, ShellProxy

__all__ = [
    "ShellProxy",
    "CommandError",
    "RosLaunchCommand",
]
