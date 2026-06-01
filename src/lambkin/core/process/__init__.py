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

"""Process execution utilities for benchmark runs.

Provides background process management via cgroups v2, with automatic
cleanup and unexpected exit detection.
"""

from .background import BackgroundProcess, background
from .cgroup import (
    find_delegated_cgroup,
    kill_cgroup,
    make_cgroup,
    make_iteration_cgroup,
    make_process_cgroup,
    remove_cgroup,
)

__all__ = [
    "BackgroundProcess",
    "background",
    "find_delegated_cgroup",
    "make_cgroup",
    "kill_cgroup",
    "remove_cgroup",
    "make_iteration_cgroup",
    "make_process_cgroup",
]
