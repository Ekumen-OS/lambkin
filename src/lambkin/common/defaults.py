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

"""Default values shared across the lambkin SDK."""

from typing import Final

# Default name for the benchmarks output directory.
BENCHMARKS_DIRNAME: Final[str] = "results"

# If True, commands are logged but not executed.
DRY_RUN: Final[bool] = False

# Seconds to wait after SIGTERM before sending SIGKILL.
SIGTERM_GRACE_PERIOD: Final[float] = 3.0

# Seconds to wait after SIGKILL before giving up.
SIGKILL_GRACE_PERIOD: Final[float] = 5.0

# Polling interval in seconds when waiting for cgroup processes to exit.
CGROUP_POLL_INTERVAL: Final[float] = 0.05

# Sampling interval in seconds for the ResourceMonitor.
RESOURCE_MONITOR_INTERVAL: Final[float] = 1.0

# Where to route process output. Either 'file' or 'console'.
LOG_OUTPUT: Final[str] = "file"

# Default log level for lambkin SDK output.
LOG_LEVEL: Final[str] = "info"
