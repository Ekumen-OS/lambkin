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

"""General-purpose utilities for the lambkin SDK.

This module provides standalone helper functions with no dependencies on
lambkin internals. Functions here are reusable across the SDK and user
benchmarks alike.
"""


def format_elapsed_time(seconds: float) -> str:
    """Format an elapsed time into a human-readable string.

    Scales the output unit to the duration so the result is always easy to
    read at a glance, from sub-second runs to multi-day sweeps:

    - Under 60 s:  ``'0.0023s'``
    - Under 1 h:   ``'45m 03s'``
    - Under 1 day: ``'2h 15m 07s'``
    - 1 day or more: ``'2d 03h 15m 07s'``

    Sub-second precision is kept only for runs under 60 seconds; longer
    durations are truncated to whole seconds.

    Args:
        seconds (float): Elapsed time in seconds, as returned by ``time.monotonic()``.

    Returns:
        A human-readable elapsed time string.
    """
    if seconds < 60:
        return f"{seconds:.4f}s"
    total = int(seconds)
    d, remainder = divmod(total, 86400)
    h, remainder = divmod(remainder, 3600)
    m, s = divmod(remainder, 60)
    if d > 0:
        return f"{d}d {h:02d}h {m:02d}m {s:02d}s"
    if h > 0:
        return f"{h}h {m:02d}m {s:02d}s"
    return f"{m}m {s:02d}s"
