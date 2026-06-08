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

"""Unit tests for the utility functions in lambkin.utils."""

import pytest

from lambkin.utils import format_elapsed_time


@pytest.mark.parametrize(
    "seconds,expected",
    [
        (0.0, "0.0000s"),
        (0.0023, "0.0023s"),
        (59.9999, "59.9999s"),
        (60.0, "1m 00s"),
        (103.0, "1m 43s"),
        (3600.0, "1h 00m 00s"),
        (3661.0, "1h 01m 01s"),
        (86400.0, "1d 00h 00m 00s"),
        (2 * 86400 + 3 * 3600 + 15 * 60 + 7, "2d 03h 15m 07s"),
    ],
)
def test_format_elapsed_time(seconds, expected):
    """_format_elapsed_time formats durations correctly across all time scales."""
    assert format_elapsed_time(seconds) == expected
