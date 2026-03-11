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

"""Unit tests for the named_product function in lambkin.common."""

import pytest

from lambkin.common.named_product import named_product


@pytest.mark.parametrize(
    "parameters, expected_combinations , expected_len",
    [
        (  # Two parameters with 2 values each -> 2×2 = 4 combinations
            {"sensor_model": ["likelihood_field", "beam"], "num_particles": [1, 10]},
            [
                {"sensor_model": "likelihood_field", "num_particles": 1},
                {"sensor_model": "likelihood_field", "num_particles": 10},
                {"sensor_model": "beam", "num_particles": 1},
                {"sensor_model": "beam", "num_particles": 10},
            ],
            4,
        ),
        (  # Single parameter with 2 values -> 2 combinations
            {"sensor_model": ["likelihood_field", "beam"]},
            [
                {"sensor_model": "likelihood_field"},
                {"sensor_model": "beam"},
            ],
            2,
        ),
        (  # No parameters at all -> 1 combination: a single empty dict
            {},
            [{}],
            1,
        ),
        (  # One parameter with an empty list -> no combinations can be formed
            # TODO(teresa-ortega): if the user introduce a empty list -> Exception
            {"sensor_model": [], "num_particles": [1]},
            [],
            0,
        ),
    ],
)
def test_named_product_combinations(parameters, expected_combinations, expected_len):
    """named_product returns the correct number and combinations."""
    result = named_product(**parameters)
    assert result == expected_combinations
    assert len(result) == expected_len
