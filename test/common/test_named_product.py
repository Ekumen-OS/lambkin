#!/usr/bin/env python3

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

# Membership test


@pytest.mark.parametrize(
    "kwargs, expected_combinations",
    [
        (
            {"sensor_model": ["likelihood_field", "beam"], "num_particles": [1, 10]},
            [
                {"sensor_model": "likelihood_field", "num_particles": 1},
                {"sensor_model": "likelihood_field", "num_particles": 10},
                {"sensor_model": "beam", "num_particles": 1},
                {"sensor_model": "beam", "num_particles": 10},
            ],
        ),
        (
            {"sensor_model": ["likelihood_field", "beam"]},
            [
                {"sensor_model": "likelihood_field"},
                {"sensor_model": "beam"},
            ],
        ),
        (
            {},
            [{}],
        ),
        (
            {"sensor_model": [], "num_particles": [1]},
            [],
        ),
    ],
)
def test_named_product_combinations(kwargs, expected_combinations):
    """named_product contains exactly the expected combination dicts."""
    result = named_product(**kwargs)
    assert result == expected_combinations


# Length test


@pytest.mark.parametrize(
    "kwargs, expected_len",
    [
        (
            {"sensor_model": ["likelihood_field", "beam"], "num_particles": [1, 10]},
            4,
        ),
        (
            {
                "sensor_model": ["likelihood_field", "beam"],
                "num_particles": [1, 10],
                "rate": [10, 100],
            },
            8,
        ),
        (
            {"sensor_model": ["likelihood_field", "beam"]},
            2,
        ),
        (
            {"sensor_model": [], "num_particles": [1]},
            0,
        ),
        (
            {},
            1,
        ),
    ],
)
def test_named_product_length(kwargs, expected_len):
    """named_product returns the correct number of combinations (cartesian product)."""
    result = named_product(**kwargs)
    assert len(result) == expected_len
