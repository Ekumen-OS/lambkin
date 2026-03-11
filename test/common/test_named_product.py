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

import unittest

from lambkin.common.named_product import named_product


class TestNamedProduct(unittest.TestCase):
    """Test suite for the named_product utility function."""

    def test_two_parameters_correct_length(self):
        """Tests if named_product generates the correct number of combinations.

        It verifies that two parameters with 2 values each produce
        4 combinations (2 × 2).
        """
        result = named_product(
            sensor_model=["likelihood_field", "beam"], num_particles=[1, 10]
        )
        self.assertEqual(len(result), 4)

    def test_two_parameters_correct_combinations(self):
        """Tests if named_product generates all expected combinations.

        It verifies that every possible combination of the given
        parameters is present in the result.
        """
        result = named_product(
            sensor_model=["likelihood_field", "beam"], num_particles=[1, 10]
        )
        self.assertIn({"sensor_model": "likelihood_field", "num_particles": 1}, result)
        self.assertIn({"sensor_model": "likelihood_field", "num_particles": 10}, result)
        self.assertIn({"sensor_model": "beam", "num_particles": 1}, result)
        self.assertIn({"sensor_model": "beam", "num_particles": 10}, result)

    def test_three_parameters_correct_length(self):
        """Tests if named_product scales correctly with three parameters.

        It verifies that three parameters with 2 values each produce
        8 combinations (2³).
        """
        result = named_product(
            sensor_model=["likelihood_field", "beam"],
            num_particles=[1, 10],
            rate=[10, 100],
        )
        self.assertEqual(len(result), 8)

    def test_single_parameter(self):
        """Tests if named_product works correctly with a single parameter.

        It verifies that a single parameter returns one dict per value.
        """
        result = named_product(sensor_model=["likelihood_field", "beam"])
        self.assertEqual(
            result, [{"sensor_model": "likelihood_field"}, {"sensor_model": "beam"}]
        )

    def test_returns_list_of_dicts(self):
        """Tests if named_product always returns a list of dictionaries.

        It verifies the return type is always a list and each
        element is a dictionary regardless of input size.
        """
        result = named_product(sensor_model=["likelihood_field"])
        self.assertIsInstance(result, list)
        self.assertIsInstance(result[0], dict)

    def test_empty_list_returns_no_combinations(self):
        """Tests if named_product handles an empty parameter list correctly.

        It verifies that an empty list in any parameter produces
        no combinations at all.
        """
        result = named_product(sensor_model=[], num_particles=[1])
        self.assertEqual(result, [])

    def test_no_parameters_returns_one_empty_dict(self):
        """Tests if named_product handles no parameters gracefully.

        It verifies that calling named_product with no arguments
        returns a list with a single empty dictionary.
        """
        result = named_product()
        self.assertEqual(result, [{}])


if __name__ == "__main__":
    unittest.main()
