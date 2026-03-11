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

"""Named product factory for lambkin benchmarks.

Provides named_product(), a utility to generate combinatorial parameter sets.
Used to define benchmark configurations in a readable and structured way.
"""

import itertools


def named_product(**parameters):
    """Generate all combinations of named parameters.

    Takes keyword arguments where each value is a list of options
    and returns a list of dictionaries representing every possible
    combination, one dictionary per benchmark run configuration.

    Args:
        **parameters: Named parameter lists to combine. Each value must be a list.

    Returns:
        A list of dicts, each mapping parameter names to a specific value.
    """
    # Example:
    # parameters == (sensor_model = [likelihood_field,beam],
    #            num_particles = [1, 10, 1000, 2000])

    # keys = ("sensor_model", "num_particles")
    keys = list(parameters.keys())

    # values = ( [likelihood_field,beam],[1, 10, 1000, 2000] )
    values = list(parameters.values())

    # TO_DO: Empty list exception

    # Make combinations
    combinations = itertools.product(*values)

    # Create the dictionary
    variants = []
    for combination in combinations:
        # {"sensor_model" : "likelihood_field", "num_particles" : "1"}
        variant = dict(zip(keys, combination, strict=True))
        # TO_DO : combination fails
        variants.append(variant)
    return variants
