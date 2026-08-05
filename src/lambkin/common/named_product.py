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
from typing import Any


def named_product(**parameters: list[Any]) -> list[dict[str, Any]]:
    """Generate all combinations of named parameters.

    Takes parameters where each value is a list of options
    and returns a list of dictionaries representing every possible
    combination, one dictionary per benchmark run configuration.

    Args:
        **parameters: Named parameter lists to combine. Each value must be a list.

    Returns:
        A list of dicts, each mapping parameter names to a specific value.
    """
    keys = list(parameters.keys())
    values = list(parameters.values())
    # TODO(teresa-ortega): Empty list exception
    combinations = itertools.product(*values)

    variants = []
    for combination in combinations:
        # Re-attach the parameter labels to the generated values.
        # zip() pairs keys with values; dict() creates the mapping.
        variant = dict(zip(keys, combination, strict=True))
        # TODO(teresa-ortega) : combination fails
        variants.append(variant)
    return variants
