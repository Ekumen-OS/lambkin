# Copyright 2023 Ekumen, Inc.
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

"""This module provides py:mod:`pint` powered physical units."""

import functools
import pint

Quantity = pint.Quantity

DEFAULT_REGISTRY = ureg = pint.UnitRegistry(preprocessors=[
    lambda s: s.replace('%', ' percent ')
])
DEFAULT_REGISTRY.define('percent = 0.01 = %')


@functools.lru_cache
def base_unit_scale(unit: str) -> float:
    """Get SI unit scale factor."""
    if not unit:
        return 1.0
    return ureg.Quantity(unit).to_base_units().magnitude
