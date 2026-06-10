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

"""Common utilities and primitives shared across the lambkin SDK.

Provides defaults, named products, base exceptions, and signal handling utilities.
"""

from lambkin.common import defaults, exceptions, signals

from .named_product import named_product

__all__ = [
    "defaults",
    "exceptions",
    "named_product",
    "signals",
]
