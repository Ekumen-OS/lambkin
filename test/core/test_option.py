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

"""Unit tests for the option decorator in lambkin.core.decorators."""

import pytest

from lambkin.core.decorators.option import option


@pytest.fixture
def simple_fn():
    """A plain function to decorate."""

    def fn():
        pass

    return fn


def test_option_stores_single_option(simple_fn):
    """@l.option stores one option definition on the function."""
    decorated = option("--clock-rate", default=100.0)(simple_fn)
    assert len(decorated._options) == 1
    assert decorated._options[0] == (("--clock-rate",), {"default": 100.0})


def test_option_stacks_multiple_decorators(simple_fn):
    """Multiple @l.option decorators each add one entry to fn._options."""
    decorated = option("--clock-rate", default=100.0)(simple_fn)
    decorated = option("--sensor-topic", default="/scan")(decorated)
    assert len(decorated._options) == 2


def test_option_stores_attrs_correctly(simple_fn):
    """@l.option stores all kwargs correctly."""
    decorated = option("--clock-rate", default=100.0)(simple_fn)
    _, attrs = decorated._options[0]
    assert attrs["default"] == 100.0


def test_option_normalization_single_flag(simple_fn):
    """--clock-rate normalizes to clock_rate."""
    decorated = option("--clock-rate", default=100.0)(simple_fn)
    param_decls, _ = decorated._options[0]
    key = max(param_decls, key=len).lstrip("-").replace("-", "_")
    assert key == "clock_rate"


def test_option_normalization_picks_longest_flag(simple_fn):
    """--clock-rate is picked over -c when normalizing."""
    decorated = option("--clock-rate", "-c", default=100.0)(simple_fn)
    param_decls, _ = decorated._options[0]
    key = max(param_decls, key=len).lstrip("-").replace("-", "_")
    assert key == "clock_rate"


def test_option_no_options_by_default(simple_fn):
    """A plain function has no _options attribute."""
    assert not hasattr(simple_fn, "_options")
