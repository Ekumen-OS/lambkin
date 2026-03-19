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
from click import Option

from lambkin.core.decorators.option import option


@pytest.fixture
def simple_fn():
    """A plain function to decorate."""

    def fn():
        pass

    return fn


def test_option_uses_namespaced_dunder(simple_fn):
    """Verify we use the protected __lambkin_options__ attribute for safety."""
    decorated = option("--clock-rate")(simple_fn)
    assert hasattr(decorated, "__lambkin_options__")


def test_option_stores_single_option(simple_fn):
    """@lambkin.option stores one click.Option on the function."""
    decorated = option("--clock-rate", default=100.0)(simple_fn)
    assert len(decorated.__lambkin_options__) == 1
    assert isinstance(decorated.__lambkin_options__[0], Option)


def test_option_is_real_click_object(simple_fn):
    """Verify we are actually storing click.Option objects, not tuples."""
    decorated = option("--clock-rate", default=100.0)(simple_fn)
    opt = decorated.__lambkin_options__[0]
    assert isinstance(opt, Option)
    assert opt.name == "clock_rate"


def test_option_stacks_multiple_decorators():
    """Multiple @option decorators append to __lambkin_options__, not overwrite."""

    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn():
        pass

    assert len(fn.__lambkin_options__) == 2


def test_option_stacks_bottom_up():
    """Decorators are applied bottom-up, so --sensor-topic is first."""

    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn():
        pass

    assert fn.__lambkin_options__[0].name == "sensor_topic"
    assert fn.__lambkin_options__[1].name == "clock_rate"


def test_option_fails_on_invalid_declaration(simple_fn):
    """option() raises an error if the flag name is invalid."""
    with pytest.raises(ValueError):
        option("clock-rate")(simple_fn)


def test_option_no_options_by_default(simple_fn):
    """A plain function has no _options attribute."""
    assert not hasattr(simple_fn, "__lambkin_options__")
