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

"""Unit tests for the input decorator in lambkin.core.decorators."""

import pytest

from lambkin.core.ctx import Context
from lambkin.core.decorators.benchmark import benchmark
from lambkin.core.decorators.input import InputRegistry


@pytest.fixture
def variant():
    """Base variants for testing."""
    return [
        {"sensor_model": "beam", "num_particles": 10},
    ]


def test_register_returns_original_function():
    """register() returns the original function unchanged."""
    registry = InputRegistry()

    def dataset(ctx):
        return "data.mcap"

    assert registry.register(dataset) is dataset


def test_registered_hook_name_is_preserved():
    """The ctx.inputs attribute name is the hook's __name__."""
    registry = InputRegistry()

    def my_dataset(ctx):
        return "x"

    registry.register(my_dataset)
    assert registry._hooks[0].__name__ == "my_dataset"


def test_hook_with_no_parameters_raises():
    """A hook with no parameters raises ValueError at resolve time."""
    registry = InputRegistry()
    ctx = Context.__new__(Context)

    def bad_hook():
        return "value"

    registry.register(bad_hook)
    with pytest.raises(ValueError, match="exactly 1 parameter"):
        registry.resolve(ctx)


def test_hook_with_extra_parameters_raises():
    """A hook with more than 1 parameter raises ValueError at resolve time."""
    registry = InputRegistry()
    ctx = Context.__new__(Context)

    def bad_hook(ctx, extra):
        return "value"

    registry.register(bad_hook)
    with pytest.raises(ValueError, match="exactly 1 parameter"):
        registry.resolve(ctx)


def test_hook_returning_none_raises():
    """A hook that returns None raises ValueError."""
    registry = InputRegistry()
    ctx = Context.__new__(Context)

    def dataset(ctx):
        return None

    registry.register(dataset)
    with pytest.raises(ValueError, match="None"):
        registry.resolve(ctx)


def test_hook_with_no_return_raises():
    """A hook with no return statement raises ValueError."""
    registry = InputRegistry()
    ctx = Context.__new__(Context)

    def dataset(ctx):
        pass

    registry.register(dataset)
    with pytest.raises(ValueError, match="None"):
        registry.resolve(ctx)


def test_hook_returning_empty_string_raises():
    """A hook that returns an empty string raises ValueError."""
    registry = InputRegistry()
    ctx = Context.__new__(Context)

    def dataset(ctx):
        return ""

    registry.register(dataset)
    with pytest.raises(ValueError, match="empty"):
        registry.resolve(ctx)


def test_hook_returning_blank_string_raises():
    """A hook that returns a whitespace-only string raises ValueError."""
    registry = InputRegistry()
    ctx = Context.__new__(Context)

    def dataset(ctx):
        return "   "

    registry.register(dataset)
    with pytest.raises(ValueError, match="empty"):
        registry.resolve(ctx)


def test_ctx_inputs_populated_before_fn_runs(variant):
    """ctx.inputs.dataset is available inside the benchmark function after resolve."""
    seen = []

    @benchmark(variants=variant, num_iterations=1)
    def nominal(ctx):
        seen.append(ctx.inputs.dataset)

    @nominal.input
    def dataset(ctx):
        return "path/to/dataset.mcap"

    nominal(output_dir="/tmp")
    assert seen == ["path/to/dataset.mcap"]


def test_multiple_inputs_all_injected(variant):
    """All registered inputs are injected into ctx.inputs before the benchmark runs."""
    seen = []

    @benchmark(variants=variant, num_iterations=1)
    def nominal(ctx):
        seen.append((ctx.inputs.dataset, ctx.inputs.map))

    @nominal.input
    def dataset(ctx):
        return "path/to/dataset.mcap"

    @nominal.input
    def map(ctx):
        return "path/to/map.yaml"

    nominal(output_dir="/tmp")
    assert seen == [("path/to/dataset.mcap", "path/to/map.yaml")]
