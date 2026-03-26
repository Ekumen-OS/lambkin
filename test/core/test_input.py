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


def test_two_hooks_registered_in_order():
    """Hooks are stored in registration order."""
    registry = InputRegistry()

    def dataset(ctx):
        return "d"

    def map(ctx):
        return "m"

    registry.register(dataset)
    registry.register(map)

    assert [h.__name__ for h in registry._hooks] == ["dataset", "map"]


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


def test_inputs_resolved_on_every_iteration(variant):
    """resolve() must run once per iteration, not just once upfront."""
    call_count = [0]
    seen = []

    @benchmark(variants=variant, num_iterations=3)
    def nominal(ctx):
        seen.append(ctx.inputs.counter)

    @nominal.input
    def counter(ctx):
        call_count[0] += 1
        return call_count[0]

    nominal(output_dir="/tmp")

    assert seen == [1, 2, 3]
