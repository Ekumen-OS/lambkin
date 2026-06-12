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

from pathlib import Path
from types import SimpleNamespace

import pytest

from lambkin.core.ctx.benchmark_context import BenchmarkContext
from lambkin.core.ctx.iteration_context import IterationContext
from lambkin.core.ctx.source import Source
from lambkin.core.ctx.variant_context import VariantContext
from lambkin.core.decorators.benchmark import benchmark
from lambkin.core.decorators.input import InputRegistry


@pytest.fixture
def variant():
    """Base variants for testing."""
    return [
        {"sensor_model": "beam", "num_particles": 10},
    ]


@pytest.fixture
def base_options():
    """Minimal dry-run options."""
    return {
        "dry_run": True,
        "no_cache": False,
        "log_output": "file",
        "log_level": "info",
    }


@pytest.fixture
def source(tmp_path):
    """A Source pointing to this test file."""
    return Source(path=Path(__file__))


@pytest.fixture
def bctx(source, base_options, tmp_path):
    """An entered BenchmarkContext."""
    with BenchmarkContext(
        source=source, options=base_options, base_dir=tmp_path
    ) as ctx:
        yield ctx


@pytest.fixture
def vctx(bctx):
    """An entered VariantContext."""
    with VariantContext(
        benchmark_ctx=bctx,
        variant={"sensor_model": "beam", "num_particles": 10},
        variant_index=0,
    ) as ctx:
        yield ctx


def test_register_returns_original_function():
    """register() returns the original function unchanged."""
    registry = InputRegistry()

    def dataset(ctx):
        return "data.mcap"

    assert registry.register(dataset) is dataset


def test_register_with_scope_returns_original_function():
    """register(scope=...) also returns the original function unchanged."""
    registry = InputRegistry()

    def dataset(ctx):
        return "data.mcap"

    assert registry.register(scope="variant")(dataset) is dataset


def test_registered_hook_name_is_preserved(bctx):
    """The ctx.inputs attribute name is the hook's __name__."""
    registry = InputRegistry()

    def my_dataset(ctx):
        return "some_value"

    registry.register(my_dataset)
    result = registry.resolve(bctx)
    assert hasattr(result, "my_dataset")


def test_hook_with_no_parameters_raises():
    """A hook with no parameters raises ValueError at registration time."""
    registry = InputRegistry()

    def bad_hook():
        return "value"

    with pytest.raises(ValueError, match="exactly 1 parameter."):
        registry.register(bad_hook)


def test_hook_with_extra_parameters_raises():
    """A hook with more than 1 parameter raises ValueError at registration time."""
    registry = InputRegistry()

    def bad_hook(ctx, extra):
        return "value"

    with pytest.raises(ValueError, match="exactly 1 parameter."):
        registry.register(bad_hook)


def test_name_collision_across_scopes_raises():
    """Registering two hooks with the same name in different scopes raises."""
    registry = InputRegistry()

    def dataset(ctx):
        return "benchmark.mcap"

    registry.register(dataset)

    def dataset(ctx):  # noqa: F811
        return "variant.mcap"

    with pytest.raises(ValueError, match="Hook name conflict"):
        registry.register(dataset, scope="variant")


def test_invalid_scope_raises():
    """Registering a hook with an invalid scope raises ValueError."""
    registry = InputRegistry()

    def dataset(ctx):
        return "data.mcap"

    with pytest.raises(ValueError, match="Invalid scope"):
        registry.register(dataset, scope="invalid")


def test_hook_returning_none_raises(bctx):
    """A hook that returns None raises ValueError."""
    registry = InputRegistry()

    def dataset(ctx):
        return None

    registry.register(dataset)
    with pytest.raises(ValueError, match="None"):
        registry.resolve(bctx)


def test_hook_with_no_return_raises(bctx):
    """A hook with no return statement raises ValueError."""
    registry = InputRegistry()

    def dataset(ctx):
        pass

    registry.register(dataset)
    with pytest.raises(ValueError, match="None"):
        registry.resolve(bctx)


def test_hook_returning_empty_string_raises(bctx):
    """A hook that returns an empty string raises ValueError."""
    registry = InputRegistry()

    def dataset(ctx):
        return ""

    registry.register(dataset)
    with pytest.raises(ValueError, match="empty"):
        registry.resolve(bctx)


def test_hook_returning_blank_string_raises(bctx):
    """A hook that returns a whitespace-only string raises ValueError."""
    registry = InputRegistry()

    def dataset(ctx):
        return "   "

    registry.register(dataset)
    with pytest.raises(ValueError, match="empty"):
        registry.resolve(bctx)


def test_benchmark_scoped_hook_receives_benchmark_context(bctx):
    """Benchmark-scoped hook receives a BenchmarkContext."""
    registry = InputRegistry()
    received = []

    def dataset(ctx):
        received.append(ctx)
        return "data.mcap"

    registry.register(dataset)
    registry.resolve(bctx)
    assert received[0] is bctx


def test_variant_scoped_hook_receives_variant_context(vctx):
    """Variant-scoped hook receives a VariantContext with correct variant."""
    registry = InputRegistry()
    received = []

    def dataset(ctx):
        received.append(ctx)
        return "data.mcap"

    registry.register(dataset, scope="variant")
    registry.resolve(vctx)
    assert received[0] is vctx
    assert received[0].variant.sensor_model == "beam"


def test_iteration_scoped_hook_receives_iteration_context(vctx):
    """Iteration-scoped hook receives an IterationContext."""
    registry = InputRegistry()
    received = []

    def seed(ctx):
        received.append(ctx)
        return f"seed_{ctx.iteration}"

    registry.register(seed, scope="iteration")

    with IterationContext(variant_ctx=vctx, iteration=2) as ctx:
        registry.resolve(ctx)
    assert isinstance(received[0], IterationContext)
    assert received[0].iteration == 2


def test_variant_inputs_merged_with_benchmark_inputs(bctx, vctx):
    """Variant-scoped inputs are merged with benchmark-scoped inputs."""
    registry = InputRegistry()

    def reference(ctx):
        return "ref.tum"

    def dataset(ctx):
        return f"{ctx.variant.sensor_model}.mcap"

    registry.register(reference)
    registry.register(dataset, scope="variant")

    bctx.inputs = registry.resolve(bctx)
    vctx.inputs = registry.resolve(vctx)

    assert vctx.inputs.reference == "ref.tum"
    assert vctx.inputs.dataset == "beam.mcap"


def test_iteration_inputs_merged_with_parent_inputs(bctx, vctx):
    """Iteration-scoped inputs are merged with benchmark + variant inputs."""
    registry = InputRegistry()

    it = 3
    multiplier = 43

    def reference(ctx):
        return "ref.tum"

    def dataset(ctx):
        return f"{ctx.variant.sensor_model}.mcap"

    def seed(ctx):
        return ctx.iteration * multiplier

    registry.register(reference)
    registry.register(dataset, scope="variant")
    registry.register(seed, scope="iteration")

    bctx.inputs = registry.resolve(bctx)
    vctx.inputs = registry.resolve(vctx)

    with IterationContext(variant_ctx=vctx, iteration=it) as ctx:
        ctx.inputs = registry.resolve(ctx)
        assert ctx.inputs.reference == "ref.tum"
        assert ctx.inputs.dataset == "beam.mcap"
        assert ctx.inputs.seed == it * multiplier


def test_inputs_locked_after_resolution(bctx):
    """ctx.inputs raises AttributeError if assigned again after resolution."""
    bctx.inputs = SimpleNamespace(reference="ref.tum")
    with pytest.raises(AttributeError, match="read-only after resolution"):
        bctx.inputs = SimpleNamespace(reference="other.tum")


def test_ctx_inputs_populated_before_fn_runs(variant, tmp_path):
    """ctx.inputs.dataset is available inside the benchmark function after resolve."""
    seen = []

    @benchmark(variants=variant, num_iterations=1)
    def nominal(ctx):
        seen.append(ctx.inputs.dataset)

    @nominal.input
    def dataset(ctx):
        return "path/to/dataset.mcap"

    nominal(base_dir=tmp_path)
    assert seen == ["path/to/dataset.mcap"]


def test_multiple_inputs_all_injected(variant, tmp_path):
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

    nominal(base_dir=tmp_path)
    assert seen == [("path/to/dataset.mcap", "path/to/map.yaml")]


def test_input_hook_called_once_regardless_of_variants_and_iterations(tmp_path):
    """Input hooks are resolved once, regardless of variants and iterations."""
    call_count = 0

    @benchmark(
        variants=[
            {"sensor_model": "beam", "num_particles": 10},
            {"sensor_model": "lidar", "num_particles": 20},
        ],
        num_iterations=3,
    )
    def nominal(ctx):
        pass

    @nominal.input
    def dataset(ctx):
        nonlocal call_count
        call_count += 1
        return "path/to/dataset.mcap"

    nominal(base_dir=tmp_path)
    assert call_count == 1


def test_variant_scoped_hook_sees_correct_variant(tmp_path):
    """Variant-scoped hook receives the correct variant for each run."""
    seen = []
    variants = [
        {"sensor_model": "beam"},
        {"sensor_model": "likelihood"},
    ]

    @benchmark(variants=variants, num_iterations=1)
    def nominal(ctx):
        seen.append(ctx.inputs.dataset)

    @nominal.input(scope="variant")
    def dataset(ctx):
        return f"{ctx.variant.sensor_model}.mcap"

    nominal(base_dir=tmp_path)
    assert seen == ["beam.mcap", "likelihood.mcap"]


def test_iteration_scoped_hook_sees_correct_iteration(tmp_path):
    """Iteration-scoped hook receives the correct iteration index."""
    seen = []

    @benchmark(variants=[{"x": 1}], num_iterations=3)
    def nominal(ctx):
        seen.append(ctx.inputs.seed)

    @nominal.input(scope="iteration")
    def seed(ctx):
        return ctx.iteration

    nominal(base_dir=tmp_path)
    assert seen == [0, 1, 2]


def test_input_hooks_called_even_when_all_iterations_cached(variant, tmp_path):
    """Input hooks are resolved even when all iterations are cache hits."""
    call_count = 0

    @benchmark(variants=variant, num_iterations=1)
    def nominal(ctx):
        pass

    @nominal.input
    def dataset(ctx):
        nonlocal call_count
        call_count += 1
        return "path/to/dataset.mcap"

    # First run — completes the iteration
    nominal(base_dir=tmp_path)
    assert call_count == 1

    # Second run — iteration is cached, but input hooks still resolve
    call_count = 0
    nominal(base_dir=tmp_path)
    assert call_count == 1
