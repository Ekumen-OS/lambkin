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

"""Unit tests for the context function in lambkin.core.ctx."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from lambkin.core.ctx.context import Context
from lambkin.core.ctx.source import Source


@pytest.fixture
def base_variant():
    """Return a base variant dict for testing."""
    return {"sensor_model": "beam", "num_particles": 10}


@pytest.fixture
def base_options():
    """Return a base options dict for testing."""
    return {"clock": True, "qos_option_path": "sensor_data", "rate": 1.0}


@pytest.fixture
def base_source():
    """Return a base Source instance for testing."""
    return Source(path=Path(__file__))


@pytest.fixture
def ctx(tmp_path, base_variant, base_options, base_source):
    """Return a fully constructed Context instance for testing."""
    return Context(
        variant=base_variant,
        iteration=0,
        options=base_options,
        source=base_source,
        base_dir=tmp_path,
    )


@pytest.mark.parametrize(
    "variant, iteration, variant_index, expected_sensor, expected_particles",
    [
        ({"sensor_model": "beam", "num_particles": 10}, 0, 0, "beam", 10),
        (
            {"sensor_model": "likelihood_field", "num_particles": 500},
            2,
            1,
            "likelihood_field",
            500,
        ),
        ({"sensor_model": "beam", "num_particles": 1}, 10, 2, "beam", 1),
    ],
)
def test_output_dirs_are_created_on_instantiation(
    tmp_path,
    base_options,
    base_source,
    variant,
    iteration,
    variant_index,
    expected_sensor,
    expected_particles,
):
    """variation_dir and iteration_dir are created on disk on instantiation."""
    ctx = Context(
        variant=variant,
        iteration=iteration,
        options=base_options,
        source=base_source,
        base_dir=tmp_path,
        variant_index=variant_index,
    )
    expected_variant_dir = tmp_path / f"var_{variant_index + 1}"
    expected_iteration_dir = expected_variant_dir / f"iter_{iteration + 1}"

    assert ctx.paths.base_dir == tmp_path
    assert ctx.paths.variant_dir == expected_variant_dir
    assert ctx.paths.iteration_dir == expected_iteration_dir
    assert ctx.paths.variant_dir.exists()
    assert ctx.paths.iteration_dir.exists()
    assert ctx.variant.sensor_model == expected_sensor
    assert ctx.variant.num_particles == expected_particles


@pytest.mark.parametrize(
    "variant, expected_attrs",
    [
        (
            {"sensor_model": "beam", "num_particles": 10},
            {"sensor_model": "beam", "num_particles": 10},
        ),
        (
            {
                "sensor_model": "likelihood_field",
                "num_particles": 500,
                "map_resolution": 0.05,
            },
            {
                "sensor_model": "likelihood_field",
                "num_particles": 500,
                "map_resolution": 0.05,
            },
        ),
    ],
)
def test_variation_attributes(
    tmp_path, base_options, base_source, variant, expected_attrs
):
    """ctx.variation exposes all key-value pairs from the variation dict."""
    ctx = Context(
        variant=variant,
        iteration=0,
        options=base_options,
        source=base_source,
        base_dir=tmp_path,
    )
    for key, value in expected_attrs.items():
        assert getattr(ctx.variant, key) == value


@pytest.mark.parametrize(
    "options, expected_clock, expected_qos, expected_rate",
    [
        (
            {"clock": True, "qos_option_path": "sensor_data", "rate": 1.0},
            True,
            "sensor_data",
            1.0,
        ),
        (
            {"clock": False, "qos_option_path": "system_default", "rate": 2.0},
            False,
            "system_default",
            2.0,
        ),
    ],
)
def test_options_attributes(
    tmp_path,
    base_variant,
    base_source,
    options,
    expected_clock,
    expected_qos,
    expected_rate,
):
    """ctx.options exposes clock, qos_option_path and rate correctly."""
    ctx = Context(
        variant=base_variant,
        iteration=0,
        options=options,
        source=base_source,
        base_dir=tmp_path,
    )
    assert ctx.options.clock == expected_clock
    assert ctx.options.qos_option_path == expected_qos
    assert ctx.options.rate == expected_rate


def test_source_path_is_set_correctly(ctx):
    """ctx.source.path reflects the path passed at construction."""
    assert ctx.source.path == Path(__file__)


def test_inputs_defaults_to_empty_namespace(ctx):
    """ctx.inputs is None when no inputs are registered."""
    assert ctx.inputs is None


@pytest.mark.parametrize("iteration", [0, 1, 5, 42])
def test_iteration_stored(tmp_path, base_variant, base_options, base_source, iteration):
    """ctx.iteration stores the zero-based repetition index."""
    ctx = Context(
        variant=base_variant,
        iteration=iteration,
        options=base_options,
        source=base_source,
        base_dir=tmp_path,
    )
    assert ctx.iteration == iteration


def test_context_is_immutable(tmp_path):
    """Context fields cannot be reassigned after construction."""
    source = Source("/my_benchmark.py")
    ctx = Context(
        variant={"sensor_model": "beam"},
        iteration=0,
        options={"dry_run": True},
        source=source,
        base_dir=tmp_path,
        variant_index=0,
    )

    with pytest.raises(AttributeError):
        ctx.variant = SimpleNamespace()

    with pytest.raises(AttributeError):
        ctx.options = SimpleNamespace()

    with pytest.raises(AttributeError):
        ctx.inputs = SimpleNamespace()

    with pytest.raises(AttributeError):
        ctx.iteration = 99

    with pytest.raises(AttributeError):
        ctx.shell = None
