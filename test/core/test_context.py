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
def base_variation():
    """Return a base variation dict for testing."""
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
def ctx(tmp_path, base_variation, base_options, base_source):
    """Return a fully constructed Context instance for testing."""
    return Context(
        variation=base_variation,
        iteration=0,
        output_dir=tmp_path,
        options=base_options,
        source=base_source,
    )


@pytest.mark.parametrize(
    "variation, iteration, variation_index, expected_sensor, expected_particles",
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
    variation,
    iteration,
    variation_index,
    expected_sensor,
    expected_particles,
):
    """variation_dir and iteration_dir are created on disk on instantiation."""
    ctx = Context(
        variation=variation,
        iteration=iteration,
        output_dir=tmp_path,
        options=base_options,
        source=base_source,
        variation_index=variation_index,
    )
    expected_variation_dir = tmp_path / f"var_{variation_index + 1}"
    expected_iteration_dir = expected_variation_dir / f"iter_{iteration}"

    assert ctx.output.variation_dir == expected_variation_dir
    assert ctx.output.iteration_dir == expected_iteration_dir
    assert ctx.output.variation_dir.exists()
    assert ctx.output.iteration_dir.exists()
    assert ctx.variation.sensor_model == expected_sensor
    assert ctx.variation.num_particles == expected_particles


def test_bag_dir_not_created_before_access(ctx):
    """bag/ folder does not exist before ctx.output.bag_dir is accessed."""
    assert not (ctx.output.iteration_dir / "bag").exists()


def test_metrics_dir_not_created_before_access(ctx):
    """metrics/ folder does not exist before ctx.output.metrics_dir is accessed."""
    assert not (ctx.output.iteration_dir / "metrics").exists()


def test_bag_dir_created_on_access(ctx):
    """bag/ folder is created when ctx.output.bag_dir is first accessed."""
    bag_dir = ctx.output.bag_dir
    assert bag_dir.exists()
    assert bag_dir == ctx.output.iteration_dir / "bag"
    assert ctx.output.bag_dir == ctx.output.bag_dir


def test_metrics_dir_created_on_access(ctx):
    """metrics/ folder is created when ctx.output.metrics_dir is first accessed."""
    metrics_dir = ctx.output.metrics_dir
    assert metrics_dir.exists()
    assert metrics_dir == ctx.output.iteration_dir / "metrics"
    assert ctx.output.metrics_dir == ctx.output.metrics_dir


@pytest.mark.parametrize(
    "variation, expected_attrs",
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
    tmp_path, base_options, base_source, variation, expected_attrs
):
    """ctx.variation exposes all key-value pairs from the variation dict."""
    ctx = Context(
        variation=variation,
        iteration=0,
        output_dir=tmp_path,
        options=base_options,
        source=base_source,
    )
    for key, value in expected_attrs.items():
        assert getattr(ctx.variation, key) == value


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
    base_variation,
    base_source,
    options,
    expected_clock,
    expected_qos,
    expected_rate,
):
    """ctx.options exposes clock, qos_option_path and rate correctly."""
    ctx = Context(
        variation=base_variation,
        iteration=0,
        output_dir=tmp_path,
        options=options,
        source=base_source,
    )
    assert ctx.options.clock == expected_clock
    assert ctx.options.qos_option_path == expected_qos
    assert ctx.options.rate == expected_rate


def test_source_path_is_set_correctly(ctx):
    """ctx.source.path reflects the path passed at construction."""
    assert ctx.source.path == Path(__file__)


def test_inputs_defaults_to_empty_namespace(ctx):
    """ctx.inputs is an empty SimpleNamespace when no inputs are registered."""
    assert isinstance(ctx.inputs, SimpleNamespace)
    assert vars(ctx.inputs) == {}


def test_inputs_can_be_dynamically_populated(ctx):
    """ctx.inputs correctly accepts dynamic attribute assignments via setattr().

    This verifies the behavior required by the @nominal.input system,
    which injects data post-instantiation via setattr(ctx.inputs, name, value).
    """
    ctx.inputs.dataset = Path("/data/bags/run1.bag")
    ctx.inputs.my_folder = Path("/data/my_folder")

    assert ctx.inputs.dataset == Path("/data/bags/run1.bag")
    assert ctx.inputs.my_folder == Path("/data/my_folder")


@pytest.mark.parametrize("iteration", [0, 1, 5, 42])
def test_iteration_stored(
    tmp_path, base_variation, base_options, base_source, iteration
):
    """ctx.iteration stores the zero-based repetition index."""
    ctx = Context(
        variation=base_variation,
        iteration=iteration,
        output_dir=tmp_path,
        options=base_options,
        source=base_source,
    )
    assert ctx.iteration == iteration
