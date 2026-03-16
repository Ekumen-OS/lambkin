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

import pytest

from lambkin.core.ctx.context import Context


@pytest.fixture
def base_variation():
    """Return a base variation dict for testing."""
    return {"sensor_model": "beam", "num_particles": 10}


@pytest.fixture
def base_options():
    """Return a base options dict for testing."""
    return {"clock": True, "qos_option_path": "sensor_data", "rate": 1.0}


@pytest.fixture
def ctx(tmp_path, base_variation, base_options):
    """Return a fully constructed Context instance for testing."""
    return Context(
        variation=base_variation,
        iteration=0,
        output_dir=tmp_path,
        options=base_options,
    )


@pytest.mark.parametrize(
    "variation, iteration, expected_variation_dir, expected_iteration_dir, "
    "expected_sensor, expected_particles",
    [
        (
            {"sensor_model": "beam", "num_particles": 10},
            0,
            "beam_10",
            "beam_10/iter_0",
            "beam",
            10,
        ),
        (
            {"sensor_model": "likelihood_field", "num_particles": 500},
            2,
            "likelihood_field_500",
            "likelihood_field_500/iter_2",
            "likelihood_field",
            500,
        ),
        (
            {"sensor_model": "beam", "num_particles": 1},
            10,
            "beam_1",
            "beam_1/iter_10",
            "beam",
            1,
        ),
    ],
)
def test_output_dirs_are_created_on_instantiation(
    tmp_path,
    base_options,
    variation,
    iteration,
    expected_variation_dir,
    expected_iteration_dir,
    expected_sensor,
    expected_particles,
):
    """variation_dir and iteration_dir are created on disk on instantiation."""
    ctx = Context(
        variation=variation,
        iteration=iteration,
        output_dir=tmp_path,
        options=base_options,
    )
    assert ctx.output.variation_dir == tmp_path / expected_variation_dir
    assert ctx.output.iteration_dir == tmp_path / expected_iteration_dir
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
def test_variation_attributes(tmp_path, base_options, variation, expected_attrs):
    """ctx.variation exposes all key-value pairs from the variation dict."""
    ctx = Context(
        variation=variation,
        iteration=0,
        output_dir=tmp_path,
        options=base_options,
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
    )
    assert ctx.options.clock == expected_clock
    assert ctx.options.qos_option_path == expected_qos
    assert ctx.options.rate == expected_rate


@pytest.mark.parametrize(
    "source_path, expected",
    [
        ("/opt/ros/overlay/amcl", Path("/opt/ros/overlay/amcl")),
        ("relative/path/pkg", Path("relative/path/pkg")),
    ],
)
def test_source_path_set_correctly(
    tmp_path, base_variation, base_options, source_path, expected
):
    """ctx.source.path reflects the source_path argument passed at construction."""
    ctx = Context(
        variation=base_variation,
        iteration=0,
        output_dir=tmp_path,
        options=base_options,
        source_path=source_path,
    )
    assert ctx.source.path == expected


def test_inputs_defaults_to_empty_path(ctx):
    """ctx.inputs.dataset defaults to Path() when dataset_path is not provided."""
    assert ctx.inputs.dataset == Path()


@pytest.mark.parametrize(
    "dataset_path, expected",
    [
        ("/data/bags/run1.bag", Path("/data/bags/run1.bag")),
        ("bags/my_bag", Path("bags/my_bag")),
    ],
)
def test_inputs_dataset_set_correctly(
    tmp_path, base_variation, base_options, dataset_path, expected
):
    """ctx.inputs.dataset reflects the dataset_path argument passed at construction."""
    ctx = Context(
        variation=base_variation,
        iteration=0,
        output_dir=tmp_path,
        options=base_options,
        dataset_path=dataset_path,
    )
    assert ctx.inputs.dataset == expected


@pytest.mark.parametrize("iteration", [0, 1, 5, 42])
def test_iteration_stored(tmp_path, base_variation, base_options, iteration):
    """ctx.iteration stores the zero-based repetition index."""
    ctx = Context(
        variation=base_variation,
        iteration=iteration,
        output_dir=tmp_path,
        options=base_options,
    )
    assert ctx.iteration == iteration
