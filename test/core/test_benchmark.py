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

"""Unit tests for the benchmark decorator in lambkin.core.decorators."""

from pathlib import Path

import pytest

from lambkin.core.decorators.benchmark import _parse_options, benchmark
from lambkin.core.decorators.option import option


@pytest.fixture
def variants():
    """Base variants for testing."""
    return [
        {"sensor_model": "beam", "num_particles": 10},
        {"sensor_model": "likelihood", "num_particles": 100},
    ]


def test_parse_options_returns_empty_dict_when_no_options():
    """_parse_options returns empty dict when fn has no __lambkin_options__."""

    def fn(ctx):
        pass

    result = _parse_options(fn, [])
    assert result == {}


def test_parse_options_returns_defaults_when_no_args():
    """_parse_options returns default values when no CLI args are provided."""

    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn(ctx):
        pass

    result = _parse_options(fn, [])
    assert result == {"clock_rate": 100.0, "sensor_topic": "/scan"}


def test_parse_options_returns_cli_values_when_provided():
    """_parse_options returns CLI values when args are provided."""

    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn(ctx):
        pass

    result = _parse_options(fn, ["--clock-rate", "50.0"])
    assert result == {"clock_rate": 50.0, "sensor_topic": "/scan"}


def test_benchmark_loops_over_variants_and_iterations(variants):
    """Benchmark calls fn once per (variation, iteration) pair."""
    calls = []

    @benchmark(variants=variants, num_iterations=3)
    def fn(ctx):
        calls.append(ctx)

    fn()
    assert len(calls) == 6


def test_benchmark_variation_attributes_are_correct(variants):
    """ctx.variation exposes the variant dict as attributes."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    def fn(ctx):
        contexts.append(ctx)

    fn()
    assert contexts[0].variation.sensor_model == "beam"
    assert contexts[0].variation.num_particles == 10
    assert contexts[1].variation.sensor_model == "likelihood"
    assert contexts[1].variation.num_particles == 100


def test_benchmark_options_defaults_injected(variants):
    """Default option values are injected into ctx.options."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn(ctx):
        contexts.append(ctx)

    fn()
    assert contexts[0].options.clock_rate == 100.0
    assert contexts[0].options.sensor_topic == "/scan"


def test_benchmark_source_path_points_to_benchmark_script(variants):
    """Source.path points to the file where the benchmark function is defined."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    def fn(ctx):
        contexts.append(ctx)

    fn()
    assert contexts[0].source.path == Path(__file__)


def test_benchmark_options_injected_via_args(variants):
    """CLI args passed explicitly to wrapper() override decorator defaults."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    @option("--clock-rate", default=100.0)
    def fn(ctx):
        contexts.append(ctx)

    fn(args=["--clock-rate", "50.0"])
    assert contexts[0].options.clock_rate == 50.0
