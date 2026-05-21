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
from unittest.mock import patch

import pytest

from lambkin.common import defaults
from lambkin.core.ctx.context import Context
from lambkin.core.ctx.source import Source
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
    assert result == {
        "dry_run": defaults.DRY_RUN,
        "show_options": False,
    }


def test_parse_options_returns_defaults_when_no_args():
    """_parse_options returns default values when no CLI args are provided."""

    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn(ctx):
        pass

    result = _parse_options(fn, [])
    assert result == {
        "dry_run": defaults.DRY_RUN,
        "show_options": False,
        "clock_rate": 100.0,
        "sensor_topic": "/scan",
    }


def test_parse_options_returns_cli_values_when_provided():
    """_parse_options returns CLI values when args are provided."""

    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn(ctx):
        pass

    result = _parse_options(fn, ["--clock-rate", "50.0"])
    assert result == {
        "dry_run": defaults.DRY_RUN,
        "show_options": False,
        "clock_rate": 50.0,
        "sensor_topic": "/scan",
    }


def test_benchmark_preserves_metadata(variants, tmp_path):
    """@benchmark preserves the decorated function's name and docstring."""

    @benchmark(variants=variants, num_iterations=1)
    def my_cool_benchmark(ctx):
        """Standard docstring."""
        pass

    assert my_cool_benchmark.__name__ == "my_cool_benchmark"
    assert my_cool_benchmark.__doc__ == "Standard docstring."


def test_benchmark_loops_over_variants_and_iterations(variants, tmp_path):
    """Benchmark calls fn once per (variant, iteration) pair."""
    calls = []

    @benchmark(variants=variants, num_iterations=3)
    def fn(ctx):
        calls.append(ctx)

    fn(output_dir=tmp_path)
    assert len(calls) == 6


def test_benchmark_variant_attributes_are_correct(variants, tmp_path):
    """ctx.variant exposes the variant dict as attributes."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    def fn(ctx):
        contexts.append(ctx)

    fn(output_dir=tmp_path)
    assert contexts[0].variant.sensor_model == "beam"
    assert contexts[0].variant.num_particles == 10
    assert contexts[1].variant.sensor_model == "likelihood"
    assert contexts[1].variant.num_particles == 100


def test_benchmark_options_defaults_injected(variants, tmp_path):
    """Default option values are injected into ctx.options."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    @option("--clock-rate", default=100.0)
    @option("--sensor-topic", default="/scan")
    def fn(ctx):
        contexts.append(ctx)

    fn(output_dir=tmp_path)
    assert contexts[0].options.clock_rate == 100.0
    assert contexts[0].options.sensor_topic == "/scan"


def test_benchmark_options_injected_via_args(variants, tmp_path):
    """CLI args passed explicitly to wrapper() override decorator defaults."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    @option("--clock-rate", default=100.0)
    def fn(ctx):
        contexts.append(ctx)

    fn(args=["--clock-rate", "50.0"], output_dir=tmp_path)
    assert contexts[0].options.clock_rate == 50.0


def test_benchmark_source_path_points_to_benchmark_script(variants, tmp_path):
    """Source.path points to the file where the benchmark function is defined."""
    contexts = []

    @benchmark(variants=variants, num_iterations=1)
    def fn(ctx):
        contexts.append(ctx)

    fn(output_dir=tmp_path)
    assert contexts[0].source.path == Path(__file__)


def test_benchmark_default_output_dir_uses_source_path(variants, tmp_path):
    """When output_dir=None, output is resolved relative to the source path."""
    contexts = []
    fake_script = tmp_path / "my_benchmark.py"
    fake_script.touch()
    fake_source = Source(path=fake_script)

    @benchmark(variants=variants, num_iterations=1)
    def fn(ctx):
        contexts.append(ctx)

    with patch("lambkin.core.decorators.benchmark.Source", return_value=fake_source):
        fn()

    assert contexts[0].output.base_dir == tmp_path / Context.BENCHMARKS_DIRNAME
    assert contexts[0].source.path == fake_script


def test_benchmark_empty_variants_raises_error(tmp_path):
    """Calling a benchmark with an empty variants list raises an exception."""
    expected_message = (
        "You have provided an empty variants list; therefore, no "
        "benchmarking iterations will be executed."
    )

    with pytest.raises(ValueError, match=expected_message):

        @benchmark(variants=[], num_iterations=1)
        def fn(ctx):
            pass


def test_parse_options_includes_dry_run_by_default():
    """_parse_options always includes dry_run even when no user options are declared."""

    def fn(ctx):
        pass

    result = _parse_options(fn, [])
    assert "dry_run" in result
    assert result["dry_run"] is defaults.DRY_RUN


def test_parse_options_dry_run_can_be_set_via_cli():
    """_parse_options returns dry_run=True when --dry-run is passed."""

    def fn(ctx):
        pass

    result = _parse_options(fn, ["--dry-run"])
    assert result["dry_run"] is True


def test_show_options_no_options_registered(capsys):
    """No @lambkin.option shows a 'no options' message."""

    @benchmark(variants=[{}], num_iterations=1)
    def fn(ctx):
        pass

    with pytest.raises(SystemExit) as exc:
        fn(args=["--show-options"])

    assert exc.value.code == 0
    captured = capsys.readouterr()
    assert "No options registered in this script." in captured.out


def test_show_options_displays_registered_options(capsys):
    """@lambkin.option entries are shown with name and default."""

    @benchmark(variants=[{}], num_iterations=1)
    @option("--clock-rate", default=100.0)
    def fn(ctx):
        pass

    with pytest.raises(SystemExit) as exc:
        fn(args=["--show-options"])

    assert exc.value.code == 0
    captured = capsys.readouterr()
    assert "--clock-rate" in captured.out
    assert "100.0" in captured.out
