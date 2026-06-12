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

"""Unit tests for BenchmarkContext, VariantContext, and IterationContext."""

from pathlib import Path

import pytest
import yaml

from lambkin.core.ctx.benchmark_context import BenchmarkContext
from lambkin.core.ctx.iteration_context import IterationContext
from lambkin.core.ctx.source import Source
from lambkin.core.ctx.variant_context import VariantContext


@pytest.fixture
def source(tmp_path):
    """A Source pointing to this test file."""
    return Source(path=Path(__file__))


@pytest.fixture
def base_options():
    """Minimal options dict that avoids cgroups and marks nothing as completed."""
    return {
        "dry_run": True,
        "no_cache": False,
        "log_output": "file",
        "log_level": "info",
    }


@pytest.fixture
def variant():
    """A simple variant dict."""
    return {"sensor_model": "beam", "num_particles": 10}


@pytest.fixture
def bctx(source, base_options, tmp_path):
    """A BenchmarkContext, not entered."""
    return BenchmarkContext(source=source, options=base_options, base_dir=tmp_path)


@pytest.fixture
def vctx(bctx, variant):
    """An entered VariantContext parented to bctx, variant_index=0."""
    with VariantContext(benchmark_ctx=bctx, variant=variant, variant_index=0) as ctx:
        yield ctx


class TestBenchmarkContext:
    """Tests for BenchmarkContext lifecycle and properties."""

    def test_base_dir_is_created_on_enter(self, tmp_path, source, base_options):
        """__enter__ creates the base directory on disk."""
        target = tmp_path / "results"
        assert not target.exists()
        with BenchmarkContext(source=source, options=base_options, base_dir=target):
            assert target.exists()

    def test_source_property(self, bctx, source):
        """Source property returns the Source passed at construction."""
        assert bctx.source is source

    def test_options_are_namespaced(self, bctx):
        """Options exposes dict keys as attributes."""
        assert bctx.options.dry_run is True
        assert bctx.options.log_level == "info"

    def test_base_dir_property(self, tmp_path, bctx):
        """base_dir property returns a Path equal to the one passed in."""
        assert bctx.base_dir == tmp_path

    def test_exit_is_noop(self, tmp_path, source, base_options):
        """__exit__ does not raise and does not remove the directory."""
        with BenchmarkContext(source=source, options=base_options, base_dir=tmp_path):
            pass
        assert tmp_path.exists()


class TestVariantContext:
    """Tests for VariantContext lifecycle and delegation."""

    def test_variant_dir_is_created_on_enter(self, bctx, variant):
        """__enter__ creates the var_N directory."""
        with VariantContext(
            benchmark_ctx=bctx, variant=variant, variant_index=0
        ) as vctx:
            assert vctx.variant_dir.exists()
            assert vctx.variant_dir.name == "var_1"

    def test_variant_index_controls_folder_name(self, bctx, variant):
        """variant_index determines the var_N folder name."""
        with VariantContext(
            benchmark_ctx=bctx, variant=variant, variant_index=2
        ) as vctx:
            assert vctx.variant_dir.name == "var_3"

    def test_variant_attributes(self, vctx):
        """Variant exposes dict keys as attributes."""
        assert vctx.variant.sensor_model == "beam"
        assert vctx.variant.num_particles == 10

    def test_variant_index_property(self, vctx):
        """variant_index returns the zero-based index."""
        assert vctx.variant_index == 0

    def test_source_delegated_to_benchmark_context(self, vctx, source):
        """Source is delegated to BenchmarkContext."""
        assert vctx.source is source

    def test_options_delegated_to_benchmark_context(self, vctx):
        """Options is delegated to BenchmarkContext."""
        assert vctx.options.dry_run is True

    def test_base_dir_delegated_to_benchmark_context(self, vctx, tmp_path):
        """base_dir is delegated to BenchmarkContext."""
        assert vctx.base_dir == tmp_path

    def test_exit_is_noop(self, bctx, variant):
        """__exit__ does not raise and does not remove the variant directory."""
        with VariantContext(
            benchmark_ctx=bctx, variant=variant, variant_index=0
        ) as vctx:
            pass
        assert vctx.variant_dir.exists()


class TestIterationContext:
    """Tests for IterationContext cache, metadata, and delegation."""

    def test_output_dirs_created_on_enter(self, vctx):
        """__enter__ creates both variant_dir and iteration_dir on disk."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.paths.variant_dir.exists()
            assert ctx.paths.iteration_dir.exists()

    def test_iteration_dir_name(self, vctx):
        """iteration_dir name follows iter_N pattern (1-based)."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.paths.iteration_dir.name == "iter_1"
        with IterationContext(variant_ctx=vctx, iteration=4) as ctx:
            assert ctx.paths.iteration_dir.name == "iter_5"

    def test_iteration_property(self, vctx):
        """Iteration returns the zero-based index."""
        with IterationContext(variant_ctx=vctx, iteration=3) as ctx:
            assert ctx.iteration == 3

    def test_variant_delegated_to_variant_context(self, vctx):
        """Variant is delegated to VariantContext."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.variant.sensor_model == "beam"
            assert ctx.variant.num_particles == 10

    def test_variant_index_delegated_to_variant_context(self, vctx):
        """variant_index is delegated to VariantContext."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.variant_index == 0

    def test_options_delegated_via_variant_context(self, vctx):
        """Options is delegated through VariantContext to BenchmarkContext."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.options.dry_run is True

    def test_source_delegated_via_variant_context(self, vctx, source):
        """Source is delegated through VariantContext to BenchmarkContext."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.source is source

    def test_inputs_defaults_to_none(self, vctx):
        """Inputs is None when not provided."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.inputs is None

    def test_skipped_is_false_on_cache_miss(self, vctx):
        """Skipped is False on first run."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx.skipped is False

    def test_shell_raises_before_enter(self, vctx):
        """Accessing shell before __enter__ raises AttributeError."""
        ctx = IterationContext(variant_ctx=vctx, iteration=0)
        with pytest.raises(AttributeError):
            _ = ctx.shell

    def test_metadata_written_on_enter(self, vctx):
        """Metadata file is written on context entry."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            assert ctx._metadata_path.exists()
            content = yaml.safe_load(ctx._metadata_path.read_text())
            assert "started_at" in content
            assert "run_hash" in content
            assert "variant_index" in content

    def test_completed_at_not_written_on_dry_run(self, vctx):
        """dry_run=True never writes completed_at."""
        with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
            pass
        content = yaml.safe_load(ctx._metadata_path.read_text())
        assert "completed_at" not in content

    def test_completed_at_written_on_clean_exit(self, tmp_path, source):
        """completed_at is written after a successful non-dry-run exit."""
        options = {
            "dry_run": False,
            "no_cache": False,
            "log_output": "file",
            "log_level": "info",
        }
        with BenchmarkContext(
            source=source, options=options, base_dir=tmp_path
        ) as bctx:
            with VariantContext(
                benchmark_ctx=bctx, variant={"x": 1}, variant_index=0
            ) as vctx:
                with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
                    pass
        content = yaml.safe_load(ctx._metadata_path.read_text())
        assert "completed_at" in content

    def test_completed_at_not_written_on_exception(self, tmp_path, source):
        """completed_at is not written when an exception propagates."""
        options = {
            "dry_run": False,
            "no_cache": False,
            "log_output": "file",
            "log_level": "info",
        }
        with pytest.raises(RuntimeError):
            with BenchmarkContext(
                source=source, options=options, base_dir=tmp_path
            ) as bctx:
                with VariantContext(
                    benchmark_ctx=bctx, variant={"x": 1}, variant_index=0
                ) as vctx:
                    with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
                        raise RuntimeError("simulated failure")
        content = yaml.safe_load(ctx._metadata_path.read_text())
        assert "completed_at" not in content

    def test_skipped_is_true_on_cache_hit(self, tmp_path, source):
        """Skipped is True on second run with same inputs and dry_run=False."""
        options = {
            "dry_run": False,
            "no_cache": False,
            "log_output": "file",
            "log_level": "info",
        }
        variant = {"x": 1}
        with BenchmarkContext(
            source=source, options=options, base_dir=tmp_path
        ) as bctx:
            with VariantContext(
                benchmark_ctx=bctx, variant=variant, variant_index=0
            ) as vctx:
                with IterationContext(variant_ctx=vctx, iteration=0):
                    pass
                with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
                    assert ctx.skipped is True

    def test_no_cache_bypasses_cache(self, tmp_path, source):
        """no_cache=True forces rerun even when iteration already completed."""
        options_normal = {
            "dry_run": False,
            "no_cache": False,
            "log_output": "file",
            "log_level": "info",
        }
        options_no_cache = {**options_normal, "no_cache": True}
        variant = {"x": 1}

        with BenchmarkContext(
            source=source, options=options_normal, base_dir=tmp_path
        ) as bctx:
            with VariantContext(
                benchmark_ctx=bctx, variant=variant, variant_index=0
            ) as vctx:
                with IterationContext(variant_ctx=vctx, iteration=0):
                    pass

        with BenchmarkContext(
            source=source, options=options_no_cache, base_dir=tmp_path
        ) as bctx:
            with VariantContext(
                benchmark_ctx=bctx, variant=variant, variant_index=0
            ) as vctx:
                with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
                    assert ctx.skipped is False

    def test_leftover_artifacts_cleaned_on_rerun(self, tmp_path, source):
        """Stale files from a failed run are removed on the next entry."""
        options = {
            "dry_run": True,
            "no_cache": False,
            "log_output": "file",
            "log_level": "info",
        }
        variant = {"x": 1}

        with pytest.raises(RuntimeError):
            with BenchmarkContext(
                source=source, options=options, base_dir=tmp_path
            ) as bctx:
                with VariantContext(
                    benchmark_ctx=bctx, variant=variant, variant_index=0
                ) as vctx:
                    with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
                        stale = ctx.paths.iteration_dir / "stale_output.txt"
                        stale.touch()
                        raise RuntimeError("simulated failure")

        assert stale.exists()

        with BenchmarkContext(
            source=source, options=options, base_dir=tmp_path
        ) as bctx:
            with VariantContext(
                benchmark_ctx=bctx, variant=variant, variant_index=0
            ) as vctx:
                with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
                    assert not (ctx.paths.iteration_dir / "stale_output.txt").exists()

    def test_from_params_constructs_correctly(
        self, tmp_path, source, base_options, variant
    ):
        """from_params builds the full parent chain from raw parameters."""
        ctx = IterationContext.from_params(
            variant=variant,
            variant_index=1,
            iteration=2,
            options=base_options,
            source=source,
            base_dir=tmp_path,
        )
        assert ctx.iteration == 2
        assert ctx.variant_index == 1
        assert ctx.variant.sensor_model == "beam"
        assert ctx.paths.variant_dir.name == "var_2"
        assert ctx.paths.iteration_dir.name == "iter_3"

    def test_from_params_can_be_used_as_context_manager(
        self, tmp_path, source, base_options, variant
    ):
        """from_params result works as a context manager."""
        ctx = IterationContext.from_params(
            variant=variant,
            variant_index=0,
            iteration=0,
            options=base_options,
            source=source,
            base_dir=tmp_path,
        )
        with ctx as entered:
            assert entered is ctx
            assert ctx.paths.iteration_dir.exists()

    def test_shell_raises_on_cache_hit(self, tmp_path, source):
        """Accessing shell on a cache hit raises AttributeError."""
        options = {
            "dry_run": False,
            "no_cache": False,
            "log_output": "file",
            "log_level": "info",
        }
        variant = {"x": 1}

        with BenchmarkContext(
            source=source, options=options, base_dir=tmp_path
        ) as bctx:
            with VariantContext(
                benchmark_ctx=bctx, variant=variant, variant_index=0
            ) as vctx:
                with IterationContext(variant_ctx=vctx, iteration=0):
                    pass
                with IterationContext(variant_ctx=vctx, iteration=0) as ctx:
                    assert ctx.skipped is True
                    with pytest.raises(AttributeError):
                        _ = ctx.shell
