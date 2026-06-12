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

"""Integration test: scoped input hooks resolve at the correct lifecycle level."""

from lambkin.core.decorators.benchmark import benchmark


def test_scoped_inputs_resolve_at_correct_lifecycle(tmp_path):
    """Benchmark, variant, and iteration inputs resolve at the right scope.

    - Benchmark-scoped hooks run once before the loop.
    - Variant-scoped hooks run once per variant, with access to ctx.variant.
    - Iteration-scoped hooks run once per cache-miss iteration.
    All three scopes are merged and accessible via ctx.inputs inside fn(ctx).
    """
    benchmark_calls = []
    variant_calls = []
    iteration_calls = []
    seen_inputs = []

    variants = [
        {"sensor_model": "beam", "dataset": "dataset1.mcap"},
        {"sensor_model": "likelihood", "dataset": "dataset2.mcap"},
    ]
    num_iterations = 2

    @benchmark(variants=variants, num_iterations=num_iterations)
    def nominal(ctx):
        seen_inputs.append((
            ctx.variant.sensor_model,
            ctx.inputs.shared,
            ctx.inputs.variant_dataset,
            ctx.inputs.seed,
        ))

    @nominal.input
    def shared(ctx):
        benchmark_calls.append(1)
        return "ground_truth.tum"

    @nominal.input(scope="variant")
    def variant_dataset(ctx):
        variant_calls.append(ctx.variant.sensor_model)
        return ctx.variant.dataset

    @nominal.input(scope="iteration")
    def seed(ctx):
        iteration_calls.append(ctx.iteration)
        return f"seed_{ctx.iteration}"

    nominal(args=["--dry-run"], base_dir=tmp_path)

    # Benchmark scope — called exactly once before the loop
    assert len(benchmark_calls) == 1

    # Variant scope — called once per variant, with correct variant each time
    assert len(variant_calls) == len(variants)
    assert variant_calls == ["beam", "likelihood"]

    # Iteration scope — called once per (variant, iteration) pair
    assert len(iteration_calls) == len(variants) * num_iterations

    # All three scopes merged correctly and accessible inside fn(ctx)
    assert seen_inputs[0] == ("beam", "ground_truth.tum", "dataset1.mcap", "seed_0")
    assert seen_inputs[1] == ("beam", "ground_truth.tum", "dataset1.mcap", "seed_1")
    assert seen_inputs[2] == (
        "likelihood",
        "ground_truth.tum",
        "dataset2.mcap",
        "seed_0",
    )
    assert seen_inputs[3] == (
        "likelihood",
        "ground_truth.tum",
        "dataset2.mcap",
        "seed_1",
    )
