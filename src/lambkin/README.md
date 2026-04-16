# LAMBKIN SDK

The LAMBKIN Python SDK is the core library for building SLAM evaluation
pipelines. It provides the orchestration primitives you compose into your
benchmark: process lifecycle management, parameter sweep execution, pipeline
stages, and structured I/O.

## Structure

A benchmark is structured around three stages that LAMBKIN sequences,
parallelizes, and keeps organized:

1. **Ingestion** — locate or download datasets and prepare inputs.
2. **Execution** — launch and supervise the processes that run your algorithm.
3. **Egression** — collect outputs and compute metrics.

Each stage is a Python callable that receives a context object carrying
configuration, paths, and state. You implement the logic; LAMBKIN handles
the rest.

LAMBKIN writes all artifacts under a consistent directory tree:
```
results/
└── <variant_n>/
    └── iter_<n>/
        ├── output.mcap
        ├── out.zip
        └── ...
```

## Usage

A benchmark is a decorated Python function. The `@lambkin.benchmark`
decorator handles iteration, parameter expansion, and context setup.
Inputs and outputs are registered as hooks on the benchmark function.

```python
import lambkin

@lambkin.benchmark(
    variants=lambkin.common.named_product(
        param_a=["x", "y"],
        param_b=[1, 10, 100],
    ),
    num_iterations=5,
)
@lambkin.option("--clock-rate", default=100.0)
def my_benchmark(ctx):
    with lambkin.process.background(ctx.shell.my_recorder, "-O", "output.mcap"):
        with lambkin.process.background(
            ctx.shell.my_algorithm,
            f"param_a:={ctx.variant.param_a}",
            f"param_b:={ctx.variant.param_b}",
            f"input:={ctx.inputs.dataset}",
        ):
            ctx.shell.my_player(ctx.inputs.dataset, "-r", ctx.options.clock_rate)

@my_benchmark.input
def dataset(ctx):
    return ctx.source.path.parent / "datasets" / "my_dataset.mcap"


```

> **Note:** Real execution is not yet supported. `--dry-run` is the only
> supported mode at this time. The shell proxy prints commands rather than
> running them.

For a complete, working example using the Beluga algorithm, see
[`Beluga Example`](examples/beluga/beluga_benchmark.py).

## Requirements

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv)

## Installation

```bash
git clone git@github.com:Ekumen-OS/lambkin.git
uv sync
```
