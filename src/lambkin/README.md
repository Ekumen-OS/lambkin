# LAMBKIN SDK

The LAMBKIN Python SDK is the core library for building SLAM evaluation pipelines. It provides the orchestration primitives you compose into your benchmark: process lifecycle management, parameter sweep execution, pipeline stages, and structured I/O.

## Architecture

A benchmark is structured around three stages that LAMBKIN sequences and keeps organized:

1. **Ingestion** — locate or download datasets and prepare inputs.
2. **Execution** — launch and supervise the processes that run your algorithm.
3. **Egression** — collect outputs and compute metrics.

Each stage is a Python callable that receives a context object carrying configuration, paths, and state. You implement the logic; LAMBKIN handles the rest.

## Core Concepts

LAMBKIN exposes a small set of composable primitives. Together they cover the full lifecycle of a benchmark — from declaring inputs and sweeping parameters to launching processes and collecting results.

**`named_product`**
Takes named parameter lists and returns every possible combination as a list of dictionaries, one per benchmark run configuration. Pass the result to `Benchmark` via `variants=` to sweep all combinations automatically.

**`Benchmark`**
Drives the benchmark execution loop. It parses options registered via `Option` once before the loop, then creates a `Context` for every `(variant, iteration)` pair and calls the decorated function with it. A base context is used during setup to resolve inputs before the loop begins. Can be used as a decorator via `@benchmark`.

**`Context`**
Carries all namespaced information for one benchmark variant. Builds `ctx.variant`, `ctx.inputs`, `ctx.options`, and `ctx.output` from the given parameters, and automatically creates the required output folders on disk before the benchmark function runs.

**`Source`**
Describes the benchmark script being executed. Exposed on the context as `ctx.source`, it gives benchmark stages access to the script's location and metadata without hardcoding paths.

**`Input`**
Registers a data resolution hook on a benchmark. Hooks are resolved before each iteration and their return values injected into `ctx.inputs` under the hook's function name, keeping data resolution decoupled from benchmark logic. Can be used as a decorator via `@input`.

**`Option`**
Abstracts shell command dispatch. Exposes the host environment's executables as Python attributes — accessing `shell.my_tool` returns a callable that runs `my_tool` with the given arguments, letting benchmark scripts invoke external processes without hardcoding paths or constructing subprocess calls manually. Accessible via `ctx.shell`.

**`ctx.shell`**
Exposes the host environment's executables as Python attributes. Accessing `ctx.shell.my_tool` returns a callable that, when invoked, runs `my_tool` with the given arguments. This lets benchmark scripts call external processes as if they were native Python functions, without hardcoding paths or constructing subprocess calls manually.


## Requirements

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv)

## Installation

```bash
git clone git@github.com:Ekumen-OS/lambkin.git
uv sync
```


## Usage

A benchmark is a decorated Python function. The @lambkin.benchmark decorator handles iteration, parameter expansion, and context setup. Inputs and outputs are registered as hooks on the benchmark function.

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

if __name__ == "__main__":
    my_benchmark()

```
> **Note:** Real execution is not yet supported. `--dry-run` is the only supported mode at this time. The shell proxy prints commands rather than running them.
> Background process orchestration will be implemented in subsequent phases.

For a complete, working example using the Beluga algorithm, see [`Beluga Example`](examples/beluga/beluga_benchmark.py).

## Expected Output

LAMBKIN writes all artifacts under a consistent directory tree:
```
results/
└── <variant_n>/
    └── iter_<n>/
        ├── output.mcap
        ├── out.zip
        └── ...
```
