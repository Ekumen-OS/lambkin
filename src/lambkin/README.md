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

**Named Product**

Takes named parameter lists and returns every possible combination as a list of dictionaries, one per benchmark run configuration. Pass the result to `Benchmark` via `variants=` to sweep all combinations automatically.

**Benchmark**

Drives the benchmark execution loop, handling iteration, parameter expansion, and context setup. It parses options registered via `Option` once before the loop, then creates a `Context` for every combination of variant and iteration and calls the decorated function with it. A base context is used during setup to resolve inputs before the loop begins. Can be used as a decorator via `@benchmark`.

**Context**

Carries all namespaced information for one benchmark variant and iteration. Holds configuration, resolved inputs, options, and output paths for the current run, and automatically creates the required output folders on disk before the benchmark function runs.

**Source**

Describes the benchmark script being executed. Exposed through the context, it gives benchmark stages access to the script's location and metadata without hardcoding paths.

**Input**

Registers a data resolution hook on a benchmark function. Each hook must be a callable that accepts a single `Context` object as its argument. Hook names must be unique — registering two hooks with the same name raises an error. Hooks are resolved once before the execution loop and their return values injected into the context under the hook's function name. Can be used as a decorator via `@input`.

**Option**

Registers a CLI option on a benchmark. Built on top of [`click`](https://click.palletsprojects.com/en/stable/options/), so any attribute supported by `click.Option` can be passed. Flag names must start with `-` or `--`. Declared options are collected and parsed once before the execution loop, and their values made available through the context. Can be used as a decorator via `@option`.

**Shell**

Abstracts shell command dispatch. Exposes the host environment's executables as Python attributes — accessing `shell.my_tool` returns a callable that runs `my_tool` with the given arguments, letting benchmark scripts invoke external processes without hardcoding paths or constructing subprocess calls manually. Accessible through the context.

**Background Process**

A context manager that wraps a Shell command and manages its full lifecycle — start, monitor, and clean up — ensuring no orphaned processes survive when the benchmark ends or is interrupted. Uses cgroups v2 to guarantee kernel-level cleanup of the entire process tree, including descendants that have detached via setsid or setpgid. Used via lambkin.process.background(...)

## CLI

LAMBKIN exposes a lambkin command that runs your benchmark script inside a transient systemd cgroup scope, ensuring all child processes are tracked and cleaned up automatically.

```bash
Usage: lambkin [OPTIONS] SCRIPT [SDK_OPTIONS] [CUSTOM_OPTIONS]

Options:
  --help          Show this message and exit.

SDK Options (always available):
  --dry-run       Run the benchmark in dry-run mode: commands are logged but
                  not executed.
  --show-options  List all SDK and custom options available for this benchmark
                  script and exit.

Custom Options (script-defined):
  Options registered in your benchmark script via @lambkin.option.
  Run 'lambkin SCRIPT --show-options' to list them.
```

## Logging
LAMBKIN uses Python's standard logging module for its own informational messages. Subprocess output is handled separately through output redirection — each process can be configured independently with _log_output.
Three output modes are supported:

* "console" — route subprocess stdout/stderr to the terminal.
* "file" — write subprocess output to a per-process log file under the iteration output directory.

The mode can be set at three levels, applied in precedence order:

* Per-call — _log_output keyword at the call site, intercepted by LAMBKIN and never forwarded to the process.
* Benchmark option — via @lambkin.option("--log-output", default="file").
Environment variable — LAMBKIN_LOG_OUTPUT=both.
ShellProxy default — ShellProxy(log_output="file").

## Requirements

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv)
- Linux with cgroups v2 and systemd (required for background process management)

## Installation

```bash
git clone -b next-gen git@github.com:Ekumen-OS/lambkin.git
uv sync
```


## Usage

A minimal example composing the SDK primitives described above into a working benchmark.

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
**Run it with the CLI:**

```bash
lambkin my_benchmark.py --clock-rate 50.0
```

Inspect all available options without running:
```bash
lambkin my_benchmark.py --show-options
```

Validate the benchmark pipeline without executing any process:
```bash
lambkin my_benchmark.py --dry-run
```
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
