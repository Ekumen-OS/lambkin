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

Runs a process in the background while the benchmark continues executing. Takes a shell command (without calling it) and manages its full lifecycle — start, monitor, and clean up — as a context manager. When the context exits, it terminates the process and all its descendants; any process spawned outside a `background()` block is not covered.

> [!WARNING]
> Pass the command proxy to background() without calling it — `ctx.shell.my_tool`, not `ctx.shell.my_tool()`. Calling it with () runs the process immediately as a foreground blocking call and background() will raise an error.

## Process Management with cgroups v2

LAMBKIN places each iteration in its own cgroup, so every process spawned during that run — whether inside a background() block or not — is tracked and torn down unconditionally when the iteration ends. This means no leftover processes survive into the next iteration, and no process can escape by calling `setsid` or `setpgid` — the kernel enforces containment regardless.

The cgroup hierarchy for a run looks like this:
```
app.slice/                               ← user's systemd app slice
└── lambkin-my_benchmark-a1b2c3d4/       ← one per CLI invocation
    └── iter-var_1-iter_1-e5f6a7b8/      ← one per (variant, iteration) pair
        ├── my_algorithm-a9b0c1d2/       ← ros2 launch process
        └── my_recorder-e3f4a5b6/        ← ros2 bag record process
```
When running on the host, a user systemd app slice (app.slice) is always available. In containerized environments no app slice may exist — in that case, LAMBKIN falls back to the nearest delegated cgroup it can find. For example, under Podman with `--systemd=always`:
```
user.slice/user-1000.slice/user@1000.service/  ← delegated cgroup root
└── lambkin-my_benchmark-a1b2c3d4/
    └── iter-var_1-iter_1-e5f6a7b8/
        ├── my_algorithm-a9b0c1d2/
        └── my_recorder-e3f4a5b6/
```

> [!WARNING]
> The cgroup design provides process lifetime containment, not network isolation. If iterations were to run in parallel, processes from different iterations could still communicate with each other.

## Process Cleanup

When an iteration completes, LAMBKIN tears down the iteration cgroup by sending `SIGTERM` to all remaining processes, waiting for a grace period, then sending `SIGKILL` to any survivors. On Ctrl-C, the CLI writes 1 to `cgroup.kill`, which the kernel propagates instantly to the entire iteration cgroup.


## CLI

LAMBKIN exposes a lambkin command that runs your benchmark script inside a transient systemd cgroup scope, ensuring all child processes are tracked and cleaned up automatically.

```bash
Usage: lambkin [OPTIONS] SCRIPT [SDK_OPTIONS] [CUSTOM_OPTIONS]

LAMBKIN is a benchmarking SDK for robotics applications. It runs your
benchmark script inside a dedicated cgroup v2 scope, ensuring all child
processes are tracked and cleaned up automatically.


Options:
  --help  Show this message and exit.

SDK Options (always available):
  --dry-run       Run the benchmark in dry-run mode: commands are logged but
                  not executed.
  --show-options  List all options registered via @lambkin.option.
  --log-output    Where to route process output: file or console.
  --log-level     Log level for lambkin SDK output.

Custom Options (script-defined):
  Options registered in your benchmark script via @lambkin.option.
  Run 'lambkin SCRIPT --show-options' to list them.
```

## Logging

LAMBKIN has two independent logging systems: one for its own internal messages and one for subprocess output.

### SDK Logging

Controls the verbosity of LAMBKIN's own internal messages via the `--log-level` SDK option. Accepts any level supported by [Python's logging](https://docs.python.org/3/library/logging.html#logging-levels) module, case-insensitive:
```bash
lambkin my_benchmark.py --log-level debug
lambkin my_benchmark.py --log-level DEBUG  # equivalent
```

The LAMBKIN logger is fully isolated from the root logger — user scripts can configure their own logging without any interference.

### Process Logging

Controls where subprocess stdout and stderr are routed. Each process can be configured independently via log_output.
```bash
lambkin my_benchmark.py --log-output console
```

| Mode | Behavior |
|------|-----------|
| `"console"` | Routes subprocess stdout/stderr to the terminal |
| `"file"` | Writes subprocess output to a per-process log file under the iteration output directory |

Log files are named after the command and written to the iteration directory:
```
results/var_1/iter_1/
├── my_algorithm.stdout.log
├── my_algorithm.stderr.log
├── my_recorder.stdout.log
└── my_recorder.stderr.log
```

Precedence (highest to lowest):

1. **CLI option** — `--log-output` flag passed to the `lambkin` command
2. **Per-call** — `log_output` keyword at the call site
3. **ShellProxy default** — `ShellProxy(log_output="file")`

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
@lambkin.option("--clock-rate", default=1.0)
def my_benchmark(ctx):
    with lambkin.process.background(ctx.shell.my_recorder, "-o", "output.mcap"):
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
Run it with the CLI:

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
        ├── lambkin_metadata.yaml
        ├── output.mcap
        ├── out.zip
        ├── my_algorithm.stdout.log
        ├── my_algorithm.stderr.log
        ├── my_recorder.stdout.log
        └── my_recorder.stderr.log
```
