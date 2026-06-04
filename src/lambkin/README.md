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

**Process Isolation with cgroups v2**
LAMBKIN uses cgroups v2 to track and clean up every process spawned during a benchmark run. Unlike process groups or sessions, a process cannot escape its cgroup by calling setsid or setpgid — the kernel enforces containment regardless of what the process does. This makes it the only reliable mechanism for cleaning up an entire process tree.
The cgroup hierarchy for a run looks like this:
```
app.slice/                               ← user's systemd app slice
└── lambkin-my_benchmark-a1b2c3d4/       ← one per CLI invocation
    └── iter-var_1-iter_1-e5f6a7b8/      ← one per (variant, iteration) pair
        ├── ros2-a9b0c1d2/               ← ros2 launch process
        └── ros2-e3f4a5b6/               ← ros2 bag record process
```
When running on the host, a user systemd app slice (app.slice) is always available. In containerized environments no app slice may exist — in that case, LAMBKIN falls back to the nearest delegated cgroup it can find. For example, under Podman with --systemd=always:
```
user.slice/user-1000.slice/user@1000.service/  ← delegated cgroup root
└── lambkin-my_benchmark-a1b2c3d4/
    └── iter-var_1-iter_1-e5f6a7b8/
        ├── my_algorithm-a9b0c1d2/
        └── my_recorder-e3f4a5b6/
```

Cleanup on exit. When a background() context exits normally, LAMBKIN sends SIGTERM to all processes in the cgroup, waits for a grace period, then sends SIGKILL to any survivors. On Ctrl-C, the CLI writes 1 to cgroup.kill, which the kernel propagates instantly to the entire tree.


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

LAMBKIN uses Python's standard logging module for its own informational messages. Subprocess output is handled separately through output redirection — each process can be configured independently with `log_output`.

**Output modes**

| Mode | Behavior |
|------|-----------|
| `"console"` | Routes subprocess stdout/stderr to the terminal |
| `"file"` | Writes subprocess output to a per-process log file under the iteration output directory |

Log files are named after the command and written to the iteration directory:
```
results/var_1/iter_1/
├── ros2_launch.stdout.log
├── ros2_launch.stderr.log
├── ros2_bag_record.stdout.log
└── ros2_bag_record.stderr.log
```

**Precedence** (highest to lowest)

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
