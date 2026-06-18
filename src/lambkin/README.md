# LAMBKIN SDK

The LAMBKIN Python SDK is the core library for building SLAM evaluation pipelines. It provides the orchestration primitives you compose into your benchmark: process lifecycle management, parameter sweep execution, pipeline stages, and structured I/O.


## Table of Contents

- [Architecture](#architecture)
- [Core Concepts](#core-concepts)
- [Process Management with cgroups v2](#process-management-with-cgroups-v2)
- [CLI](#cli)
- [Partial Restarts](#partial-restarts)
- [Logging](#logging)
- [Reading Results](#reading-results)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Use Cases](#use-cases)
- [Expected Output](#expected-output)

## Architecture

A benchmark is structured around three moments that LAMBKIN sequences and keeps organized:

1. **Setup** — input hooks resolve datasets, maps, or any other dependency your benchmark needs, at the scope where they belong (once for the whole run, once per variant, or once per iteration).
2. **Execution** — your benchmark function runs once per `(variant, iteration)` pair, with a dedicated context, cgroup, and shell to launch and supervise external processes.
3. **Result access** — once every iteration finishes, output hooks run once against the full results tree, free to aggregate, plot, or log metrics using `lambkin.data`.

Each stage is a plain Python function that receives a context object carrying configuration, paths, and state. You implement the logic; LAMBKIN handles the rest.

## Core Concepts

LAMBKIN exposes a small set of composable primitives. Together they cover the full lifecycle of a benchmark — from declaring inputs and sweeping parameters, to launching processes and reading results back.

**Named Product**

Takes named parameter lists and returns every possible combination as a list of dictionaries, one per benchmark run configuration. Pass the result to `Benchmark` via `variants=` to sweep all combinations automatically.

**Benchmark**

Drives the benchmark execution loop, handling iteration, parameter expansion, and context setup. It parses options registered via `Option` once before the loop, then creates a context for every combination of variant and iteration and calls the decorated function with it. Can be used as a decorator via `@benchmark`.

**Context**

Carries all namespaced information for a benchmark run, split into three nested scopes that mirror the execution loop:

- `BenchmarkContext` — lives for the entire run. Holds `source`, `options`, `base_dir`, and benchmark-scoped `inputs`.
- `VariantContext` — lives for one variant sweep. Adds `variant`, `variant_index`, `variant_dir`, and inputs merged with the benchmark scope.
- `IterationContext` — lives for one `(variant, iteration)` pair, and is what your benchmark function receives as `ctx`. Adds `iteration`, `paths` (with `base_dir`, `variant_dir`, `iteration_dir`), `shell`, `skipped`, and inputs merged with both parent scopes.

Each level is a context manager: entering it creates the corresponding output directory on disk, and exiting it tears down anything it owns — the iteration's cgroup, in the innermost case. `ctx.inputs` is read-only once resolved; assigning to it a second time raises `AttributeError`.

**Source**

Describes the benchmark script being executed. Exposed through the context as `ctx.source`, it gives benchmark stages access to the script's own location (`ctx.source.path`) without hardcoding paths or relying on `__file__`, which would point at the SDK rather than the user's script.

**Input**

Registers a data resolution hook on a benchmark function. Each hook must be a callable that accepts a single `Context` object as its argument and must return a non-empty value. Hook names must be unique across the entire benchmark — registering two hooks with the same name, even under different scopes, raises an error. Can be used as a decorator via `@input`.

Hooks can be scoped to one of three lifecycle levels via `scope=`:

- `"benchmark"` (default) — resolved once before the variant loop. Use for inputs that don't depend on the current variant or iteration, e.g. a shared dataset.
- `"variant"` — resolved once per variant. Use for inputs that depend on `ctx.variant` but not `ctx.iteration`, e.g. selecting a dataset file by sensor model.
- `"iteration"` — resolved once per iteration, only on a cache miss. Use for inputs that depend on both `ctx.variant` and `ctx.iteration`, e.g. a per-iteration random seed.

```python
@nominal.input
def dataset(ctx): ...  # benchmark scope (default)


@nominal.input(scope="variant")
def calibration(ctx): ...  # variant scope
```

Resolved inputs are merged down the hierarchy, so a variant-scoped hook can rely on benchmark-scoped inputs already being available on `ctx.inputs`, and so on for iteration scope.

**Output**

Registers a function as a callback that runs once, after the entire benchmark loop completes, receiving the benchmark-scoped context. Hook names must be unique. Can be used as a decorator via `@output`.

> [!WARNING]
> Output hooks receive a `BenchmarkContext`, not an `IterationContext` — every iteration's cgroup and shell have already been torn down by the time hooks run. Don't call `ctx.shell` or launch any process inside an output hook; read artifacts from disk via `ctx.base_dir` instead (see [Reading Results](#reading-results)).

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

### Process Cleanup

When an iteration completes, LAMBKIN tears down the iteration cgroup by sending `SIGTERM` to all remaining processes, waiting for a grace period, then sending `SIGKILL` to any survivors. On Ctrl-C, the CLI writes 1 to `cgroup.kill`, which the kernel propagates instantly to the entire iteration cgroup.

## CLI

LAMBKIN exposes a `lambkin` command that runs your benchmark script inside a transient systemd cgroup scope, ensuring all child processes are tracked and cleaned up automatically.

```bash
Usage: lambkin [OPTIONS] SCRIPT [SDK_OPTIONS] [CUSTOM_OPTIONS]

LAMBKIN is a benchmarking SDK for robotics applications. It runs your
benchmark script inside a dedicated cgroup v2 scope, ensuring all child
processes are tracked and cleaned up automatically.

Options:
  --help  Show this message and exit.

SDK Options (always available):
  --dry-run        Run the benchmark in dry-run mode: commands are logged but
                   not executed.
  --show-options   List all options registered via @lambkin.option.
  --log-output     Where to route process output: file or console.
  --log-level      Log level for lambkin SDK output.
  --show-variants  Print all variants with their var_N number and exit.
  --variants       Comma-separated variant numbers or ranges to run, matching
                   var_N folder names (e.g. --variants 3:5,42 runs var_3/,
                   var_4/, var_5/, var_42/). Defaults to all variants.
  --no-cache       Bypass the partial restart cache and force a full rerun of
                   all iterations, regardless of prior completion.

Custom Options (script-defined):
  Options registered in your benchmark script via @lambkin.option.
  Run 'lambkin SCRIPT --show-options' to list them.
```

> [!WARNING]
> The SDK option names above are reserved. Declaring any of them via `@lambkin.option` in your script raises a Click error at parse time due to duplicate parameter names.

`--show-options` and `--show-variants` behave like `--help`: they print information and exit immediately, never reaching the benchmark body.

### Selecting a subset of variants

Useful to retry a failed configuration, or to split a long sweep across machines.

```bash
# List every variant with its var_N number, without running anything.
lambkin my_benchmark.py --show-variants

# Run only variants 1, 3, 4 and 5.
lambkin my_benchmark.py --variants 1,3:5
```

Variant numbers always refer to the full, original sweep — they don't shift when you select a subset, so the output folder for a given configuration (`var_N`) stays stable across runs.

## Partial Restarts

LAMBKIN can skip iterations that already completed successfully in a previous run, so an interrupted or partially failed benchmark can be resumed without redoing finished work.

Each `(variant, iteration)` pair is identified by a stable hash derived from the variant parameters, the iteration index, and the script's custom options. SDK-level flags (such as `--log-level` or `--dry-run`) are excluded from the hash, since they don't affect benchmark outputs. Right before entering an iteration, LAMBKIN reads `lambkin_metadata.yaml` from the expected output folder: if `completed_at` is present and the stored hash matches the one just computed, the iteration is skipped entirely — no folders, cgroup, or shell are touched — and a log line reports the cache hit. Changing any variant parameter or custom option invalidates only the affected iterations; everything else is left alone.

Pass `--no-cache` to bypass this check and force a full rerun of every iteration, regardless of prior completion.

> [!NOTE]
> Iterations run with `--dry-run` are never marked as completed, since no real work happens. Re-running a dry-run script will show no cache hits every time — that's expected, not a bug.

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

Controls where subprocess stdout and stderr are routed. Each process can be configured independently via `log_output`.

```bash
lambkin my_benchmark.py --log-output console
```

| Mode | Behavior |
|------|-----------|
| `"console"` | Routes subprocess stdout/stderr to the terminal |
| `"file"` | Writes subprocess output to a per-process log file under the iteration output directory |

Log files are named after the command and written to the iteration directory:

```bash
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

## Reading Results

After the benchmark loop finishes, `@my_benchmark.output` hooks run once against the full results tree, so they're the natural place to aggregate and report on the data every iteration wrote to disk. `lambkin.data` provides the primitives for that, and works equally well from a Jupyter notebook with no live benchmark running — all it needs is a path.

- **`lambkin.data.access.iterations(source)`** — walks `results/var_*/iter_*/`, skips any iteration that didn't complete, and returns one entry per completed iteration with `iter_dir`, `variant` (e.g. `"var_1"`), `iteration`, and `params` (the variant's parameters as a `SimpleNamespace`). Accepts either a context-like object exposing `.base_dir`, or a plain path/string.
- **`lambkin.data.evo.series(source, filename)`** — same traversal, plus loads the `evo` result file (e.g. `"output.ape.zip"`) from each iteration directory and exposes `time`, `error`, and `distance` arrays, ready to plot.
- **`lambkin.data.evo.stats(source, filename)`** — same traversal, but exposes the aggregate statistics `evo` computes for each result: `rmse`, `mean`, `median`, `std`, `min`, `max`, `sse`.

```python
import matplotlib.pyplot as plt
import lambkin


@my_benchmark.output
def plots(ctx):
    for entry in lambkin.data.evo.series(ctx, "output.ape.zip"):
        plt.plot(
            entry.time, entry.error, label=f"{entry.variant} / iter {entry.iteration}"
        )
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()
    plt.savefig(ctx.base_dir / "plots.png")


@my_benchmark.output
def stats(ctx):
    for entry in lambkin.data.evo.stats(ctx, "output.ape.zip"):
        lambkin.logger.info(
            "%s iter %d: rmse=%.4f mean=%.4f max=%.4f",
            entry.variant,
            entry.iteration,
            entry.rmse,
            entry.mean,
            entry.max,
        )
```

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

For a complete, working example using the Beluga algorithm, see [`Beluga Example`](../../examples/beluga/beluga_benchmark.py).

## Use Cases

[Core Concepts](#core-concepts) describes what each primitive does in isolation. This section composes them into complete, working patterns for common benchmarking needs.

### Shared dataset across all variants

The default case: one dataset, reused by every variant and iteration. Resolve it once, at benchmark scope, so it isn't re-fetched on every run.

```python
@lambkin.benchmark(
    variants=lambkin.common.named_product(num_particles=[10, 100, 1000]),
    num_iterations=10,
)
def nominal(ctx):
    ctx.shell.my_algorithm(
        f"num_particles:={ctx.variant.num_particles}", f"input:={ctx.inputs.dataset}"
    )


@nominal.input
def dataset(ctx):
    return ctx.source.path.parent / "datasets" / "magazino.mcap"
```

### A different dataset per variant

When the input itself depends on the variant — e.g. a different sensor model needs a different recording — scope the hook to `"variant"`, so it only re-resolves when the variant changes, not on every iteration.

```python
@lambkin.benchmark(
    variants=lambkin.common.named_product(sensor_model=["beam", "likelihood_field"]),
    num_iterations=5,
)
def nominal(ctx):
    ctx.shell.my_algorithm(
        f"sensor_model:={ctx.variant.sensor_model}", f"input:={ctx.inputs.dataset}"
    )


@nominal.input(scope="variant")
def dataset(ctx):
    return ctx.source.path.parent / "datasets" / f"{ctx.variant.sensor_model}.mcap"
```

### A per-iteration input, e.g. a random seed

When the input must change on every single run — not just every variant — scope it to `"iteration"`. It's only re-resolved on a cache miss, so a completed iteration won't recompute it on a later restart.

```python
@nominal.input(scope="iteration")
def seed(ctx):
    return ctx.variant_index * 1000 + ctx.iteration


def nominal(ctx):
    ctx.shell.my_algorithm(f"seed:={ctx.inputs.seed}")
```

### Comparing algorithms or datasets as separate dimensions

`named_product` doesn't care whether a parameter is a tuning knob or a completely different code path — both end up as a value on `ctx.variant`. Branch inside the benchmark function on whichever values mean "different algorithm" or "different dataset":

```python
@lambkin.benchmark(
    variants=lambkin.common.named_product(
        algorithm=["beluga_amcl", "my_other_localizer"],
        dataset=["magazino", "warehouse_b"],
    ),
    num_iterations=10,
)
def nominal(ctx):
    ctx.shell.ros2.launch(
        "my_package",
        f"{ctx.variant.algorithm}.launch.py",
        f"input:={getattr(ctx.inputs, ctx.variant.dataset)}",
    )


@nominal.input
def magazino(ctx):
    return ctx.source.path.parent / "datasets" / "magazino.mcap"


@nominal.input
def warehouse_b(ctx):
    return ctx.source.path.parent / "datasets" / "warehouse_b.mcap"
```

`ctx.inputs` is a plain namespace, not a dict — use `getattr` to look it up dynamically by a variant value, as above, rather than `ctx.inputs[...]`.

### Converting trajectory formats with evo

For ROS 2 bags, `evo` extracts and converts trajectories natively — no custom conversion utility is needed. This is the same workflow [`beluga_benchmark.py`](../../examples/beluga/beluga_benchmark.py) uses, with `evo_ape` reading both topics straight from the recorded bag:

```python
ctx.shell.evo_ape.bag2(
    "output",
    "/ground_truth",
    "/pose",
    "--t_max_diff",
    "0.5",
    "--save_results",
    "output.ape.zip",
)
```

`--t_max_diff` is necessary here because the estimated and ground-truth topics are rarely published with the exact same timestamp; without it, `evo_ape` may fail to associate poses at all.

If you'd rather work with a `.tum` trajectory file directly — e.g. to inspect it with `evo_traj` before computing any metric — extract it from the bag first, then feed both `.tum` files to `evo_ape`:

```python
ctx.shell.evo_traj.bag2("output", "/pose", "--save_as_tum")
ctx.shell.evo_ape.tum(
    "ground_truth.tum",
    "pose.tum",
    "--t_max_diff",
    "0.5",
    "--save_results",
    "output.ape.zip",
)
```

> [!NOTE]
> An earlier design considered a custom `Bag2TUM`/`TUM2Bag` conversion utility for this. It was dropped: `evo` already supports both directions natively through its `bag2` reader, so a custom converter would only duplicate it.

### Aggregating results after the run

Already its own section — see [Reading Results](#reading-results) for the `@output` + `lambkin.data.evo` pattern used to plot and log statistics once the full sweep finishes.

### Reprocessing results from disk, without re-running

`lambkin.data.access.iterations()`, `evo.series()`, and `evo.stats()` don't need a live benchmark — they accept a plain path just as well as a context. That means you can revisit a `results/` directory later, from a notebook or a standalone script, and regenerate a different plot or report without re-running anything:

```python
import matplotlib.pyplot as plt
import lambkin

for entry in lambkin.data.evo.stats("results/", "output.ape.zip"):
    print(f"{entry.variant} iter {entry.iteration}: rmse={entry.rmse:.4f}")

plt.figure()
for entry in lambkin.data.evo.series("results/", "output.ape.zip"):
    plt.plot(entry.time, entry.error, label=f"{entry.variant}/iter_{entry.iteration}")
plt.legend()
plt.savefig("results/replot.png")
```

This is the same code you'd put in an `@output` hook, just pointed at a path string instead of `ctx`. Useful for generating a new report from an old run, comparing two separate `results/` directories, or trying out a plot before committing it to the benchmark script itself.

## Expected Output

LAMBKIN writes all artifacts under a consistent directory tree:

```bash
results/
├── variants.yaml
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

`lambkin_metadata.yaml` is always written by the SDK itself. Everything else under `iter_<n>/` is whatever your benchmark function's commands wrote to the current working directory — the exact names and shapes depend entirely on the tools you call (e.g. `ros2 bag record -o output` creates an `output/` *directory* with its own internal files, not a single `output.mcap`). Any artifact written by an `@output` hook (e.g. an aggregated plot) lives one level up, directly under `results/`, since output hooks run at benchmark scope after every iteration has finished.
