# LAMBKIN SDK

The LAMBKIN Python SDK is the core library for building SLAM evaluation pipelines. It provides the orchestration primitives you compose into your benchmark: process lifecycle management, parameter sweep execution, pipeline stages, and structured I/O.


## Table of Contents

- [Architecture](#architecture)
- [Expected Output](#expected-output)
- [Core Concepts](#core-concepts)
- [Process Management with cgroups v2](#process-management-with-cgroups-v2)
- [CLI](#cli)
- [Partial Restarts](#partial-restarts)
- [Logging](#logging)
- [Results](#results)
  - [Metrics](#metrics)
  - [Report Generation](#report-generation)
  - [Reprocessing](#reprocessing)
- [Cookbook](#cookbook)

## Architecture

A benchmark is structured around three stages that LAMBKIN sequences and keeps organized:

1. **Ingestion** — input hooks resolve datasets, maps, or any other dependency your benchmark needs, at the scope where they belong (once for the whole run, once per variant, or once per iteration).
2. **Execution** — your benchmark function runs once per `(variant, iteration)` pair, with a dedicated context, cgroup, and shell to launch and supervise external processes.
3. **Egression** — once every iteration finishes, output hooks run once against the full results tree, free to aggregate, plot, or log metrics using `lambkin.data`.

Each stage is a plain Python function that receives a context object carrying configuration, paths, and state. You implement the logic; LAMBKIN handles the rest.


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

`lambkin_metadata.yaml` is always written by the SDK itself. Everything else under `iter_<n>/` is whatever your benchmark function's commands wrote to the current working directory — the exact names and shapes depend entirely on the tools you call (e.g. `ros2 bag record -o output` creates an `output/` *directory* with its own internal files, not a single `output.mcap`). Artifacts written by `@output` hooks are not placed anywhere automatically — it's up to the hook to decide where to write them (e.g. `ctx.base_dir / "plots.png"` to put them under `results/`).

`results/` being tied to the benchmark script's location is a deliberate design choice: it means the cache always works without requiring the user to pass an explicit output path, and keeps each benchmark script bound to a fixed, reproducible setup.

> [!NOTE]
> Avoid running two benchmarks against the same `results/` directory at once — place them in separate source directories, or rename the existing `results/` folder first, so the cache stays trustworthy and outputs don't get corrupted.

## Core Concepts

LAMBKIN exposes a small set of composable primitives. Together they cover the full lifecycle of a benchmark — from declaring inputs and sweeping parameters, to launching processes and reading results back.

### Named Product

Takes named parameter lists and returns every possible combination as a list of dictionaries, one per benchmark run configuration. Pass the result to `Benchmark` via `variants=` to sweep all combinations automatically.

### Benchmark

Drives the benchmark execution loop, handling iteration, parameter expansion, and context setup. It parses options registered via `Option` once before the loop, then creates a context for every combination of variant and iteration and calls the decorated function with it. Can be used as a decorator via `@benchmark`.

### Context

Carries all namespaced information for a benchmark run, split into three nested scopes that mirror the execution loop:

- `BenchmarkContext` — lives for the entire run. Holds `source`, `options`, `base_dir`, and benchmark-scoped `inputs`.
- `VariantContext` — lives for one variant sweep. Adds `variant`, `variant_index`, `variant_dir`, and inputs merged with the benchmark scope.
- `IterationContext` — lives for one `(variant, iteration)` pair, and is what your benchmark function receives as `ctx`. Adds `iteration`, `paths` (with `base_dir`, `variant_dir`, `iteration_dir`), `shell`, `skipped`, and inputs merged with both parent scopes.

Each level is a context manager: entering it creates the corresponding output directory on disk, and exiting it tears down anything it owns — the iteration's cgroup, in the innermost case. `ctx.inputs` is read-only once resolved; assigning to it a second time raises `AttributeError`.

### Source

Describes the benchmark script being executed. Exposed through the context as `ctx.source`, it gives benchmark stages access to the script's own location (`ctx.source.path`) without hardcoding paths.

### Input

Registers a data resolution hook on a benchmark function. Each hook must be a callable that accepts a single `Context` object as its argument and must return a non-empty value. Hook names must be unique across the entire benchmark — registering two hooks with the same name, even under different scopes, raises an error. Can be used as a decorator via `@input`.

Hooks can be scoped to one of three lifecycle levels via `scope=`:

- `"benchmark"` (default) — resolved once before the variant loop. Use for inputs that don't depend on the current variant or iteration, e.g. a single dataset for the entire benchmark.
- `"variant"` — resolved once per variant. Use for inputs that depend on `ctx.variant` but not `ctx.iteration`, e.g. selecting a dataset registered as a variant parameter.
- `"iteration"` — resolved once per iteration, only on a cache miss. Use for inputs that depend on both `ctx.variant` and `ctx.iteration`, e.g. a per-iteration random seed.

```python
@nominal.input
def dataset(ctx): ...  # benchmark scope (default)


@nominal.input(scope="variant")
def calibration(ctx): ...  # variant scope
```

Resolved inputs are merged down the hierarchy, so a variant-scoped hook can rely on benchmark-scoped inputs already being available on `ctx.inputs`, and so on for iteration scope.

### Output

Registers a function as a callback that runs once, after the entire benchmark loop completes, receiving the benchmark-scoped context. Hook names must be unique. Can be used as a decorator via `@output`.

> [!NOTE]
> Output hooks receive a `BenchmarkContext`, not an `IterationContext` — every iteration's cgroup and shell have already been torn down by the time hooks run. Don't call `ctx.shell` or launch any process inside an output hook; read artifacts from disk via `ctx.base_dir` instead (see [Results](#Results)).

### Option

Registers a CLI option on a benchmark. Built on top of [`click`](https://click.palletsprojects.com/en/stable/options/), so any attribute supported by `click.Option` can be passed. Flag names must start with `-` or `--`. Declared options are collected and parsed once before the execution loop, and their values made available through the context. Can be used as a decorator via `@option`.

### Shell

Abstracts shell command dispatch. Exposes the host environment's executables as Python attributes — accessing `shell.my_tool` returns a callable that runs `my_tool` with the given arguments, letting benchmark scripts invoke external processes without hardcoding paths or constructing subprocess calls manually. Accessible through the context.

### Background Process

Runs a process in the background while the benchmark continues executing. Takes a shell command (without calling it) and manages its full lifecycle — start, monitor, and clean up — as a context manager. When the context exits, it terminates the process and all its descendants; any process spawned outside a `background()` block is not covered.

If a background process dies unexpectedly before the context exits, any foreground call currently blocked waiting on it is unblocked immediately instead of hanging, and the iteration fails with `LambkinProcessDiedUnexpectedlyError`.

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
<delegated cgroup root>/  ← whatever /proc/self/cgroup reports inside the container
└── lambkin-my_benchmark-a1b2c3d4/
    └── iter-var_1-iter_1-e5f6a7b8/
        ├── my_algorithm-a9b0c1d2/
        └── my_recorder-e3f4a5b6/
```

> [!WARNING]
> The cgroup design provides process lifetime containment, not network isolation. If iterations were to run in parallel, processes from different iterations could still communicate with each other.

### Process Cleanup

When an iteration completes, LAMBKIN tears down the iteration cgroup by sending `SIGTERM` to all remaining processes, waiting for a grace period, then sending `SIGKILL` to any survivors. On Ctrl-C, the CLI writes 1 to `cgroup.kill`, which the kernel propagates instantly to the entire iteration cgroup.

Your benchmark function's process never receives `SIGINT` directly — only the CLI's own session does.

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
`--show-options` and `--show-variants` behave like `--help`: they print information and exit immediately, never reaching the benchmark body.


> [!NOTE]
> The SDK option names above are reserved. Declaring any of them via `@lambkin.option` in your script raises a Click error at parse time due to duplicate parameter names.

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

LAMBKIN supports partial restarts by writing a metadata file to each iteration's own output folder. It records whether the iteration completed, along with a hash encoding the variant's parameters, the iteration index, and any custom options you've declared — SDK flags like `--log-level` are excluded, since they don't affect what the benchmark actually produces.

> [!WARNING]
> Currently the hash doesn't account for changes to the benchmark function's own source code, or to the contents of a file an input hook resolves (e.g. swapping in a different dataset without changing any variant or option). Edit either of those and the cache will still report a hit.

On the next run, before touching anything, LAMBKIN checks each iteration's metadata against the current hash: a match with a recorded completion means it's safe to skip; a mismatch — because a parameter or option changed — or no completion at all means it reruns. That's what lets an interrupted hundred-run sweep resume exactly where it left off, and lets changing one parameter invalidate only the iterations it actually affects.

Pass `--no-cache` to bypass this check entirely and force a full rerun of every iteration, regardless of prior completion.

> [!NOTE]
> Dry runs write the initial metadata but never get `completed_at`, so they always show as cache misses — that's expected, not a bug.

## Logging

LAMBKIN has two independent logging systems: one for its own internal messages and one for subprocess output.

### SDK Logging

Controls the verbosity of LAMBKIN's own internal messages via the `--log-level` SDK option. Accepts any level supported by [Python's logging](https://docs.python.org/3/library/logging.html#logging-levels) module, case-insensitive:

```bash
lambkin my_benchmark.py --log-level debug
lambkin my_benchmark.py --log-level DEBUG  # equivalent
```

The LAMBKIN logger is fully isolated from the root logger — user scripts can configure their own logging without any interference. `configure_logging` attaches a dedicated `StreamHandler` to the `"lambkin"` logger and sets `propagate = False`, so its messages never reach the root logger or any handler your script may have configured there. Most scripts don't need to touch this.

**Adding a handler directly to the LAMBKIN logger**

The simplest way to route LAMBKIN's output somewhere extra — a file, a custom formatter, a remote sink — is to attach a handler directly to `"lambkin"`. A logger's own handlers always fire regardless of `propagate`, so this works without touching root at all:

```python
import logging


def nominal(ctx):
    logging.getLogger("lambkin").addHandler(logging.FileHandler("lambkin.log"))
    ...
```

**Folding LAMBKIN output into an existing root-level setup**

If you're already configuring the root logger (e.g. with `logging.basicConfig`) and want LAMBKIN's messages to flow through it as well, re-enable propagation inside your benchmark function. `configure_logging` resets `propagate` to `False` unconditionally before your function is ever called, so setting it any earlier gets overwritten and doesn't stick. Note that `propagate = True` on its own is a no-op if the root logger has no handlers configured:

```python
import logging


def nominal(ctx):
    logging.basicConfig(level=logging.DEBUG)  # or configure root elsewhere
    logging.getLogger("lambkin").propagate = True
    ...
```

See Python's [Logging Cookbook](https://docs.python.org/3/howto/logging-cookbook.html) for handler, formatter, and routing patterns beyond this.

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

## Results

`lambkin.data` provides structured access to whatever a benchmark already wrote to disk:

- **`lambkin.data.access.iterations(source)`** — walks `results/var_*/iter_*/`, skips any iteration that didn't complete, and returns one entry per completed iteration with `iter_dir`, `variant` (e.g. `"var_1"`), `iteration`, and `params` (the variant's parameters as a `SimpleNamespace`). Accepts either a context-like object exposing `.base_dir`, or a plain path/string.
- **`lambkin.data.evo.series(source, filename)`** — same traversal, plus loads the `evo` result file (e.g. `"output.ape.zip"`) from each iteration directory and exposes `time`, `error`, and `distance` arrays, ready to plot.
- **`lambkin.data.evo.stats(source, filename)`** — same traversal, but exposes the aggregate statistics `evo` computes for each result: `rmse`, `mean`, `median`, `std`, `min`, `max`, `sse`.

### Metrics

LAMBKIN doesn't compute trajectory metrics itself — it invokes `evo` through `ctx.shell`, the same way it invokes any other external process, and reads back whatever `evo` writes to disk. The field names exposed by `lambkin.data.evo` (`rmse`, `mean`, `median`, `std`, `min`, `max`, `sse`) are `evo`'s own, not LAMBKIN's.

`evo` provides two metrics for trajectory evaluation (see the [evo Metrics documentation](https://github.com/MichaelGrupp/evo/wiki/Metrics) for full details):

- **APE** (`evo_ape`) — Absolute Pose Error. Directly compares corresponding poses between the estimate and the reference. Measures global consistency, i.e. how close the full trajectory is to ground truth.
- **RPE** (`evo_rpe`) — Relative Pose Error. Compares pose deltas (motions) instead of absolute poses. Measures local accuracy and drift, e.g. translational or rotational error per meter traveled.


### Output Hooks

Use `lambkin.data` inside an output hook to turn results into plots or statistics:

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
### Report Generation

`lambkin.data` provides the building blocks to generate reports from benchmark
results. The Beluga example includes a ready-to-use Jupyter cookbook at
`examples/beluga/report.ipynb` that demonstrates the full workflow using
`lambkin.data.access` and `lambkin.data.evo`.

The cookbook covers:

1. Listing available variants and iterations with `access.iterations()`.
2. APE timeseries by variant — individual iterations + per-variant mean.
3. Stats summary table — RMSE, mean, and max aggregated across iterations.
4. RMSE comparison bar chart across variants.
5. Converting results to a pandas DataFrame for advanced analysis with seaborn.
6. Exporting the notebook to HTML with `jupyter nbconvert`.

Open it after a benchmark run:

```bash
jupyter notebook examples/beluga/report.ipynb
```

### Reprocessing

None of `lambkin.data`'s functions need a live benchmark — `access.iterations()`, `evo.series()`, and `evo.stats()` accept a plain path just as well as a context. That means a `results/` directory can be revisited later, from a notebook or a standalone script, and reprocessed into a different plot or report without re-running anything:

```python
def reprocess():
    for it in lambkin.data.access.iterations("/path/to/results"):
        sh = lambkin.ShellProxy(
            dry_run=False,
            cwd=it.iter_dir,
            cgroup=None,
            log_output="console",
        )
        sh.evo_ape.bag2(
            "output",
            "/ground_truth",
            "/pose",
            "--save_results",
            "output2.ape.zip",
        )
```

This is the same code as in an `@output` hook, just pointed at a path string instead of `ctx`. Useful for generating a new metrics from an old run, comparing two separate `results/` directories, or trying out a plot before committing it to the benchmark script itself.

If your reprocessing script also needs to invoke external processes (e.g. re-running `evo_ape` with different parameters on already-recorded bags), be aware of two limitations that apply outside the benchmark loop:


* **Log files are overwritten**. `ShellProxy` tracks call counts per command name to give each invocation a unique log file suffix, but that counter resets on every new `ShellProxy` instance. If you call the same command twice in a reprocessing script, the second run's log silently overwrites the first. Set `--log-output console` (or `log_output="console"` at the call site) to avoid this.

* **No cgroup containment**. The iteration cgroup is managed exclusively by `IterationContext`. Outside of it, `ShellProxy` runs with `cgroup=None` — processes are launched directly without any cgroup, so there's no kernel-enforced cleanup if the script is interrupted.

## Cookbook

### Converting trajectory formats with evo

`evo` supports multiple trajectory file formats natively — `bag2`, `tum`, `kitti`, `euroc` — and can export between them via `evo_traj --save_as_<format>`. See the [evo Formats documentation](https://github.com/MichaelGrupp/evo/wiki/Formats#saving--exporting-to-other-formats) for the full conversion matrix.

This belongs inside `nominal()`, not in an output hook — it uses `ctx.shell`, which is only available during iteration execution (see the warning under [Output](#output) in Core Concepts).

A typical pattern: extract a topic from the recorded bag as TUM, then score it with `evo_ape`:

```python
def nominal(ctx):
    # ... ros2 bag record / ros2 launch / ros2 bag play ...

    ctx.shell.evo_traj.bag2(
        "output",
        "/amcl_pose",
        "--save_as_tum",
        "amcl_pose.tum",
    )
    ctx.shell.evo_ape.tum(
        ctx.inputs.ground_truth,
        "amcl_pose.tum",
        "--save_results",
        "output.ape.zip",
    )
```

> [!NOTE]
> `ShellProxy` converts keyword argument underscores to dashes (`save_as_tum=` → `--save-as-tum`), which `evo` won't recognize. Always pass `evo` flags that contain underscores as positional strings, as shown above.
