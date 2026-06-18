# LAMBKIN
**L**ocalization **A**nd **M**apping **B**enchmar**KIN**g

> *The user brings the algorithm. LAMBKIN handles the rest.*

LAMBKIN is a Python SDK for building SLAM evaluation pipelines that are reproducible and structured by design.

## Philosophy

Most benchmarking systems are built around a specific algorithm, dataset format, or middleware stack. Adapting them to a new setup means working around assumptions that were never designed to be removed. Reproducing a run means knowing which constants changed and when. Adding a new algorithm or dataset variant means touching plumbing that was never meant to be touched.

LAMBKIN separates the orchestration machinery from the benchmark definition. The algorithm runs as an external process — LAMBKIN does not need to know what is inside it. Parameter sweeps, process lifecycle, I/O, and metric collection are all handled by the SDK, so your script stays focused on the benchmark logic.

SLAM algorithms are rarely deterministic — particle filters resample randomly, sensor noise varies run to run, and timing jitter between processes can shift outcomes on its own. A single run's accuracy metric is one sample from a noisy distribution, not a reliable estimate of how an algorithm actually performs. LAMBKIN treats `num_iterations` as a first-class part of every benchmark definition, not an afterthought, so a variant is always evaluated as a population of runs you can summarize — with a mean, a standard deviation, an RMSE — instead of a single number taken on faith.

The same reasoning extends to where results land on disk. A benchmark only stays reproducible and reportable if its output follows a structure you didn't have to invent for that particular run: `results/var_<n>/iter_<n>/`, with metadata recorded alongside whatever artifacts your commands produced. That predictability is what lets `lambkin.data` walk any benchmark's results without being told their shape in advance, and what lets a notebook, a report generator, or a teammate's script reuse the same data months later without reverse-engineering a one-off layout.

## Capabilities

| Feature | Description |
|---|---|
| **Parameter sweeps** | Declare combinations of algorithms, datasets, and parameters. LAMBKIN runs each combination as an independent iteration. |
| **Scoped inputs** | Resolve data dependencies once at the benchmark, variant, or iteration level — not on every single run. |
| **Process lifecycle** | Launch, supervise, and terminate external processes automatically across benchmark iterations, with cgroup-based cleanup. |
| **Partial restarts** | Skip iterations that already completed successfully in a previous run, based on a hash of their inputs. |
| **Dry-run validation** | Validate an entire benchmark's flow — variants, inputs, process calls — without executing a single command, via `--dry-run`. |
| **Context passing** | Carry configuration, paths, and state through the pipeline without coupling stages to each other. |
| **Result access** | Read structured benchmark outputs (`lambkin.data`) from a notebook or a standalone script, with no live benchmark required. |

To understand how LAMBKIN works under the hood, see the [SDK documentation](src/lambkin/README.md).


## Prerequisites

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv)
- Linux with cgroups v2 and systemd (required for background process management)

## Installation

```bash
git clone -b next-gen git@github.com:Ekumen-OS/lambkin.git
uv sync
```

## Usage

A minimal example composing LAMBKIN's primitives into a working benchmark.

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

    ctx.shell.evo_ape.bag2(
        "output",
        "/ground_truth",
        "/pose",
        "--save_results",
        "output.ape.zip",
    )


@my_benchmark.input
def dataset(ctx):
    return ctx.source.path.parent / "datasets" / "my_dataset.mcap"


@my_benchmark.ouput
def plot(ctx):
    data = lambkin.data.evo.series(ctx)


if __name__ == "__main__":
    my_benchmark()
```

Run it with the CLI:

```bash
lambkin my_benchmark.py --clock-rate 50.0
```

Validate the benchmark pipeline without executing any process:

```bash
lambkin my_benchmark.py --dry-run
```

For a complete, working example using the Beluga algorithm, see [`examples/beluga/beluga_benchmark.py`](examples/beluga/beluga_benchmark.py). For the full CLI reference, see the [SDK documentation](src/lambkin/README.md#cli).

## Use Cases

### Benchmarking different datasets

When the input itself depends on the variant — e.g. a different sensor model needs a different recording — scoping the hook to `"variant"` means it's only resolved when the variant changes, not on every iteration. This keeps a multi-dataset sweep just as cheap as a single-dataset one.

```python
@nominal.input(scope="variant")
def dataset(ctx):
    return ctx.source.path.parent / "datasets" / f"{ctx.variant.sensor_model}.mcap"
```

## Cookbook

### Converting trajectory formats with evo

For ROS 2 bags, `evo` extracts and converts trajectories natively, so there's no custom conversion utility to write or maintain.

```python
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

## Project Layout

```
lambkin/
├── src/lambkin/        # The SDK package — see src/lambkin/README.md
├── examples/           # Self-contained worked examples
│   └── beluga/         # Beluga AMCL example
├── test/               # Unit and integration tests
└── pyproject.toml
```

## Examples

The [`examples/`](examples/) directory contains ready-to-run setups, each packaging a specific system with its own Docker environment, ROS 2 package, and documentation. Each integration is self-contained and optional — the SDK works independently of any of them.

Current examples:

* [examples/beluga/](examples/beluga/README.md) — Beluga AMCL localization, with a worked benchmark script and Docker setup.
