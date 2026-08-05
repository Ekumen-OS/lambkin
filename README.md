<picture>
  <source media="(prefers-color-scheme: dark)" srcset="doc/logo_white.png">
  <source media="(prefers-color-scheme: light)" srcset="doc/logo_black.png">
  <img alt="Shows the LAMBKIN logo." srcset="docs/logo_black.png">
</picture>

---

**L**ocalization **A**nd **M**apping **B**enchmar**KIN**g

> *The user brings the algorithm. LAMBKIN handles the rest.*

LAMBKIN is a Python SDK for building SLAM evaluation pipelines that are reproducible and structured by design.

> **Notice**: This is the second iteration of this software. The first version of LAMBKIN has now been deprecated and is no longer maintained, but the code can still be found in the `lambkin-classic` branch in this repository.

## Philosophy

Most benchmarking systems are built around a specific algorithm, dataset format, or middleware stack. Adapting them to a new setup means working around assumptions that were never designed to be removed. Reproducing a run means knowing which constants changed and when. Adding a new algorithm or dataset variant means touching plumbing that was never meant to be touched.

LAMBKIN separates the orchestration machinery from the benchmark definition. The algorithm runs as an external process — LAMBKIN does not need to know what is inside it. Parameter sweeps, process lifecycle, I/O, and metric collection are all handled by the SDK, so your script stays focused on the benchmark logic.

A benchmark's output deserves the same discipline as its execution: a structured, predictable layout, with metadata recorded alongside whatever artifacts your commands produce. That predictability is what lets a notebook, a report generator, or a teammate's script revisit any benchmark's output months later without reverse-engineering its shape.

## Capabilities

| Feature | Description |
|---|---|
| **Parameter sweeps** | Declare combinations of algorithms, datasets, and parameters. LAMBKIN runs each combination as an independent iteration. |
| **Iterations** | Run each variant across multiple iterations, giving results statistical significance instead of relying on a single noisy sample. |
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
- Linux with [cgroups v2](https://docs.kernel.org/admin-guide/cgroup-v2.html)

## Installation

```bash
git clone git@github.com:Ekumen-OS/lambkin.git
cd lambkin
uv tool install .
```

This installs `lambkin` onto your `PATH`. You can delete the cloned directory afterward — to pick up updates, re-clone and re-run the command. Use `--editable` instead if you want changes in the clone to apply immediately, but keep the directory in place for as long as you need updates.

Alternatively, install into a `uv`-managed virtual environment:

```bash
uv sync
```

Then run benchmarks via `uv run lambkin my_benchmark.py`.

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


@my_benchmark.output
def plot(ctx):
     for entry in lambkin.data.evo.series(ctx, "output.ape.zip"):
        plt.plot(
            entry.time, entry.error, label=f"{entry.variant} / iter {entry.iteration}"
        )
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()
    plt.savefig(ctx.base_dir / "plots.png")



if __name__ == "__main__":
    my_benchmark()
```

Run it with the CLI:

```bash
lambkin my_benchmark.py
```
Run `lambkin --help` to see all available options.

For a complete, working example using the Beluga algorithm, see [`examples/beluga/beluga_benchmark.py`](examples/beluga/beluga_benchmark.py). For the full CLI reference, see the [SDK documentation](src/lambkin/README.md#cli).

## Use Cases

### Parameter sweeps

The most basic use case: sweep one or more parameters across a range of values, running every combination under identical conditions. This is the basis for programmatic tuning — searching for the setting that optimizes a metric — and for regression testing, where re-running a fixed sweep over time surfaces any change that degrades performance.

```python
@lambkin.benchmark(
    variants=lambkin.common.named_product(num_particles=[100, 500, 1000, 5000]),
    num_iterations=30,
)
def nominal(ctx):
    ctx.shell.my_algorithm(f"num_particles:={ctx.variant.num_particles}")
    ...
```

### Benchmarking different datasets

When the input itself depends on the variant — e.g. a different sensor model needs a different recording — scoping the hook to `"variant"` means it's only resolved when the variant changes, not on every iteration. This keeps a multi-dataset sweep just as cheap as a single-dataset one.

```python
@lambkin.benchmark(
    variants=lambkin.common.named_product(dataset=["warehouse", "office"]),
    num_iterations=2,
)
def nominal(ctx):
    ...

@nominal.input(scope="variant")
def dataset(ctx):
    return ctx.source.path.parent / "datasets" / f"{ctx.variant.dataset}.mcap"
```

### Benchmarking different algorithms

`named_product` doesn't care whether a parameter is a tuning knob or a completely different code path. Adding an `algorithm` dimension to the sweep is enough to compare two localizers under identical conditions — same dataset, same number of iterations, same evaluation pipeline — with no extra orchestration:

```python
@lambkin.benchmark(
    variants=lambkin.common.named_product(
        algorithm=["beluga", "nav2_amcl"],
        sensor_model=["likelihood", "beam"],
        num_particles=[1, 10, 100, 1000, 10000],
    ),
    num_iterations=30,
)
def nominal(ctx):
    match ctx.variant.algorithm:
        case "nav2_amcl":
            nominal_nav2_amcl(ctx)
        case "beluga":
            nominal_beluga(ctx)
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
