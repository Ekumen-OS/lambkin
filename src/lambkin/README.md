# LAMBKIN SDK

This is the LAMBKIN Python SDK — the core library for building SLAM evaluation pipelines. It provides the orchestration primitives you compose into your benchmark: process lifecycle management, parameter sweep execution, pipeline stages, and structured I/O.

## Architecture

### Output folder structure

LAMBKIN automatically creates the following directory structure for all generated artifacts:

```
results/
└── <variant_n>/
    └── iter_<n>/
        ├── output.mcap
        ├── out.zip
        └── ...
```

## Requirements

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv)

## Installation

Clone the repository and install the SDK dependencies:

```bash
git clone git@github.com:Ekumen-OS/lambkin.git
uv sync
```

## Getting started

You write a Python script that imports the SDK and composes its primitives into your benchmark pipeline. LAMBKIN handles the orchestration; you define the logic.

A benchmark is structured around three stages:

1. **Ingestion** — locate or download datasets and prepare inputs.
2. **Execution** — launch and supervise the processes that run your algorithm.
3. **Egression** — collect outputs and compute metrics.

Each stage is a Python callable that receives a context object carrying configuration, paths, and state. You implement the logic; LAMBKIN sequences the stages, manages parallelism, and keeps results organized.

See [`examples/beluga/beluga_benchmark.py`](examples/beluga/beluga_benchmark.py) for a working end-to-end example of a benchmark built on top of the SDK using the Beluga algorithm.

## Running the example

For a complete walkthrough of the Beluga example, including Docker setup, dependencies, and how to run it, see the [Beluga example README](examples/beluga/README.md).
