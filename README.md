# LAMBKIN
**L**ocalization **A**nd **M**apping **B**enchmar**KIN**g

> *The user brings the algorithm. LAMBKIN handles the rest.*

LAMBKIN is a Python SDK for building SLAM evaluation pipelines that are reproducible, structured, and parallelized by design.

## Philosophy

Most benchmarking systems are built around a specific algorithm, dataset format, or middleware stack. Adapting them to a new setup means working around assumptions that were never designed to be removed. Reproducing a run means knowing which constants changed and when. Adding a new algorithm or dataset variant means touching plumbing that was never meant to be touched.

LAMBKIN separates the orchestration machinery from the benchmark definition. The algorithm runs as an external process — LAMBKIN does not need to know what is inside it. Parameter sweeps, process lifecycle, I/O, and metric collection are all handled by the SDK, so your script stays focused on the benchmark logic.

## What it provides

| | |
|---|---|
| **Parameter sweeps** | Declare combinations of algorithms, datasets, and parameters and run them concurrently |
| **Process lifecycle** | Launch, supervise, and terminate external processes automatically |
| **Organized results** | Every run is written to a structured, traceable output directory |
| **Reproducibility** | Benchmarks are defined as code — versionable and runnable by anyone with the same environment |


## Examples

The [`examples/`](examples/) directory contains ready-to-run setups, each packaging a specific system with its own environment and documentation.
To understand how LAMBKIN works under the hood, see the [SDK documentation](src/lambkin/README.md).
