# LAMBKIN
**L**ocalization **A**nd **M**apping **B**enchmar**KIN**g

> *The user brings the algorithm. LAMBKIN handles the rest.*

LAMBKIN is a Python SDK for building SLAM evaluation pipelines that are reproducible and structured by design.

## Philosophy

Most benchmarking systems are built around a specific algorithm, dataset format, or middleware stack. Adapting them to a new setup means working around assumptions that were never designed to be removed. Reproducing a run means knowing which constants changed and when. Adding a new algorithm or dataset variant means touching plumbing that was never meant to be touched.

LAMBKIN separates the orchestration machinery from the benchmark definition. The algorithm runs as an external process — LAMBKIN does not need to know what is inside it. Parameter sweeps, process lifecycle, I/O, and metric collection are all handled by the SDK, so your script stays focused on the benchmark logic.

## Capabilities

|Feature |Description |
|---|---|
| **Parameter sweeps** | Declare combinations of algorithms, datasets, and parameters. LAMBKIN runs each combination as an independent iteration. |
| **Process lifecycle** | Launch, supervise, and terminate external processes automatically across benchmark iterations. |
| **Pipeline stages** | Structure your benchmark into ingestion, execution, and egression stages, each independently customizable. |
| **Context passing** |Carry configuration, paths, and state through the pipeline without coupling stages to each other.|
| **Result access** |Read structured benchmark outputs (`lambkin.data`) from a notebook or a standalone script, with no live benchmark required.|

To understand how LAMBKIN works under the hood, see the [SDK documentation](src/lambkin/README.md).


## Examples

The [`examples/`](examples/) directory contains ready-to-run setups, each packaging a specific system with its own Docker environment, ROS2 package, and documentation. Each integration is self-contained and optional — the SDK works independently of any of them.

Current examples:

* [examples/beluga/](examples/beluga/README.md) — Beluga AMCL localization, with a worked benchmark script and Docker setup.
