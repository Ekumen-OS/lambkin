# LAMBKIN
**L**ocalization **A**nd **M**apping **B**ench**KIN**g

> *The user brings the algorithm. LAMBKIN handles the rest.*

LAMBKIN is a Python SDK for building SLAM evaluation pipelines that are reproducible, structured, and parallelized by design. Unlike fixed benchmarking tools, LAMBKIN does not impose a pipeline, does not know about any specific algorithm, and has no dependency on ROS or any other middleware. It operates at the level of processes, files, and Python — nothing more.

You write the benchmark script using the SDK. LAMBKIN provides the machinery that makes it work.

---

## Why LAMBKIN

Most benchmarking systems are built around a specific algorithm, dataset format, or middleware stack. Adapting them to a new setup means working around assumptions that were never designed to be removed. Reproducing a run means knowing which constants changed and when. Adding a new algorithm or dataset variant means touching plumbing that was never meant to be touched.

LAMBKIN separates the **orchestration machinery** from the **benchmark definition**. The algorithm runs as an external process — LAMBKIN does not need to know what is inside it. Parameter sweeps, process lifecycle, I/O, and metric collection are all handled by the SDK, so your script stays focused on the benchmark logic.

---

## What LAMBKIN provides

- **Process orchestration** — launch, supervise, synchronize, and terminate external processes across the benchmark lifecycle.
- **Named products** — declare combinations of algorithms, datasets, and parameters and run them concurrently, with results kept organized and traceable without extra plumbing.
- **Pipeline stages** — structured ingestion (datasets), execution (processes), and egression (metrics), each independently customizable and reusable.
- **Context management** — carry configuration, paths, and state through the pipeline, keeping stages decoupled from each other.

---

## How it works

You write a Python script that imports LAMBKIN and composes its primitives into your benchmark pipeline. You define what data to pull, what processes to launch, how to combine parameters, and what metrics to collect. LAMBKIN orchestrates the execution: it expands your parameter combinations, runs them in parallel, manages process lifecycle, and collects results into a consistent, structured output directory.

The result is a benchmark defined as code — explicit, versionable, shareable, and runnable by anyone with the same environment.

---

## Scope

LAMBKIN is algorithm-agnostic and middleware-agnostic by design. It has no knowledge of any specific SLAM stack and does not require any.

For concrete setups built on top of the SDK, see [`examples/`](examples/). Each example packages a specific system with its own environment and documentation, ready to run out of the box. They are fully optional — the SDK works independently of any of them.

To get started with the SDK, see [LAMBKIN SDK documentation](src/lambkin/README.md) to get started.
