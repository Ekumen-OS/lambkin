# LAMBKIN: the Localization And Mapping BenchmarKINg Toolkit

## 🌐 Overview

LAMBKIN is a mixture of automation and conventions to facilitate reproducible benchmarking and evaluation of localization and mapping systems, featuring:

-   Reproducible environments
-   Declarative benchmark definitions
-   Standard performance metrics
-   Automatic report generation

## 🧩 Components

LAMBKIN aggregates a number of open-source libraries and tools, both local to the project and third-party, that can be mixed and matched as necessary:

| Component                                                        | Description                                                                                                          |
|------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|
| [`core/lambkin-shepherd`](src/core/lambkin-shepherd)             | A Python package for RobotFramework-based benchmark definitions and orchestration.                                   |
| [`core/lambkin-clerk`](src/core/lambkin-clerk)                   | A Python package for system introspection, useful to contextualize benchmark reports.                                |
| [`external/ros`](src/external/ros)                               | ROS (1) distribution for the target platform (typically Ubuntu).                                                     |
| [`external/ros2`](src/external/ros2)                             | ROS 2 distribution for the target platform (typically Ubuntu).                                                       |
| [`external/typesetting/latex`](src/external/typesetting/latex)   | TeX Live distribution for the target platform.                                                                       |
| [`external/profiling/timemory`](src/external/profiling/timemory) | Source distribution of [timemory](https://github.com/NERSC/timemory), the performance analysis and logging toolkit.  |

These components are distributed and deployed using containers. Each container image bundles a set of components with an operating system, which ensures reproducibility (barring hardware and kernel dependencies).

## 📈 Benchmarks

The simplest approach to LAMBKIN is to draw inspiration from existing [sample benchmarks](src/benchmarks). These are written as containerized applications, built from a base package (typically a ROS package), using RobotFramework-driven definitions and Sphinx-powered reports.

## Next steps

- Run a [sample benchmark](src/benchmarks) or write your own!
- Read the [contributing guidelines](CONTRIBUTING.md).
