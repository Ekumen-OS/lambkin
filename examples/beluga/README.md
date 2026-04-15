# LAMBKIN

## Overview

LAMBKIN (Localization And Mapping Benchmarking) is a programmatic SDK for building reproducible, structured, and parallelized SLAM evaluation pipelines.
It moves away from complex automation "glue" in favor of a clean, Python-first approach to benchmarking.


## Scope

This example demonstrates how to use LAMBKIN with the Beluga AMCL localization algorithm, sweeping over different sensor models and particle counts, automatically evaluating trajectory accuracy, and aggregating results across all configurations.

## ROS2 Beluga Example

While Lambkin is algorithm-agnostic by design and can integrate with any localization package, this repository provides a specific worked example using [Beluga](https://github.com/Ekumen-OS/beluga) AMCL. It includes a ROS2 package with a launch file and a configuration file ready to run a complete benchmark out of the box.

To simplify deployment, the repository provides a dockerized setup (see [docker](docker/) folder) that includes all necessary dependencies and tools pre-configured for immediate use.

The provided launch file brings up three ROS2 nodes:

- **`beluga_amcl`** — Particle filter-based AMCL node, responsible for estimating the robot pose from sensor data and a known map.
- **`map_server`** — Loads a static occupancy grid from disk and provides the static map to the localization node.
- **`lifecycle_manager`** — Manages the lifecycle transitions (`configure` → `activate`→ `deactivate` → `cleanup`) of both the localization node and `map_server`, handling their startup and shutdown ordering automatically.

The launch file accepts parameters such as the map path, sensor model type, and maximum number of particles, allowing Lambkin to sweep different configurations automatically.

* For more details on the default configuration, see the [configuration file](https://ekumen-os.github.io/beluga/packages/beluga_amcl/docs/ros2-reference.html).

## Installation

### Prerequisites

Make sure you have the following installed:

- [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/)

### Expected artifacts

Make sure you have the following reference files available before running the benchmark:

| Artifact | Description |
|---|---|
| Rosbag | Reference sensor data to replay during the benchmark |
| Map | Static map file in `.yaml` and `.pgm` format |
| Groundtruth | Reference trajectory in `.tum` format to evaluate against |

### Setup

**1. Build and start the Docker container:**

```bash
docker compose up -d lambkin_dev
docker compose exec -it lambkin_dev bash
```
Mount your reference files as volumes in `docker-compose.yml` before starting the container:

```yaml
volumes:
  - /path/to/your/rosbag:/data/rosbag
  - /path/to/your/map:/data/map
  - /path/to/your/groundtruth:/data/groundtruth
```

> **Note:** Users are responsible for mounting their own reference files. The paths inside the container should match the ones used in the benchmark script.

**2. Install ROS2 dependencies:**

```bash
rosdep install --from-paths lambkin_ros2 --ignore-src -r -y
```

**3. Install all dependencies (including lambkin):**

```bash
uv sync
```

This installs lambkin and all its dependencies into the environment. After this step, you can use `import lambkin` in any benchmark script without any additional configuration.

**4. Build the workspace:**

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Running the benchmark

```bash
uv run examples/beluga/beluga_benchmark.py
```

By default, the benchmark runs in dry-run mode, printing the commands that would be executed without running them:

```bash
uv run examples/beluga/beluga_benchmark.py --dry-run
```

> **Note:** Real execution is not yet supported. The `--dry-run` flag is the only supported mode at this time.
