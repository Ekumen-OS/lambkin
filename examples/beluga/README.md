# Beluga Example

This directory is a worked example of a LAMBKIN benchmark pipeline applied to Beluga AMCL. It sweeps over different sensor models and particle counts, automatically evaluates trajectory accuracy, and aggregates results across all configurations. It is self-contained: it ships with its own ROS2 package, Docker environment, and benchmark script.

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

**1. Configure volume mounts**

Before starting the container, edit docker/docker-compose.yml to mount your reference files. Locate the volumes section and set the host paths accordingly:

```yaml
volumes:
  - /path/to/your/rosbag:/data/rosbag
  - /path/to/your/map:/data/map
  - /path/to/your/groundtruth:/data/groundtruth
```
The paths inside the container must match those used in the benchmark script.
**2. Start the container**

Two Docker profiles are available depending on your use case.
#### Development
The development profile mounts the repository as a volume, so code changes are reflected immediately without rebuilding the image. Dependencies are installed manually inside the container.
**1. Start the container**
```bash
docker compose --profile development up -d
docker compose --profile development exec lambkin_developer bash
```
**2. Install ROS2 dependencies**
```bash
rosdep install --from-paths lambkin_ros2 --ignore-src -r -y
```
**3. Install Python dependencies**
```bash
uv sync
```
**4. Build the ROS2 workspace**
```bash
colcon build --symlink-install
source install/setup.bash
```
#### Production
The production profile builds a fully self-contained image. All dependencies are installed and the workspace is compiled at image build time — no manual steps are needed inside the container.
**1. Build and start the container**
```bash
docker compose --profile production up -d
docker compose --profile production exec lambkin_production bash
```
The image is ready to use immediately.



## Usage

### Running the benchmark

```bash
uv run examples/beluga/beluga_benchmark.py
```
