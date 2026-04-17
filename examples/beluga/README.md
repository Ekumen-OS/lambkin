# Beluga Example

A worked example of a LAMBKIN benchmark pipeline applied to [Beluga AMCL](https://github.com/Ekumen-OS/beluga) in ROS 2. It sweeps over sensor models and particle counts, evaluates trajectory accuracy, and aggregates results across all configurations. Ships with its own ROS 2 package, Docker environment, and benchmark script.

## How it works

The benchmark brings up three ROS2 nodes via a launch file:

- **`beluga_amcl`** — particle filter-based AMCL node, estimates robot
  pose from sensor data and a known map.
- **`map_server`** — loads a static occupancy grid and serves it to the
  localization node.
- **`lifecycle_manager`** — manages lifecycle transitions of both nodes,
  handling startup and shutdown ordering automatically.

The launch file accepts the map path, sensor model type, and particle count as parameters, which LAMBKIN sweeps automatically across configurations. For default configuration details, see the [Beluga AMCL reference](https://ekumen-os.github.io/beluga/packages/beluga_amcl/docs/ros2-reference.html).

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and
  [Docker Compose](https://docs.docker.com/compose/)
- The following reference files available on the host:

| Artifact | Description |
|---|---|
| Rosbag | Reference sensor data to replay during the benchmark |
| Map | Static map file in `.yaml` and `.pgm` format |
| Groundtruth | Reference trajectory in `.tum` format to evaluate against |

## Setup

If you are running the benchmark as-is, use the Production profile. If you are modifying the benchmark script or the ROS 2 package, use the Development profile.

### **1. Configure volume mounts**

Edit `docker/docker-compose.yml` and set the host paths to your reference files:

```yaml
volumes:
  - /path/to/your/rosbag:/data/rosbag
  - /path/to/your/map:/data/map
  - /path/to/your/groundtruth:/data/groundtruth
```

### **2. Build the image**


####  Development

```bash
docker compose --profile development build
```

####  Production

```bash
docker compose --profile production build
```

### **3. Start the container**

Two Docker profiles are available depending on your use case.

#### Development

Mounts the repository as a volume so code changes are reflected immediately without rebuilding.

```bash
docker compose --profile development up -d
docker compose --profile development exec lambkin_dev bash
```

Inside the container:

```bash
apt-get update
rosdep install --from-paths /ws/examples/beluga beluga_ros2 --ignore-src -r -y
uv sync
colcon build --symlink-install
source install/setup.bash
```

#### Production

Builds a fully self-contained image with all dependencies pre-installed. No manual steps needed inside the container.

```bash
docker compose --profile production up -d
docker compose --profile production exec lambkin_prod bash
```

## Usage

```bash
uv run examples/beluga/beluga_benchmark.py
```
Once complete, results are written to results/ organized by configuration (var_<number>/iter_<number>). Each iteration contains the recorded bag, TUM trajectory, and APE metrics.

> Note: Full execution is not yet implemented. At this stage the benchmark runs in dry-run mode only, printing all commands that would be executed without running them.
