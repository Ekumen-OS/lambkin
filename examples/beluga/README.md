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
  [Docker Compose](https://docs.docker.com/compose/), or
  [Podman](https://podman.io/getting-started/installation) and
  `podman-compose` (see note below)
- The following reference files available on the host:

| Artifact | Default location in image | Description |
| --- | --- | --- |
| Rosbag | `/data/datasets/input/` | Reference sensor data to replay during the benchmark. Already includes the ground-truth trajectory topic |
| Map | `/data/maps/map.yaml` | Static map file in `.yaml` and `.pgm` format |
| Groundtruth | `/data/ground_truth/ground_truth.tum` | Reference trajectory in `.tum` format, provided for convenience if you'd rather use it directly (e.g. `evo_traj tum`) than use it from the rosbag |

> [!NOTE]
> `podman-compose` installed via `apt` may be version 1.0.6, which does not support `--profile`. Install a recent version via `pipx`:
> ```bash
> sudo apt install pipx
> pipx ensurepath
> pipx install podman-compose
> ```
> Verify the installed version with `podman-compose --version` before proceeding.

## Setup

If you are running the benchmark as-is, use the Production profile. If you are modifying the benchmark script or the ROS 2 package, use the Development profile.

### **(Optional) Use your own data**

By default, no volume mounts are needed — the example uses the rosbag, map, and groundtruth baked into the image. To benchmark your own data instead, edit the compose file ([docker-compose.yml](docker/docker-compose.yml) or [podman-compose.yml](docker/podman-compose.yml)) and uncomment the input volume mounts, pointing them at your files:

Edit `docker/docker-compose.yml` and set the host paths to your reference files:

```yaml
volumes:
  # ── Inputs (uncomment and adjust paths as needed) ─────────
  - path/to/your/datasets:/data/datasets:ro
  - path/to/your//maps:/data/maps:ro
  - path/to/your//ground_truth:/data/ground_truth:ro
  # ── Output (benchmark results persisted on host) ──────────
  - ../results:/ws/examples/beluga/results
```

Each mount overrides the corresponding default directory inside `/data/`, so you only need to uncomment the ones you're replacing.

### **2. Build the image**

#### Development

```bash
cd examples/beluga/docker
docker compose --profile development build   # Docker
podman-compose --profile development build   # Podman
```

#### Production

```bash
cd examples/beluga/docker
docker compose --profile production build    # Docker
podman-compose --profile production build    # Podman
```

### **3. Start the container**

Two profiles are available depending on your use case.

#### Development

Mounts the repository as a volume so code changes are reflected immediately without rebuilding.

**Docker**

```bash
docker compose --profile development up -d
docker compose --profile development exec lambkin_dev bash
```

**Podman**

```bash
podman-compose --profile development up -d
podman-compose --profile development run --podman-run-args="--systemd=always" --rm lambkin_dev bash
```

Inside the container:

```bash
apt-get update
rosdep install --from-paths /ws/examples/beluga/beluga_ros2 --ignore-src -r -y
uv sync
uv pip install -e /ws/src/lambkin --system --break-system-packages
colcon build --base-paths /ws/examples/beluga
source install/setup.bash
```

#### Production

Builds a fully self-contained image with all dependencies pre-installed. No manual steps needed inside the container.

**Docker**

```bash
docker compose --profile production up -d
docker compose --profile production run --rm lambkin_prod bash
```

**Podman**

```bash
podman-compose --profile production run --podman-run-args="--systemd=always" --rm lambkin_prod bash
```

> [!WARNING]
> Both runtimes require elevated privileges to support background process management. Docker runs with --privileged, granting the container broad access to host devices and kernel interfaces. Podman uses --systemd=always, which allows the container to interact with the host's cgroup v2 hierarchy.

## Usage

Inside the Docker container/enviroment, run the following command:

```bash
uv run lambkin examples/beluga/beluga_benchmark.py
```

Once complete, results are written to results/ organized by configuration (`var_<number>/iter_<number>`). Each iteration contains the recorded bag, TUM trajectory, and APE metrics.
