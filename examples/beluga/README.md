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

`beluga.launch.py` declares `map_path`, `laser_model_type`, and `max_particles` as its launch arguments (see [`beluga_ros2/launch/beluga.launch.py`](beluga_ros2/launch/beluga.launch.py)), with defaults sourced from [`beluga_ros2/params/default.ros2.yaml`](beluga_ros2/params/default.ros2.yaml). `beluga_benchmark.py` sweeps `sensor_model` and `num_particles` as its variant parameters and forwards them as `sensor_model:=` / `num_particles:=` when invoking `ros2 launch`. For the full set of parameters Beluga AMCL accepts, see the [Beluga AMCL reference](https://ekumen-os.github.io/beluga/packages/beluga_amcl/docs/ros2-reference.html).

Once the bag finishes playing back, the benchmark runs `evo_ape bag2` comparing the `/ground_truth` and `/pose` topics recorded in the bag, and writes the result to `output.ape.zip`.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and
  [Docker Compose](https://docs.docker.com/compose/), or
  [Podman](https://podman.io/getting-started/installation) and
  `podman-compose` (see note below)
- Reference data — already bundled in the image by default, see below.

| Artifact | Default location in image | Description |
| --- | --- | --- |
<<<<<<< HEAD
| Rosbag | `/data/datasets/input/` | Reference sensor data to replay during the benchmark. Already includes the ground-truth trajectory topic |
| Map | `/data/maps/map.yaml` | Static map file in `.yaml` and `.pgm` format |
| Groundtruth | `/data/ground_truth/ground_truth.tum` | Reference trajectory in `.tum` format, provided for convenience if you'd rather use it directly (e.g. `evo_traj tum`) than use it from the rosbag |
=======
| Rosbag | `/data/datasets/input/` | Reference sensor data to replay during the benchmark. Already includes the `/ground_truth` topic used by `evo_ape`. |
| Map | `/data/maps/map.yaml` | Static map file in `.yaml` and `.pgm` format. |
| Groundtruth | `/data/ground_truth/ground_truth.tum` | Reference trajectory in `.tum` format, provided for convenience if you'd rather use it directly (e.g. `evo_traj tum`) than from the rosbag. Not read by `beluga_benchmark.py` itself. |
>>>>>>> 9148ff5 (Update beluga README)

> [!NOTE]
> `podman-compose` installed via `apt` may be version 1.0.6, which does not support `--profile`. Install a recent version via `pipx`:
> ```bash
> sudo apt install pipx
> pipx ensurepath
> pipx install podman-compose
> ```
> Verify the installed version with `podman-compose --version` before proceeding.

## Setup

If you are running the benchmark as-is, use the **Production** profile. If you are modifying the benchmark script or the ROS 2 package, use the **Development** profile.

### **(Optional) Use your own data**

By default, no volume mounts are needed — the example uses the rosbag, map, and groundtruth baked into the image. To benchmark your own data instead, edit the compose file ([docker-compose.yml](docker/docker-compose.yml) or [podman-compose.yml](docker/podman-compose.yml)) and uncomment the input volume mounts, pointing them at your files:

### 1. (Optional) Use your own data

By default, no volume mounts are needed — the example uses the rosbag, map, and groundtruth baked into the image. To benchmark your own data instead, edit [`docker/docker-compose.yml`](docker/docker-compose.yml) (or [`docker/podman-compose.yml`](docker/podman-compose.yml)) and uncomment the input volume mounts, pointing them at your files:

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

### 2. Build the image

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

### 3. Start the container

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
podman-compose --profile development run --podman-run-args="--systemd=always" --rm lambkin_dev bash
```

> [!NOTE]
> Unlike Docker, this doesn't go through `up -d` + `exec`. `--systemd=always` (required for cgroup v2 delegation) can only be passed at container creation, and `podman-compose.yml` can't declare it directly (`systemd: always` is commented out — `podman-compose` doesn't support it yet). So the container is created and entered in one `run --rm` call instead of two steps.

Inside the container:

```bash
apt-get update
rosdep install --from-paths /ws/examples/beluga/beluga_ros2 --ignore-src -r -y
uv sync
uv pip install -e /ws/src/lambkin --system --break-system-packages
colcon build --base-paths /ws/examples/beluga
source install/setup.bash
```

> [!TIP]
> The repository volume mount (`../../..:/ws`) does not shadow `/data/` — the bundled dataset was copied in at build time, outside `/ws`. Confirm it's there with `ls /data`.

#### Production

Builds a fully self-contained image with all dependencies pre-installed. No manual steps needed inside the container.

**Docker**

```bash
docker compose --profile production run --rm lambkin_prod bash
```

**Podman**

```bash
podman-compose --profile production run --podman-run-args="--systemd=always" --rm lambkin_prod bash
```

> [!TIP]
> Same bundled dataset as Development, copied into the image at the same `/data/` path during build. Confirm it before running the benchmark with `ls -R /data`, or check it from the host without entering an interactive shell:
> ```bash
> docker compose --profile production run --rm lambkin_prod ls -R /data     # Docker
> podman-compose --profile production run --rm lambkin_prod ls -R /data    # Podman
> ```

> [!WARNING]
> Both runtimes require elevated privileges to support background process management. Docker runs with `--privileged`, granting the container broad access to host devices and kernel interfaces. Podman uses `--systemd=always`, which allows the container to interact with the host's cgroup v2 hierarchy.

## Usage

Inside the Docker container or environment, run:

```bash
uv run lambkin examples/beluga/beluga_benchmark.py
```

On top of the [SDK-wide options](../../src/lambkin/README.md#cli), this benchmark registers its own:

| Flag | Default | Description |
|---|---|---|
| `--clock-rate` | `1.0` | Playback rate forwarded to `ros2 bag play -r`. |
| `--sensor-topic` | `/scan` | Declared as an option, but not currently read inside `nominal()` — check [`beluga_benchmark.py`](beluga_benchmark.py) before relying on it. |

List every available option (SDK and custom) without running anything:

```bash
uv run lambkin examples/beluga/beluga_benchmark.py --show-options
```

List all variants before running, or re-run only a subset (e.g. to retry a failed configuration):

```bash
uv run lambkin examples/beluga/beluga_benchmark.py --show-variants
uv run lambkin examples/beluga/beluga_benchmark.py --variants 3:5
```

### Output

Once complete, results are written to `results/`, organized by configuration (`var_<number>/iter_<number>`):

```
results/
├── variants.yaml
├── plots.png                    # written once, after the full sweep, by the `plots` output hook
└── var_<n>/
    └── iter_<n>/
        ├── lambkin_metadata.yaml    # lambkin's own iteration metadata (run hash, timestamps, paths)
        ├── output/                  # ROS 2 bag directory from `ros2 bag record --output output -a`
        │   ├── output_0.mcap
        │   └── metadata.yaml        # ros2 bag's own metadata, not lambkin's
        └── output.ape.zip           # evo_ape result comparing /ground_truth against /pose
```

Two output hooks run once, after every iteration finishes:

- **`plots`** — plots APE error over time for every `(variant, iteration)` on the same figure and saves it to `results/plots.png`.
- **`stats`** — logs RMSE, mean, and max APE per iteration to the console.

Both read their data through [`lambkin.data.evo`](../../src/lambkin/README.md#reading-results), the SDK's result-access layer.
