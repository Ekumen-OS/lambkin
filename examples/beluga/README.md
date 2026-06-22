# Beluga Example

A worked example of a LAMBKIN benchmark pipeline applied to [Beluga AMCL](https://github.com/Ekumen-OS/beluga) in ROS 2. It sweeps over sensor models and particle counts, evaluates trajectory accuracy with `evo`, and aggregates results across all configurations. Ships with its own ROS 2 package, Docker environment, and benchmark script.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and
  [Docker Compose](https://docs.docker.com/compose/), or
  [Podman](https://podman.io/getting-started/installation) and
  `podman-compose` (see note below) — only needed for [running in a container](#run-in-a-container).
- Reference data is bundled in [`ekumenlabs/lambkin-beluga-datasets`](https://hub.docker.com/r/ekumenlabs/lambkin-beluga-datasets) and copied into the image at build time.

| Artifact | Default location in image | Description |
| --- | --- | --- |
| Rosbag | `/data/datasets/input/` | Reference sensor data to replay during the benchmark. Already includes the `/ground_truth` topic used by `evo_ape`. |
| Map | `/data/maps/map.yaml` | Static map file in `.yaml` and `.pgm` format. |
| Groundtruth | `/data/ground_truth/ground_truth.tum` | Reference trajectory in `.tum` format, provided for convenience if you'd rather use it directly (e.g. `evo_traj tum`) than from the rosbag. Not read by `beluga_benchmark.py` itself. |

> [!NOTE]
> `podman-compose` installed via `apt` may be version 1.0.6, which does not support `--profile`. Install a recent version via `pipx`:
> ```bash
> sudo apt install pipx
> pipx ensurepath
> pipx install podman-compose
> ```
> Verify the installed version with `podman-compose --version` before proceeding.

## How It Works

`nominal()` in [`beluga_benchmark.py`](beluga_benchmark.py) runs four steps per `(variant, iteration)`:

1. **`ros2 bag record`** (background) — records every topic to `output/`.
2. **`ros2 launch`** (background) — brings up the Beluga localization stack: `beluga_amcl` (particle filter, estimates pose from sensor data and a known map), `map_server` (serves the static occupancy grid), and `lifecycle_manager` (handles startup/shutdown ordering for both).
3. **`ros2 bag play`** (foreground) — replays the reference dataset at `--clock-rate`, blocking until playback finishes.
4. **`evo_ape`** (once both background processes exit) — compares the `/ground_truth` and `/pose` topics recorded in the bag, and writes the result to `output.ape.zip`.

`beluga.launch.py` declares `map_path`, `laser_model_type`, and `max_particles` as its launch arguments (see [`beluga_ros2/launch/beluga.launch.py`](beluga_ros2/launch/beluga.launch.py)), with defaults sourced from [`beluga_ros2/params/default.ros2.yaml`](beluga_ros2/params/default.ros2.yaml). `beluga_benchmark.py` sweeps `sensor_model` and `num_particles` as its variant parameters and forwards them as `sensor_model:=` / `num_particles:=` when invoking `ros2 launch`. For the full set of parameters Beluga AMCL accepts, see the [Beluga AMCL reference](https://ekumen-os.github.io/beluga/packages/beluga_amcl/docs/ros2-reference.html).

## How to Run

### Run in a Container

If you are running the benchmark as-is, use the **Production** profile. If you are modifying the benchmark script or the ROS 2 package, use the **Development** profile.

#### 1. (Optional) Use your own data

By default, no volume mounts are needed — the example uses the rosbag, map, and groundtruth baked into the image. To benchmark your own data instead, edit the compose file ([docker-compose.yml](docker/docker-compose.yml) or [podman-compose.yml](docker/podman-compose.yml)) and uncomment the input volume mounts, pointing them at your files:

```yaml
volumes:
  # ── Inputs (uncomment and adjust paths as needed) ─────────
  - /path/to/your/datasets:/data/datasets:ro
  - /path/to/your/maps:/data/maps:ro
  - /path/to/your/ground_truth:/data/ground_truth:ro
  # ── Output (benchmark results persisted on host) ──────────
  - ../results:/ws/examples/beluga/results
```

Each mount overrides the corresponding default directory inside `/data/`, so you only need to uncomment the ones you're replacing.

> [!NOTE]
> `docker/podman-compose.yml`'s own input-mount comments still point at the older `/ws/examples/beluga/rosbags`, `/maps`, `/groundtruth` paths instead of `/data/...`. This only matters if you want to override the bundled data with Podman — the default (no mounts) works the same on both runtimes, since `/data` comes from the image build, not from either compose file. If you do need to override data on Podman, mount your files to `/data/datasets`, `/data/maps`, and `/data/ground_truth` directly rather than copying those stale comments.

#### 2. Build the image

##### Development

```bash
cd examples/beluga/docker
docker compose --profile development build   # Docker
podman-compose --profile development build   # Podman
```

##### Production

```bash
cd examples/beluga/docker
docker compose --profile production build    # Docker
podman-compose --profile production build    # Podman
```

#### 3. Start the container

Two profiles are available depending on your use case.

##### Development

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

##### Production

Builds a fully self-contained image with all dependencies pre-installed. No manual steps needed inside the container.

**Docker**

```bash
docker compose --profile production run --rm lambkin_prod bash
```

**Podman**

```bash
podman-compose --profile production run --podman-run-args="--systemd=always" --rm lambkin_prod bash
```

> [!WARNING]
> Both runtimes require elevated privileges to support background process management. Docker runs with `--privileged`, granting the container broad access to host devices and kernel interfaces. Podman uses `--systemd=always`, which allows the container to interact with the host's cgroup v2 hierarchy.

### Run on the Host

If you'd rather run the example directly on the host — e.g. you already have ROS 2 Jazzy installed locally — you can extract the bundled reference dataset from the dataset image without starting it:

```bash
docker create --name lambkin_data_extract ekumenlabs/lambkin-beluga-datasets:jazzy true
docker cp lambkin_data_extract:/data/. ./your_folder
docker rm lambkin_data_extract
```

`docker create` only registers the container without running it, so this never executes anything inside the image — it just makes its filesystem layers available so `docker cp` can pull `/data/` out, then `docker rm` discards the unused container.

> [!WARNING]
> `your_folder` above is just a placeholder — name it (and place it) however you like. But `beluga_benchmark.py`'s input hooks hardcode the absolute paths `/data/datasets/input` and `/data/maps/map.yaml`, so to run the example unmodified you need to extract to `/data` at the filesystem root instead (`sudo mkdir -p /data && sudo chown "$USER" /data` first if you don't already have write access there). If you'd rather keep the dataset under `your_folder`, edit those two `@nominal.input` hooks in [`beluga_benchmark.py`](beluga_benchmark.py) to point there instead.

Everything the Dockerfile would otherwise set up — ROS 2 Jazzy, `rosdep`-installed dependencies for `beluga_ros2`, `uv sync`, `colcon build` — is then your own responsibility; see the commands under [Development](#development) above for reference.

### Run the Benchmark

Inside the container, or on the host once set up:

```bash
uv run lambkin examples/beluga/beluga_benchmark.py
```

On top of the [SDK-wide options](../../src/lambkin/README.md#cli), this benchmark registers its own:

| Flag | Default | Description |
|---|---|---|
| `--clock-rate` | `1.0` | Playback rate forwarded to `ros2 bag play -r`. |
| `--sensor-topic` | `/scan` | Declared as an option, but not currently read inside `nominal()` — check [`beluga_benchmark.py`](beluga_benchmark.py) before relying on it. |

For everything else — listing options, selecting a subset of variants, dry-running, reading results back with `lambkin.data` — see the [SDK documentation](../../src/lambkin/README.md), which applies the same way to this example as to any other benchmark.
