# EKF Odometry Baseline Example

A worked example of a LAMBKIN benchmark comparing **raw wheel odometry** against an
**EKF-fused (odometry + IMU) estimate** produced by
[`robot_localization`](https://github.com/cra-ros-pkg/robot_localization). Unlike the
[Beluga example](../beluga/README.md), there is no map and no particle filter here — this measures
how well dead reckoning alone tracks the robot, which is the floor any localization system should
beat. Ships with its own ROS 2 package and Docker environment.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and
  [Docker Compose](https://docs.docker.com/compose/) — only needed for
  [running in a container](#run-in-a-container).
- Reference data is copied into the image at build time.

| Artifact | Default location in image | Description |
| --- | --- | --- |
| Rosbag | `/data/datasets/input/` | Sensor data to replay. Must carry the two topics the EKF fuses (see [Dataset requirements](#dataset-requirements)). |
| Groundtruth | `/data/ground_truth/groundtruth.tum` | Reference trajectory in `.tum` format, compared against via `evo_rpe`. |

### Dataset requirements

This benchmark is more sensitive to its input than the Beluga one, because the EKF fuses specific
topics rather than a generic scan. The bag at `/data/datasets/input/` must publish:

| Topic | Type | Used for |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | The `raw_odom` baseline, and the odometry input to the EKF. |
| `/imu/data_raw_transformed` | `sensor_msgs/msg/Imu` | The EKF's angular-velocity input. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Sim time source for `ekf_node`. See [Sim time and the recorded `/clock` topic](#sim-time-and-the-recorded-clock-topic). |

The two sensor topic names are configured in
[`ekf_ros2/params/ekf.ros2.yaml`](ekf_ros2/params/ekf.ros2.yaml) as `odom0` and `imu0` —
point them at whatever your own recording uses if the names differ.

This example shares its data with the [Beluga example](../beluga/README.md): the bundled
[`ekumenlabs/lambkin-beluga-datasets`](https://hub.docker.com/r/ekumenlabs/lambkin-beluga-datasets)
image already carries both topics, so no extra dataset is needed and the benchmark runs out of the
box. The same 137 s recording drives both examples — Beluga consumes its laser scans, this one its
wheel odometry and IMU.

To benchmark your own recording instead, point the build at a different data image with the
`DATASET_IMAGE` build argument:

```bash
DATASET_IMAGE=my-datasets:latest docker compose --profile development build
```

Alternatively, leave the image alone and uncomment the input volume mounts in
[`docker-compose.yml`](docker/docker-compose.yml), which shadow `/data/` at run time.

## How It Works

`nominal()` in [`ekf_benchmark.py`](ekf_benchmark.py) sweeps a single variant parameter,
`algorithm`, whose two values take completely different paths through the same evaluation:

- **`raw_odom`** — launches nothing at all. It extracts `/odom` straight out of the source bag with
  `evo_traj`, which is the cheapest possible variant: no ROS system, no playback, no recording.
- **`ekf_fused`** — launches `ekf_node` in the background, records `/odom_filtered`, plays the bag
  through the running filter, then extracts the recorded output.

Both branches converge on the same `evo_rpe` call. This is the interesting structural point of the
example: **a variant parameter does not have to be a tuning knob**. Here it selects between "read a
topic off disk" and "stand up a ROS node and stream a bag through it", and LAMBKIN sequences,
isolates, and scores both identically.

### Sim time and the recorded `/clock` topic

`ekf_node` runs with `use_sim_time: true`, taking "now" from the `/clock` topic instead of the wall
clock — required because the bag's timestamps are years in the past. The bundled dataset already
records its own `/clock`, so `nominal()` does not pass `--clock` to `ros2.bag.play`.

Passing `--clock` on a bag that already has a recorded `/clock` topic starts a second, independent
publisher on the same topic. `ekf_node` cannot tell the two apart, and its sim time advances
erratically as a result — this truncated `/odom_filtered` to ~86 s of the 137 s bag, silently
dropping 205 of 447 ground-truth poses from the comparison.

If your own recording lacks a `/clock` topic, add `--clock` back to the `ros2.bag.play` call.

### Why RPE instead of APE

The [Beluga example](../beluga/README.md) uses `evo_ape` (Absolute Pose Error). This one uses
`evo_rpe` (Relative Pose Error) with `--delta 1 --delta_unit m`, measuring error accumulated per
metre travelled. Open-loop odometry drifts without bound — absolute error would mostly report how
long the recording is, whereas drift *rate* is comparable across datasets of different lengths.

## How to Run

### Run in a Container

Use the **Production** profile to run as-is, or **Development** to modify the benchmark or the ROS 2
package.

#### 1. Build the image

```bash
cd examples/ekf/docker
docker compose --profile production build
```

Swap `production` for `development` to build the development image.

#### 2. Start the container

##### Development

Mounts the repository as a volume so code changes are reflected immediately without rebuilding.

```bash
docker compose --profile development up -d
docker compose --profile development exec lambkin_dev bash
```

Inside the container:

```bash
apt-get update
rosdep install --from-paths /ws/examples/ekf/ekf_ros2 --ignore-src -r -y
uv sync
uv pip install -e /ws --system --break-system-packages
colcon build --base-paths /ws/examples/ekf
source install/setup.bash
```

##### Production

Fully self-contained; no manual steps needed inside the container.

```bash
docker compose --profile production run --rm lambkin_prod bash
```

> [!WARNING]
> Docker runs with `--privileged` to support background process management (cgroup v2 access).

### Run the Benchmark

```bash
uv run lambkin examples/ekf/ekf_benchmark.py
```

On top of the [SDK-wide options](../../src/lambkin/README.md#cli), this benchmark registers:

| Flag | Default | Description |
|---|---|---|
| `--clock-rate` | `1.0` | Playback rate forwarded to `ros2 bag play -r`. Only affects the `ekf_fused` variant; `raw_odom` never plays the bag. |

## Interpreting the Results

Both variants write `output.rpe.zip`, aggregated by the `stats` and `plots` output hooks into
per-variant RMSE and a timeseries plot at `results/plots.png`.

> [!WARNING]
> **Check that both variants cover the same time span before comparing their RMSE.** `evo` matches
> estimate poses to ground truth within `--t_max_diff` and silently discards anything unmatched, so a
> trajectory that stops partway through still yields a plausible-looking RMSE over a shorter segment.
> A `plots.png` where one curve ends earlier than the other is the tell; compare pose counts if in
> doubt:
>
> ```bash
> wc -l results/var_1/iter_1/odom.tum results/var_2/iter_1/odom_filtered.tum
> ```
>
> See [Sim time and the recorded `/clock` topic](#sim-time-and-the-recorded-clock-topic) for a case
> where this happens. Set up correctly, both variants match all 447 ground-truth poses (79
> delta-pairs for `raw_odom`, 77 for `ekf_fused` — small differences here are expected, since
> `--delta 1m` pairing depends on the exact sampled path).

Do not assume the fused estimate wins. This recording's IMU reports a **constant orientation** with an
all-zero `orientation_covariance`, so there is no usable absolute heading to fuse — which is why
[`ekf.ros2.yaml`](ekf_ros2/params/ekf.ros2.yaml) takes only yaw *rate* from the IMU (`imu0_config`)
and leaves heading to odometry's own dead reckoning. With that little extra information available,
the filter may well not beat raw odometry, and that is a legitimate result: "add a filter" is a
hypothesis a benchmark should test rather than assume.

For everything else — listing options, selecting a subset of variants, dry-running, reading results
back with `lambkin.data` — see the [SDK documentation](../../src/lambkin/README.md).
