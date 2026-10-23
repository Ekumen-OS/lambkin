# Beluga odometry comparison

This benchmark compares four paths while keeping the Beluga boundary fixed:

| Estimator | Camera | IMU | External output | Beluga input |
| --- | --- | --- | --- | --- |
| OpenVINS | mono | yes | `/ov_msckf/odomimu` | `/odom` |
| OpenVINS | stereo | yes | `/ov_msckf/odomimu` | `/odom` |
| ORB-SLAM3 | mono | no | `/orbslam3/odom` | `/odom` |
| ORB-SLAM3 | stereo | no | `/orbslam3/odom` | `/odom` |

The common path is:

```text
camera (+ IMU) -> external estimator -> nav_msgs/Odometry
              -> external_vio_adapter -> planar /odom
              -> Beluga's SE(2) odometry interface
```

The adapter is the comparison boundary. It applies the configured body-to-base
TF, transforms pose/twist covariance, retains x/y/yaw and vx/vy/wz, and assigns
large variance to discarded z/roll/pitch axes. It does not fuse the estimator
with Beluga or implement an estimator itself.

## Dataset

The downloaded representative sequence is kept outside the repository:

```text
<dataset-root>/kaist_vio/rotation_fast.hf.bag
```

The official KAIST VIO recording contains a mono RGB topic,
rectified stereo infrared topics, IMU and motion-capture ground truth. For a
fair OpenVINS mono/stereo comparison, both modes use the calibrated left
infrared camera; mono disables the right camera. The
benchmark profile is in
[`external_vio/config/kaist_vio.yaml`](external_vio/config/kaist_vio.yaml).

The downloaded file is a ROS 1 bag. OpenVINS documents that ROS 2 users must
convert KAIST data through ROS 1 before ROS 2 bag playback. The reproducible
conversion used here is:

```bash
rosbags-convert \
  --src <dataset-root>/kaist_vio/rotation_fast.hf.bag \
  --dst <dataset-root>/kaist_vio/rotation_fast_ros2 \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_jazzy
```

The converted bag is passed with `--dataset`. Its image topics are raw
`sensor_msgs/Image`, so the benchmark uses `input_transport:=raw`.
Because the KAIST bag does not contain a robot `imu→base_link` TF, the
benchmark disables the body-to-base transform by default. Enable
`--use-base-frame-transform true` only when the replay also provides that TF.

The selected file is the `rotation_fast` sequence from the [official KAIST
VIO dataset](https://github.com/url-kaist/kaistviodataset), downloaded from
the repository's [Hugging Face mirror](https://huggingface.co/datasets/zinuok/KAIST-VIO-Dataset).
The verified raw file is 5,772,796,533 bytes with SHA-256
`d77f19421a271e8cb0475971ee3475d8954e19b1b081b758966ba81f3e57ebbb`.

KAIST VIO has no 2-D laser map. Therefore this dataset directly evaluates the
external estimator and the Beluga planar SE(2) boundary. To evaluate full
Beluga AMCL on the same replay, a laser/map recording must also be supplied via
`--map-path` and the Beluga parameter file must use the corresponding scan
topic.

## Run the benchmark

Build Lambkin's Beluga example in the development container, then source the
workspace:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
uv pip install -e /ws/src/lambkin --system --break-system-packages
colcon build --base-paths /ws/examples/beluga
source install/setup.bash
```

Inspect the sweep without running it:

```bash
lambkin /ws/examples/beluga/odometry_benchmark.py --show-variants
lambkin /ws/examples/beluga/odometry_benchmark.py --dry-run
```

Run against a converted KAIST ROS 2 bag:

```bash
lambkin /ws/examples/beluga/odometry_benchmark.py \
  --dataset /data/datasets/kaist_vio/rotation_fast_ros2 \
  --input-transport raw \
  --no-cache
```

The ORB-SLAM3 wrapper is external and its launch interface differs between
projects. The default profile assumes a wrapper exposing `mode`, `use_imu`,
and `odom_topic` launch arguments. Override it if needed:

```bash
lambkin /ws/examples/beluga/odometry_benchmark.py \
  --vo-launch-package orbslam3_ros \
  --vo-launch-file realtime.launch.py \
  --vo-launch-args 'mode:={camera} use_imu:=false odom_topic:=/orbslam3/odom' \
  --dataset /data/datasets/kaist_vio/rotation_fast_ros2 \
  --input-transport raw \
  --no-cache
```

The recorded output is under the benchmark `results/` tree. Each completed
iteration also contains `output.ape.zip`, produced by `evo_ape bag2` from
`/pose_transformed` to `/odom`, projected to the xy plane and aligned. Mono
results additionally use scale correction because monocular VO cannot recover
metric scale by itself. Use `ros2 bag info` on each `output/` directory to
verify that `/odom`, `/pose_transformed`, and the estimator topics were
recorded.

If `--map-path` is supplied, the benchmark also launches the existing
`beluga_ros2` AMCL stack and writes `output.beluga.ape.zip`, comparing Beluga's
`/pose` output with the same ground truth. This second metric requires the
replay to contain the laser scan topic configured in
[`beluga_ros2/params/default.ros2.yaml`](beluga_ros2/params/default.ros2.yaml).

## Interpretation

For the KAIST replay, compare the planar `/odom` trajectory against the planar
projection of `/pose_transformed` using the same timestamp association and
SE(2) alignment. Do not compare raw OpenVINS SE(3) directly with Beluga's
SE(2) result. The mono path has unobservable metric scale and is expected to
be less stable; stereo supplies metric scale. OpenVINS' own dataset notes also
warn about monocular behavior on smooth KAIST Urban trajectories.

ORB-SLAM3 is used only as an external VO-only source. No ORB-SLAM3 source or
GPL code is copied into Lambkin. Install the chosen wrapper separately and
publish `nav_msgs/Odometry`; the adapter and benchmark remain unchanged.
