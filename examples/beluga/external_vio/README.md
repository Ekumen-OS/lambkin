# External VIO integration

This example provides an external VIO integration for the Beluga benchmark. It
does not reimplement a visual-inertial estimator and it does not add OpenVINS
to Beluga. OpenVINS remains an external ROS 2 node; Beluga consumes the
standard odometry published by the adapter package.

## Data flow

```text
camera + IMU
    -> OpenVINS (`ov_msckf`)
    -> /ov_msckf/odomimu (`nav_msgs/Odometry`)
    -> TF body frame to robot base frame
    -> planar_odometry_adapter
    -> /beluga/vio/odometry (`nav_msgs/Odometry`)
    -> Beluga OdomProbe / AMCL
```

OpenVINS publishes its odometry on `odomimu` and uses `global` and `imu` as
the default frame IDs in its ROS 2 visualizer. Before projection, the adapter
looks up the configured transform from the OpenVINS child frame (usually
`imu`) to the robot base frame (usually `base_link`). It transforms the pose,
twist, and twist covariance, changes the output child frame to the base frame,
and then keeps x/y/yaw and vx/vy/wz. Discarded z, roll, and pitch axes receive
a large variance.

## Package

The package is `external_vio_adapter`. Its first implementation contains:

- `planar_odometry_adapter`: ROS 2 node that converts external 6-DoF odometry
  into a planar `nav_msgs/Odometry`
- `planar_odometry_adapter.launch.py`: runs only the adapter
- `openvins_beluga.launch.py`: starts `ov_msckf` and the adapter together
- `config/planar_odometry_adapter.yaml`: default topic and covariance settings

## Run the adapter with an existing OpenVINS instance

```bash
ros2 launch external_vio_adapter planar_odometry_adapter.launch.py
```

The default input and output topics are:

```text
/ov_msckf/odomimu -> /beluga/vio/odometry
```

They can be overridden without changing the package:

```bash
ros2 launch external_vio_adapter planar_odometry_adapter.launch.py \
  input_topic:=/ov_msckf/odomimu \
  output_topic:=/beluga/vio/odometry
```

## Run OpenVINS and the adapter together

OpenVINS must be installed in the same ROS 2 environment and must have a
calibrated estimator configuration for the selected camera and IMU.

```bash
ros2 launch external_vio_adapter openvins_beluga.launch.py \
  openvins_config_path:=/path/to/estimator_config.yaml \
  openvins_odom_topic:=/ov_msckf/odomimu \
  beluga_odom_topic:=/beluga/vio/odometry
```

The default `euroc_mav` configuration is only a launch sanity check. It is not
a substitute for calibration of the target camera and IMU.

For the KAIST VIO profile, `openvins_beluga.launch.py` also starts three
`image_transport` republishers. They normalize either raw or compressed KAIST
topics into the raw `sensor_msgs/Image` topics expected by OpenVINS, then override both
OpenVINS' `topic_*` parameters and its default/configured topic names. Use
`use_stereo:=false` and `max_cameras:=1` for mono, or `use_stereo:=true` and
`max_cameras:=2` for stereo. The mono comparison uses the calibrated left
infrared camera; the RGB stream is republished for future experiments but is
not used by the KAIST OpenVINS calibration.

## Beluga connection

Configure the Beluga odometry probe to subscribe to the adapter output:

```yaml
odometry:
  type: beluga::Odometry
  device_name: vio
  topic_name: /beluga/vio/odometry
  world_frame: map
  base_frame: ""
  use_pose: true
  use_linear_velocity: true
  use_angular_velocity: true
```

The adapter performs the body-to-base transform, so set the Beluga
`base_frame` to an empty string to avoid applying the same transform twice.
The `world_frame` setting still requires a valid TF from the OpenVINS world
frame (for example `global`) to Beluga's world frame (for example `map`).

## External dependency boundary

This package contains only the ROS 2 adapter and launch/configuration glue.
OpenVINS remains an independently installed external dependency. No OpenVINS
source is copied into this repository, which keeps the Beluga core independent
from the external estimator and its license.

The upstream OpenVINS documentation describes the ROS 2 installation and
`subscribe.launch.py` interface:

- <https://docs.openvins.com/gs-installing.html>
- <https://docs.openvins.com/classov__msckf_1_1ROS2Visualizer.html>

## Current limitations

- The adapter assumes the incoming pose covariance uses the standard ROS
  `[x, y, z, roll, pitch, yaw]` ordering
- Pose covariance is re-expressed with a local numerical Jacobian; its
  behavior near RPY singularities should still be evaluated with real data
- It does not change OpenVINS camera or IMU configuration
- Accuracy and computational performance still need evaluation with recorded
  data or simulation
