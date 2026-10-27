# External odometry projection library

This package is a reusable C++ library for adapting external odometry to a
planar estimator. It contains no ROS node, subscriber, publisher, or launch
file.

The public API provides:

- `ProjectToPlanar`: keeps `x`, `y`, `yaw`, `vx`, `vy`, and `wz`
- `TransformToBase`: applies a body-frame to base-frame transform
- `AdaptToPlanar`: applies the frame transform and planar projection

OpenVINS, visual odometry, or another external estimator remains outside this
package. A ROS node or benchmark harness can call this library at its own
integration boundary.
