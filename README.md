# LAMBKIN
## Overview

Lambkin is a mixture of automation tools and conventions for SLAM benchmarking, featuring:

-   Reproducible environments.
-   Declarative benchmark definitions.
-   Standard performance metrics.
-   Automatic report generation.

To do this, the main library leverages existing technologies such as:

- [Robot Framework](https://robotframework.org/) and [roslaunch](http://wiki.ros.org/roslaunch) for process orchestration.
- [timem](https://timemory.readthedocs.io/en/develop/features.html#command-line-tools) for computational performance instrumentation.
- [evo](https://michaelgrupp.github.io/evo/) for localization performance instrumentation.
- [pandas](https://pandas.pydata.org/) and [seaborn](https://seaborn.pydata.org/) for metrics visualization.
- [rosbag](http://wiki.ros.org/rosbag) for data storage.
- [docker](https://docs.docker.com/) for containerization.
- [reStructuredText](https://docutils.sourceforge.io/rst.html) for report generation.

In the following sections, you'll find a description of the rationale behind the use of the main technologies.

### Docker

In order to generate reproducible environments, we've based our design around [docker](https://docs.docker.com/) containers. This gives us the ability to create isolated environments that satisfy all benchmark prerequisites. These come from two sources:
- From the SLAM system: dependencies in the form of libraries installed from a package manager or source built using a specific build system.
- From the benchmark itself: automation and performance monitoring tools.

For this purpose, we've developed a base docker image with common automation tools and utilities that can be used across all the different SLAM systems.
A benchmark package extends this base image with the required installation steps for the SLAM system under test.

### ROS

When developing this tool, we decided to target ROS SLAM packages due to its widespread adoption among the robot development community.

All the benchmarking scripts are distributed in their own ROS package, containing [launch files](http://wiki.ros.org/roslaunch), configuration files, and dependency declarations. Furthermore, the automation library and tools are also distributed in ROS packages.

We provide run scripts for the containers that mount everything into the same workspace, ready to build and run the different benchmarks.

We've also chosen [rosbag](http://wiki.ros.org/rosbag) as the preferred format for dataset instrumentation. The data formats of publicly available datasets vary significantly, so we've developed [conversion tools](src/tools) for many of them.

### Robot Framework

For the benchmark automation scripts we use [Robot Framework](https://robotframework.org/), which is a generic open source framework for robotic process automation (RPA).

We've extended it with custom libraries that allow you to write benchmarking steps in a human-readable format. This format is flexible enough to adapt to a wide variety of scenarios. Here is an example of the syntax:

```robot
Use /tf /scan /pose data in rgbd_dataset_freiburg2_pioneer_360.bag as input
Track /tf:world.kinect /tf:map.base_link trajectories
And save the resulting map
Use tum_benchmark.launch in cartographer_ros_benchmark package to launch
Use a sampling rate of 20 Hz to track computational performance
Benchmark Cartographer ROS for 100 iterations
```

## Supported metrics

We currently support the following performance metrics:

- Absolute Pose Error (APE) and Relative Pose Error (RPE) using [evo](https://michaelgrupp.github.io/evo/) for localization performance (for more information see [Prokhorov et. al.](https://arxiv.org/pdf/1910.04755.pdf)).
- CPU usage and Resident Set Size (RSS), i.e. the portion of a process’ memory space currently in RAM, using [timem](https://timemory.readthedocs.io/en/develop/features.html#command-line-tools) for process performance.

## Next steps

- Read about `lambkin`'s [architecture](docs/ARCHITECTURE.md).
- Get hands-on experience with the [getting started](docs/GETTING_STARTED.md) tutorial.
