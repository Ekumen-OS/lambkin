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
