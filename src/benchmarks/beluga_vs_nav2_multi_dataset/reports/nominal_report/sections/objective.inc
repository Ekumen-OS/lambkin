Objective
---------

Compare `Beluga AMCL <https://github.com/Ekumen-OS/beluga>`_ and `Nav2 AMCL <https://github.com/ros-planning/navigation2/tree/main/nav2_amcl>`_ localization systems in terms of localization and computational performance for the nominal case.


Dataset
^^^^^^^

For this report, `Magazino datasets <https://google-cartographer-ros.readthedocs.io/en/latest/data.html#magazino>`_, published with the Cartographer Public Data set under Apache License v2.0, were chosen. As these datasets are distributed in rosbag format, equivalent datasets in rosbag2 format were recreated. As both localization systems need a map to work with, and a groundtruth is necessary for performance evaluation, offline mapping was conducted using Cartographer ROS.


Metrics
^^^^^^^

To characterize the localization performance of both systems, this report uses:

* **APE**. The Absolute Pose Error is the difference between estimated and reference trajectories after alignment when taken as a whole. It is a measure of global accuracy and consistency.

Metrics are aggregated across multiple runs of each parameter variation to ensure statistical significance.

