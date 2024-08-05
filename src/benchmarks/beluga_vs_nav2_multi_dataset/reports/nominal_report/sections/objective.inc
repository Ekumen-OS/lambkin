Introduction
------------

Objective
^^^^^^^^^

To gather data on the localization performance and resource usage of both
`Beluga AMCL <https://github.com/Ekumen-OS/beluga>`_ and `Nav2 AMCL <https://github.com/ros-planning/navigation2/tree/main/nav2_amcl>`_
when used in a diversity of pre-recorded environments.

This data will be used to further improve the state of localization in ROS 2 and detect potential issues in the current implementations.

Scope
^^^^^

This document is limited to the presentation of the methodology and the results used to gather the performance data.

No root-cause analysis of issues or performance tuning is performed in this document.

For the sake of brevity, a limited number of configuration variants are used in the experiments: the sensor model and the motion model.
The rest of the configuration parameters are kept at their default values, which are the same for both AMCL implementations.




