Introduction
------------

Objective
^^^^^^^^^

To gather data on the localization performance and resource usage of both
`Beluga AMCL <https://github.com/Ekumen-OS/beluga>`_ and `Nav2 AMCL <https://github.com/ros-planning/navigation2/tree/main/nav2_amcl>`_
when used in a variety of pre-recorded environments.

This data will be used to further improve the state of localization in ROS 2 and detect potential issues in the current implementations.

Scope
^^^^^

This document is limited to the presentation of the methodology, description of the input datasets, and enumeration of the results.
Analysis and interpretation of the results will be performed in the article or blog post that will be created
to make these results public.

A limited number of configuration variants are used in the experiments: the sensor model and the motion model.
Both differential drive and omnidirectional robots are used in the experiments, therefore exercising the different motion models available in the AMCL implementations.
The rest of the configuration parameters are kept at their default values, which are the same for both AMCL implementations.




