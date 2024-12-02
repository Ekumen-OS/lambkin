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

For the sake of brevity, a limited number of configuration variants are tested in this benchmark. For each dataset both
the beam and the likelihood field sensor models are evaluated. Both differential drive
and omnidirectional robots are represented in the input datasets.
The rest of the configuration parameters are kept at their default values, which are the same for both Beluga AMCL and Nav2 AMCL.




