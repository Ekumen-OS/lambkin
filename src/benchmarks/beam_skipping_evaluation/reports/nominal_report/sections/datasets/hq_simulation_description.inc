Dataset information
^^^^^^^^^^^^^^^^^^^

This is a synthetic dataset created recording a simulated robot wandering around a
:math:`450 \mathrm{m}^2` office environment. The robot is modeled after a 
`Kobuki platform <https://iclebo-kobuki.readthedocs.io/en/latest/about.html>`_ featuring
an `RPLidar A1 <https://www.slamtec.com/en/Lidar/A1Spec>`_, an 8m range 2D lidar. The
simulation was performed using Gazebo Classic.

The dataset was recorded in a single run, and then subdivided into multiple 30 minute segments. In all cases the 
robot is navigating between a number a of randomly selected waypoints. The average speed of the robot is about 0.28 m/s.

The robot simulation model was configured to have small imperfections in the odometry readings. This introduces
a small amount of odometry drift and emulates conditions closer to real-world data.

The simulated environment is populated with static obstacles (furniture). These obstacles are not included in the occupancy
grid map. This allows using this dataset to evaluate the impact of unmapped obstacles in localization performance.

Each bag file has its own occupancy grid map, which is a isometric transformation of the map shown in the figure below.
The transformations are intended to ensure that the initial pose of the robot is (0m, 0m, 0 deg) in all cases.

.. figure:: assets/representative_diff_drive_sim_24hs_map.png
   :scale: 99 %

   Representative occupancy grid map of a bag file in the set.

The distribution of the furniture in the environment can be observed in the image below.

.. figure:: assets/hq_simulated_environment.jpg
   :scale: 99 %

   The simulated environment used to record the dataset.



