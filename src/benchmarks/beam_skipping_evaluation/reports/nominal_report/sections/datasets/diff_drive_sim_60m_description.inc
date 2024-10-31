Dataset information
^^^^^^^^^^^^^^^^^^^

This is a synthetic dataset created recording a simulated robot wandering around a
:math:`450 \mathrm{m}^2` office environment for 24 hours.

The simulated diff-drive robot is modelled after a `Kobuki platform <https://iclebo-kobuki.readthedocs.io/en/latest/about.html>`_ featuring an `RPLidar A1 <https://www.slamtec.com/en/Lidar/A1Spec>`_, an 8m range 2D lidar. The
simulation was performed using Gazebo Classic.

The dataset was recorded in a single run, and it consists of a single continuous bagfile with a duration of 24 hours.
During this time the robot is periodically sent navigation goals to reachable locations to ensure it keeps moving
around the environment.

The robot covers approximately 24 km during those 24 hours, with an average speed of about 0.28 m/s.

The robot model was configured to introduce small imperfections in the odometry to cause a small amount of drift and
emulate conditions closer to real-world.

The simulated environment is populated with static obstacles (furniture), but the localization map intentionally
excludes them to evaluate the systems under test in a condition where unmapped obstacles perturb the estimation.

The localization map used to evaluate the localization performance with this dataset is the one shown below.

.. figure:: assets/representative_diff_drive_sim_24hs_map.png
   :scale: 99 %

   Localization map used to evaluate the localization performance with the Diff Drive Sim 60m dataset.

The simulated environment used to record the Diff Drive Sim 60m dataset is shown below.

.. figure:: assets/hq_simulated_environment.jpg
   :scale: 99 %

   The simulated environment used to record the Diff Drive Sim 60m dataset.
