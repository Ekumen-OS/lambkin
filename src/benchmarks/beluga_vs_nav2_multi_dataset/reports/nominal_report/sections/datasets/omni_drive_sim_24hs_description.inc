Dataset information
^^^^^^^^^^^^^^^^^^^

This is a synthetic dataset created recording a simulated omni-drive robot wandering around the
`AWS Robomaker Bookstore World <https://github.com/aws-robotics/aws-robomaker-bookstore-world>`_ for 24 hours.

The simulated omni-drive robot is modelled after a customized `Robomaster EP <https://www.dji.com/global/robomaster-ep>`_ featuring an `RPLidar A2 <https://www.slamtec.com/en/Lidar/A2Spec>`_, a 12m range 2D lidar.
The simulation was performed using Gazebo Sim.

The dataset was recorded in a single run, and it consists of a single continuous bagfile with a duration of 24 hours.
During this time the robot is periodically sent navigation goals to reachable locations within the map to ensure it
keeps moving.

The robot covers approximately 45 km during the 24 hours, with an average speed of about 0.5 m/s.

The robot model was configured to introduce small imperfections in the odometry to cause a small amount of drift and
emulate conditions closer to real-world.

Most but not all obstacles and furniture are mapped, and one of the walls is windowed and therefore
invisible to the simulated lidar.

The localization world used to evaluate the localization performance with this dataset is the one shown below.

.. figure:: assets/representative_omni_drive_sim_24hs_map.png
   :scale: 99 %

   Localization world used to evaluate the localization performance with the Omni Drive Sim 24hs dataset.

The simulated world used to record the Omni Drive Sim 24hs dataset is shown below. It differs from the localization world in that it is populated with obstacles and furniture.
in that a few of the obstacles and furniture present in the simulated world are not present.

.. figure:: assets/bookstore_simulated_world.png
   :scale: 99 %

   The simulated environment used to record the Omni Drive Sim 24hs dataset.





