Dataset information
^^^^^^^^^^^^^^^^^^^

These results are based on a modified version of the TorWIC Mapping dataset,
which was repurposed to serve as a 2D localization test scenario. The original
dataset is available `here <https://github.com/Viky397/TorWICDataset>`_.

About the dataset,

     The purpose of this dataset is to evaluate the map mainteneance capabilities in a warehouse
     environment undergoing incremental changes. This dataset is collected in a Clearpath Robotics
     facility.

About the hardware used to record the dataset,

    The dataset was collected on the `OTTO 100 Autonomous Mobile Robot <https://robotsguide.com/robots/otto>`_, remote controlled by a human
    operator at walking speed. We record sensor measurements from an Intel RealSense D435i RGB-D
    camera, a wheel encoder, an IMU unit, and a Hokuyo UAM501 2D laser scanner, all rigidly mounted
    on the platform. The following figure shows the robot platform and the sensor frames, and the
    following table lists the specifications and formats of the sensor measurements.

About the trajectories,

    The dataset provides 18 trajectories in 4 scenarios, including the baseline setup. Each
    trajectory contains the robot traversing through a static configuration of the environment,
    starting and finishing at the fixed April-Tag. Users can stitch the trajectories together
    with the provided script to create routes with structural changes in the scene. A high-level
    overview of the scenarios and trajectories is listed in the table below.

=========================  ============  ============================
Scenario                   Trajectories  Description
=========================  ============  ============================
Baseline                   1             Default configuration with all box walls and fences forming a square.
Box shifts and rotations   9             Various box walls are rotated and shifted.
Removing boxes             4             Sections of the box walls are removed.
Moving fences              3             Fences are shifted outwards and inwards, along with the box walls.
Adding new boxes           1             The four fences are covered with stacks of boxes.
=========================  ============  ============================

The following is a representative map from the set, to exemplify the type of environment:

.. figure:: assets/representative_torwic_mapping_map.png
   :scale: 99 %

   Localization map of one of the "baseline" trajectory in the TorWIC Mapping dataset.


Transformations to the original dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To repurpose the original dataset for 2D localization evaluation, we made the following modifications:

- The original ROS 1 bagfiles were converted to ROS 2 bagfiles.
- All topics not related to lidar-based 2D localization were removed to reduce file size.
- The map->odom TF transformations were removed from the transform tree.
- The frame names in the /velocity_control/odom topic were corrected. See `this <https://github.com/Viky397/TorWICDataset/issues/6>`_.
- The bagfiles lacked the odom->base_link TF frame, so it was added from the odometry information in the /velocity_control/odom topic. The resulting transform is published at 25 Hz.
- A reference ground-truth and a matching occupancy map were generated for each bagfile using SLAM.


