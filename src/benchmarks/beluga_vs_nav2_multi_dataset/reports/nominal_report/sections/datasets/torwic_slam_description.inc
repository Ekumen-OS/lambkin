Dataset information
^^^^^^^^^^^^^^^^^^^

These results are based on a modified version of the TorWIC SLAM dataset,
which was repurposed to serve as a 2D localization test scenario. The original
dataset is available `here <https://github.com/Viky397/TorWICDataset>`_.

About the dataset,

    The purpose of this dataset is to evaluate the SLAM capabilities in a warehouse environment
    undergoing incremental changes. This dataset is collected in a Clearpath Robotics facility
    on three data collections days over a four months span.

About the hardware used to record the dataset,

    The dataset was collected on the `OTTO 100 Autonomous Mobile Robot <https://robotsguide.com/robots/otto>`_, remote controlled by
    a human operator at walking speed. We record sensor measurements from two Azure Kinect RGB-D
    cameras and a Ouster OS1-128 3D LiDAR, all rigidly mounted on the top the platform. The
    following figure shows the robot platform and the sensor frames, and the following table
    lists the specifications and formats of the sensor measurements.

About the trajectories,

    The dataset provides 20 trajectories in 3 scenarios, captured on 3 data collections days over
    4 months. Each trajectory contains the robot traversing through regions of the warehouse,
    following a predefined path in clockwise and counter-clockwise directions. The robot always
    starts at the origin of our map frame. Users can stitch the trajectories together to create
    routes with structural changes in the scene.

=========================  ============  ============================
Scenario                   Trajectories  Description
=========================  ============  ============================
Aisles - CW                5             Moving through the aisle area clockwise.
Aisles - CCW               5             Moving through the aisle area counter-clockwise.
Hallway straight - CW      2             Moving through the straight hallway area clockwise.
Hallway straight - CCW     3             Moving through the straight hallway area counter-clockwise.
Hallway Full - CW          4             Moving through the hallway, aisle and, receiving area clockwise.
Hallway Full - CCW         1             Moving through the hallway, aisle and, receiving area counter-clockwise.
=========================  ============  ============================

About the recording dates,

================  ===========================
Date              Number of trajectories
================  ===========================
Jun. 15, 2022     7
Jun. 23, 2022     7
Oct. 12, 2022     6
================  ===========================

The following is a representative map from the set, to exemplify the type of environment:

.. figure:: assets/representative_torwic_slam_map.png
   :scale: 50 %

   Localization map of one of the TorWIC SLAM dataset trajectories, representative of the environment.

Transformations to the original dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To repurpose the TorWIC Mapping dataset for 2D localization evaluation, we made the following changes from
the original dataset as distributed in here:

- The original ROS 1 bagfiles were converted to ROS 2 bagfiles.
- All topics not related to lidar-based 2D localization were removed to reduce file size.
- The map->odom TF transformations were removed from the transform tree.
- A reference ground-truth and a matching occupancy map were generated for each bagfile using SLAM.


