Dataset information
^^^^^^^^^^^^^^^^^^^

These results are based on a modified version of the TorWIC SLAM dataset,
which was repurposed to serve as a 2D localization test scenario. The original
dataset is no longer available online, but it was described in the following reference:

    “`An Object-Based Semantic World Model for Long-Term Change Detection and Semantic
    Querying. <https://web.archive.org/web/20151207202459id_/http://cgi.cs.duke.edu/~mac/papers/iros12_sm.pdf>`_”, by Julian Mason and Bhaskara Marthi, IROS 2012.

About the hardware used to record the dataset, as described in the reference quoted above:

    Our mobile robot is a `Willow Garage PR2 <https://robotsguide.com/robots/pr2>`_ with a headmounted Microsoft Kinect. When the robot is in motion,
    the Kinect is roughly 1.5 meters off the floor, and captures
    frames at 5Hz. The robot travels at 0.5 m/s. We rely on
    the PR2’s other sensors and tools in ROS [19] to provide
    autonomous navigation and accurate localization, but only
    the current localization estimate and the Kinect’s RGB-D
    point cloud are provided to our perceptual pipeline, detailed
    in Section V. As a result, the PR2 is not a prerequisite for
    our system; we only require a localized RGB-D camera. We
    do require that the sensor be fairly far from the floor, in order
    to perceive supporting surfaces and the objects on them

The dataset provides 67 trajectories distributed over multiple days and time periods. All of the trajectories
are in the same office-like environment, and represent on average 15 minutes of robot operation. The shortest
trajectory is 8.5 minutes long, and the longest is 30 minutes long.

The following is a representative map from the set, to exemplify the type of environment:

.. figure:: assets/representative_willow_garage_map.png
   :scale: 99 %

   Localization map of one of the trajectories in the Willow Garage dataset, representative of the environment.

Transformations to the original dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To repurpose the TorWIC Mapping dataset for 2D localization evaluation, we made the following changes from
the original dataset as distributed in here:

- The original ROS 1 bagfiles were converted to ROS 2 bagfiles.
- All topics not related to lidar-based 2D localization were removed to reduce file size.
- The map->odom_combined TF transformations were removed from the transform tree.
- The "/" prefix in all transform frame names was removed to avoid issues with the ROS 2 TF tree.
- A reference ground-truth and a matching occupancy map were generated for each bagfile using SLAM.










