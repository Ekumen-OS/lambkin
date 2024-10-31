Dataset information
^^^^^^^^^^^^^^^^^^^

These results are based on a modified version of the Cartographer Magazino dataset,
which was repurposed to serve as a 2D localization test scenario. The original
dataset is available `here <https://github.com/magazino/cartographer_magazino>`_.

There's little information about this dataset. In the repository linked
above it is mentioned that the dataset was recorded
in a hallway using a Magazino `TORU robot <https://robotsguide.com/robots/toru>`_.

The set contains only two bagfiles, corresponding to two different trajectories along the same hallway.
"Hallway Localization" is a bit over 2 minutes long, while "Hallway Return" is a little below 4
minutes long.

The following is a representative map from the set:

.. figure:: assets/representative_cartographer_magazino_map.png
   :scale: 99 %

   Localization map of one of the trajectories in the Cartographer Magazino dataset.

Transformations to the original dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To repurpose the original dataset for 2D localization evaluation, we made the following modifications:

- The original ROS 1 bagfiles were converted to ROS 2 bagfiles.
- All topics not related to lidar-based 2D localization were removed to reduce file size.
- A reference ground-truth and a matching occupancy map were generated for each bagfile using SLAM.




