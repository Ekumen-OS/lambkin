Dataset information
^^^^^^^^^^^^^^^^^^^

These results are based on a modified version of the OpenLORIS Scene: Office dataset,
which was repurposed to serve as a 2D localization test scenario. The original
dataset is available `here <https://lifelong-robotic-vision.github.io/dataset/scene.html>`_.

About the dataset,

     The OpenLORIS-Scene dataset aims to help evaluate the maturity of SLAM and
     scene understanding algorithms for real-world deployment, by providing
     visual, inertial and odometry data recorded with real robots in real
     scenes, and ground-truth robot trajectories acquired by motion capture
     system or high-resolution LiDARs. Multiple trajectories are provided for
     each scene, to capture scene changes caused by human activities, day-night
     shifts and other factors, which are crucial to address for long-term robot
     autonomy.

About the hardware used to record the dataset,

    The data were collected by a wheeled robot moving at human walking speed or slower.
    The primary sensors include a RealSense D435i camera and a RealSense T265 camera,
    both mounted at a fixed height of about 1m. The color images and depth images from
    D435i are recommended for monocular/RGB-D algorithms, while the dual fisheye images
    from T265 are recommended for stereo algorithms. Both provide IMU measurements with
    hardware synchronization with corresponding images. Odometry data from wheel encoders
    are also be provided.

This analysis only uses the "Office" subset of the OpenLORIS-Scene dataset.

The following is a representative map from the set, to exemplify the type of environment:

.. figure:: assets/representative_openloris_office_map.png
   :scale: 99 %

   Localization map of one of the trajectories in the OpenLORIS Scene, representative of the environment.


Transformations to the original dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To repurpose the original dataset for 2D localization evaluation, we made the following modifications:

- The original ROS 1 bagfiles were converted to ROS 2 bagfiles.
- All topics not related to lidar-based 2D localization were removed to reduce file size.
- A reference ground-truth and a matching occupancy map were generated for each bagfile using SLAM.


