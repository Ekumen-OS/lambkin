Methodology
-----------

Datasets used
^^^^^^^^^^^^^

A number of pre-recorded datasets were selected to evaluate the performance of the Beluga AMCL and Nav2 AMCL localization systems.
A number of public datasets were considered for this evaluation, looking for a diversity of environments and robots that were representative of a wide range of real-world scenarios.

The following datasets were selected based on their characteristics and availability for this evaluation:

.. list-table:: Real robot datasets
   :widths: 25 70
   :header-rows: 1

   * - Name
     - Description
   * - `Willow Garage dataset <https://web.archive.org/web/20151207202459id_/http://cgi.cs.duke.edu/~mac/papers/iros12_sm.pdf>`_
     - Large office environment with multiple recordings along different trajectories and times, including multiple rooms and hallways.
   * - `TorWIC SLAM dataset <https://github.com/Viky397/TorWICDataset>`_
     - Warehouse environment with multiple recordings along different trajectories and times.
   * - `TorWIC Mapping dataset <https://github.com/Viky397/TorWICDataset>`_
     - Trajectories in a synthetic warehouse-like environment undergoing incremental changes.
   * - `Cartographer Magazino <https://github.com/magazino/cartographer_magazino>`_
     - Two sample datasets for mapping and localization that were recorded in a hallway.
   * - `OpenLORIS-Scene dataset (Office set) <https://lifelong-robotic-vision.github.io/dataset/scene.html>`_
     - Short trajectories in an office environment.

None of these datasets were originally conceived for the evaluation of 2D localization systems,
and therefore none of them provided a complete ground-truth that included both the real-world trajectory and the environment's
occupancy grid map.

For this reason ground-truth data was generated locally from the bagfiles themselves using Cartographer SLAM to create mutually
consistent ground-truth trajectory and occupancy grid map for each bagfile.

It has to be notices that this approach has some drawbacks that need to be taken into account when interpreting the results later:

- Both the space and time diversity of the original datasets is lost, as the ground-truth is generated for each individual bagfile representing the state of the environment as perceived when the data was recorded.
- All of the objects present in the environment are part of the generated occupancy grid map; i.e. there are no unmapped objects.

These real robot datasets have two more limitations worth mentioning:

- The longest run time of any bagfile the real robot datasets listed above is around 30 minutes, which may not detect longer-term resource use issues, such as memory leaks.
- In all cases the datasets were recorded using differential-drive robots.

To somewhat mitigate these issues two additional synthetic datasets were added to the evaluation:

.. list-table:: Synthetic datasets
   :widths: 25 70
   :header-rows: 1

   * - Name
     - Description
   * - Diff Drive Sim 24hs
     - Gazebo Classic simulation of a diff-drive robot randomly wandering around a 450m^2 office environment with both mapped and unmapped furniture for just over 24hs. The simulated robot is modelled on a Kobuki platform with a mounted RPLidar A1.
   * - Omni Drive Sim 24hs
     - Gazebo Sim simulation of a omni-drive robot randomly wandering around the `AWS Robomaker Bookstore World <https://github.com/aws-robotics/aws-robomaker-bookstore-world>`_ with both mapped and unmapped furniture for just over 24hs. The simulated robot is modelled after a customized Robomaster EP with a mounted RPLidar A2 M12.

In both cases small imperfections were added to the simulated models to cause a small amount of drift in the odometry. Ground-truth was provided by Gazebo plugins from
world state information, and the occupancy grid maps were generated with using `SLAM Toolbox <https://github.com/SteveMacenski/slam_toolbox>`_.



Alterations to the real robot datasets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A number of transformations were applied to the original real robot datasets to make them compatible
with the systems under test. The most important of all is that they needed to be converted from ROS 1 to ROS 2 using the
`rosbags <https://gitlab.com/ternaris/rosbags>`_ conversion tool.

In some cases, important frame transforms were missing in the original datasets, such as the odom/base_link transform,
and they were generated from other information available in the dataset (e.g. odometry messages). Redundant information,
such as map/odom transforms, were removed to prevent conflicts with the systems under test.

Additionally, all topics not related to lidar-based 2D localization were removed to reduce file size. This was needed because
the evaluation process is very intensive in terms of storage requirements.

The two synthetic datasets were constructed for this evaluation and therefore were ROS 2 native with no missing data.

Evaluation procedure
^^^^^^^^^^^^^^^^^^^^

The evaluation was performed using the `LAMBKIN <https://gitlab.com/ternaris/lambkin>`_ framework,
which is tool described as "a mixture of automation and conventions to facilitate reproducible
benchmarking and evaluation of localization and mapping systems".

For each bagfile in each dataset and each tested configuration (likelihood or beam), LAMBKIN
replays the data through both Beluga AMCL and Nav2 AMCL at the same time, recording the output of both
in a new bagfile. This bagfile is then processed using the `evo <https://github.com/MichaelGrupp/evo>`_ tool to
summarize the localization performance of both systems against the ground-truth data.

During execution LAMBKIN also instruments both localization nodes using `timememory <https://timemory.readthedocs.io/en/develop/>`_ to
collect resource usage metrics such as CPU time, memory usage, and other system-level metrics. This information is stored along with the
results of the evo.

Each evaluation can be iterated multiple times to improve the statistical significance of the results. This comes
at the expense of increased execution time and storage requirements for the results, which can be substantial. As a
compromise, the results in this report are based on 3 iterations of each bagfile/configuration combination for real robot datasets,
and a single iteration for the synthetic datasets accounting for their very long length.

For each bagfile/configuration combination, the resulting APE metrics for all iterations are processed to produce
the median, mean, standard deviation and worst-case value. These are the values reported in the following pages.

We also reported the peak RSS (Resident Set Size) and CPU use across all iterations for each bagfile/configuration combination, as
recorded by timememory.


Evaluation Host Platform
------------------------

.. datatemplate:import-module:: lambkin.clerk

    * Hardware
      {% set cpu_info = data.hardware.cpu_info() %}
        * CPU: {{ cpu_info.description }}
            {% for cache in cpu_info.caches %}
            * {{ cache }}
            {% endfor %}
        {% set memory_info = data.hardware.memory_info() %}
        * Memory: {{ '{:~P}'.format(memory_info.ram_size.to('MB')) }}
    * Software
      {% set os_distribution_info = data.os.distribution_info() %}
      {% set ros_distribution_info = data.ros2.distribution_info() %}
        * OS: {{ os_distribution_info.description  }}
        * ROS distribution: {{ ros_distribution_info.name }}
        * Localization packages:
            {% for name in ('beluga_amcl', 'nav2_amcl') %}
             {% set pkg_info = data.ros2.package_info(name) %}
             * ``{{ pkg_info.name }}`` {{ pkg_info.version }}
            {% endfor %}

