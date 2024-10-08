SLAM Toolbox 2D SLAM benchmarks with TUM dataset
================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Definition
----------

Configuration
^^^^^^^^^^^^^

SLAM Toolbox performance was put to test as follows. Selected parameters
include: the datasets that were used, the input sensor topics, the
trajectories to compare, and the amount of iterations.

.. include:: ${benchmark_source}
   :code: robotframework

Environment
^^^^^^^^^^^

- OS: Ubuntu 20.04 LTS
- ROS: ROS Noetic Ninjemys
- SLAM system: SLAM Toolbox 1.5.6

Dataset
^^^^^^^

TUM dataset consists of four sequences from an ActivMedia Pioneer 3 robot.
In the *Pioneer 360* sequence, the robot was driven in a loop around the center of the (mostly) empty hall.
The other sequences were recorded in a large hall with several office containers and boxes.
*Pioneer SLAM 1*, *Pioneer SLAM 2* and *Pioneer SLAM 3* differ in the actual trajectories but
all contain several loop closures.

A motion capture system from MotionAnalysis was used to record the ground-truth trajectory,

Computer Vision Group, TUM Department of Informatics, Technical University of Munich.
https://vision.in.tum.de/data/datasets/rgbd-dataset/download

A Benchmark for the Evaluation of RGB-D SLAM Systems (J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers),
In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
http://dx.doi.org/10.1109/IROS.2012.6385773

Metrics
^^^^^^^

The following metrics were selected to understand different aspects of the SLAM system
under test.

To estimate quality of the SLAM system:

- **APE** (Absolute Pose Error): compares the reconstructed trajectory to the actual trajectory.
- **RPE** (Relative Pose Error): compares the reconstructed relative transformations between
  nearby poses to the actual relative transformations.

To estimate performance of the SLAM system:

- **RSS** (Resident Set Size): amount of space of physical memory (RAM) held by the SLAM system.
- **CPU Usage**: ratio between the time spent by the SLAM system, in both user and kernel space,
  and the total system time elapsed.

Aggregated information is provided for statistical analysis of the results.
For each metric, its median, mean, and standard deviation are provided.

Results
-------

In the following sections, the metrics generated by the benchmarking pipeline are presented.

.. list-table:: Metric value as a function of map resolution

   * - .. plot::

         data = lks.data.pandas.inner_join([
             lks.data.evo.stats('/tf:map.base_link', 'ape'),
             lks.data.evo.stats('/tf:map.base_link', 'rpe'),
             lks.data.timem.summary('sync_slam_toolbox_node')
         ], on=[
             'case.name', 'iteration.index',
             'variation.parameters.map_resolution',
             'variation.parameters.search_resolution'
         ])

         data = lks.data.pandas.rescale(data, {
             'sync_slam_toolbox_node.summary.cpu_util': 100,
             'sync_slam_toolbox_node.summary.peak_rss': 1 / 8e6
         })

         data = data.melt(
             var_name='metric',
             value_name='value',
             id_vars=[
                 'case.name', 'iteration.index',
                 'variation.parameters.map_resolution',
                 'variation.parameters.search_resolution'
             ],
             value_vars=[
                 '/tf:map.base_link.ape.rmse',
                 '/tf:map.base_link.rpe.rmse',
                 'sync_slam_toolbox_node.summary.cpu_util',
                 'sync_slam_toolbox_node.summary.peak_rss'
             ],
         )

         data = data.replace({'metric': {
             '/tf:map.base_link.ape.rmse': 'APE rmse (m)',
             '/tf:map.base_link.rpe.rmse': 'RPE rmse (m)',
             'sync_slam_toolbox_node.summary.cpu_util': 'CPU usage (%)',
             'sync_slam_toolbox_node.summary.peak_rss': 'Peak RSS (MB)'
         }})

         grid = sns.relplot(
             data=data, x='variation.parameters.map_resolution', y='value', hue='case.name',
             style='variation.parameters.search_resolution', col='metric', col_wrap=2,
             kind='line', facet_kws={'sharey': False, 'sharex': True}
         )
         grid.set_axis_labels(xlabel='Map resolution', ylabel='')
         grid.tight_layout()

.. list-table:: Metric value as a function of search resolution

   * - .. plot::

         data = lks.data.pandas.inner_join([
             lks.data.evo.stats('/tf:map.base_link', 'ape'),
             lks.data.evo.stats('/tf:map.base_link', 'rpe'),
             lks.data.timem.summary('sync_slam_toolbox_node')
         ], on=[
             'case.name', 'iteration.index',
             'variation.parameters.map_resolution',
             'variation.parameters.search_resolution'
         ])

         data = lks.data.pandas.rescale(data, {
             'sync_slam_toolbox_node.summary.cpu_util': 100,
             'sync_slam_toolbox_node.summary.peak_rss': 1 / 8e6
         })

         data = data.melt(
             var_name='metric',
             value_name='value',
             id_vars=[
                 'case.name', 'iteration.index',
                 'variation.parameters.map_resolution',
                 'variation.parameters.search_resolution'
             ],
             value_vars=[
                 '/tf:map.base_link.ape.rmse',
                 '/tf:map.base_link.rpe.rmse',
                 'sync_slam_toolbox_node.summary.cpu_util',
                 'sync_slam_toolbox_node.summary.peak_rss'
             ],
         )

         data = data.replace({'metric': {
             '/tf:map.base_link.ape.rmse': 'APE rmse (m)',
             '/tf:map.base_link.rpe.rmse': 'RPE rmse (m)',
             'sync_slam_toolbox_node.summary.cpu_util': 'CPU usage (%)',
             'sync_slam_toolbox_node.summary.peak_rss': 'Peak RSS (MB)'
         }})

         grid = sns.relplot(
             data=data, x='variation.parameters.search_resolution', y='value', hue='case.name',
             style='variation.parameters.map_resolution', col='metric', col_wrap=2,
             kind='line', facet_kws={'sharey': False, 'sharex': True}
         )
         grid.set_axis_labels(xlabel='Search resolution', ylabel='')
         grid.tight_layout()

.. raw:: latex

   \clearpage

Metrics
^^^^^^^

.. list-table:: Time series

   * - .. plot::

         sns.lineplot(
             lks.data.timem.history('sync_slam_toolbox_node'),
             x='sync_slam_toolbox_node.series.time',
             y='sync_slam_toolbox_node.series.page_rss',
             style='variation.parameters.search_resolution',
             hue='variation.parameters.map_resolution', n_boot=20
         )
         plt.gca().set(xlabel='Time (s)', ylabel='RSS (MB)')
         plt.tight_layout()


Estimated trajectories
^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Trajectories in the first runs before alignment

   * - .. plot::

         only_first_iterations = [
             location
             for location in lks.data.access.iterations()
             if location.metadata['iteration']['index'] == 1
         ]

         data = pd.concat([
             lks.data.evo.trajectory('/tf:map.base_link', only_first_iterations),
             lks.data.evo.trajectory('/tf:odom.base_link', only_first_iterations)
         ]).sort_values(by=['case.name', 'iteration.index', 'trajectory.name', 'trajectory.time'])

         ax = sns.relplot(
             data, x='trajectory.x', y='trajectory.y',
             col='case.name', style='trajectory.name',
             col_wrap=2, kind='line', sort=False, lw=1
         )
         plt.gca().set(xlabel='X (m)', ylabel='Y (m)')
         plt.tight_layout()
