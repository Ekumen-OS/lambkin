.. repl-quiet::

    import lambkin.shepherd.data as lks
    import pandas as pd
    import numpy as np
    import os

    os.makedirs('_generated', exist_ok=True)


Nominal Beluga AMCL vs Nav2 AMCL benchmark
==========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Objective
---------

Compare `Beluga AMCL <https://github.com/Ekumen-OS/beluga>`_ and `Nav2 AMCL <https://github.com/ros-planning/navigation2/tree/main/nav2_amcl>`_ localization systems in terms of localization and computational performance for the nominal case.

Methodology
-----------

TBD

Measurements
------------

Dataset
^^^^^^^

For this report, `Magazino datasets <https://google-cartographer-ros.readthedocs.io/en/latest/data.html#magazino>`_, published with the Cartographer Public Data set under Apache License v2.0, were chosen. As these datasets are distributed in rosbag format, equivalent datasets in rosbag2 format were recreated. As both localization systems need a map to work with, and a groundtruth is necessary for performance evaluation, offline mapping was conducted using Cartographer ROS.

Platform
^^^^^^^^

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
        * OS: {{ os_distribution_info.description  }}
        * ROS:
            {% set ros_distribution_info = data.ros2.distribution_info() %}
            * Distribution: {{ ros_distribution_info.name }}
            * Packages:
              {% for name in ('beluga_amcl', 'nav2_amcl') %}
              {% set pkg_info = data.ros2.package_info(name) %}
                * ``{{ pkg_info.name }}`` {{ pkg_info.version }}
              {% endfor %}

Metrics
^^^^^^^

To characterize the localization performance of both systems, this report uses:

* **APE**. The Absolute Pose Error is the difference between estimated and reference trajectories after alignment when taken as a whole. It is a measure of global accuracy and consistency.

Metrics are aggregated across multiple runs of each parameter variation to ensure statistical significance.


.. repl::

    def rms(x):
        return np.sqrt(np.mean(np.power(x, 2.)))

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'ape', normalization='long'),
        lks.evo.series('/nav2_amcl/pose', 'ape', normalization='long')
    ])

    data = data[[
        'case.name',
        'variation.parameters.dataset',
        'variation.parameters.laser_model',
        'trajectory.name', 'metric.series.value'
    ]]

    data = data.rename(columns = {
        'case.name': 'case_name',
        'variation.parameters.dataset': 'bagfile',
        'variation.parameters.laser_model': 'laser_model',
        'trajectory.name': 'implementation',
    })

    print (data.columns)

    data = data.groupby([
        'case_name',
        'bagfile',
        'implementation',
        'laser_model',
    ])['metric.series.value']

    ape = data.agg([rms, 'mean', 'std', 'max']).round(3)
    ape.to_pickle('_generated/ape.pkl')

    from ament_index_python import get_package_share_directory

    this_pkg_dir = get_package_share_directory('beluga_vs_nav2_multi_dataset')

    with open(f'{this_pkg_dir}/reports/nominal_report/template_table_header.txt', 'r') as f:
        TABLE_HEADER_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/template_table_row.txt', 'r') as f:
        ROW_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/template_dataset_chapter.txt', 'r') as f:
        DATASET_CHAPTER_TEMPLATE = f.read()

    with open(f'{this_pkg_dir}/reports/nominal_report/template_bagfile_section.txt', 'r') as f:
        BAGFILE_SECTION_TEMPLATE = f.read()

    output = []

    for case_name in ape.index.get_level_values(0).unique():
        output.append(DATASET_CHAPTER_TEMPLATE.format(dataset_name=case_name))
        data_for_case = ape.loc[case_name]
        for bagfile in data_for_case.index.get_level_values(0).unique():
            output.append(BAGFILE_SECTION_TEMPLATE.format(bagfile_name=bagfile))
            data_for_bagfile = data_for_case.loc[bagfile]
            try:
                beluga_likelihood_data = data_for_bagfile.loc['/beluga_amcl/pose', 'likelihood_field']
                beluga_beam_data = data_for_bagfile.loc['/beluga_amcl/pose', 'beam']
                beluga_likelihood_rms = beluga_likelihood_data['rms']
                beluga_likelihood_mean = beluga_likelihood_data['mean']
                beluga_likelihood_std = beluga_likelihood_data['std']
                beluga_likelihood_max = beluga_likelihood_data['max']
                beluga_beam_rms = beluga_beam_data['rms']
                beluga_beam_mean = beluga_beam_data['mean']
                beluga_beam_std = beluga_beam_data['std']
                beluga_beam_max = beluga_beam_data['max']
            except:
                beluga_likelihood_rms = beluga_likelihood_mean = beluga_likelihood_std = beluga_likelihood_max = "???"
            try:
                nav2_likelihood_data = data_for_bagfile.loc['/nav2_amcl/pose', 'likelihood_field']
                nav2_beam_data = data_for_bagfile.loc['/nav2_amcl/pose', 'beam']
                nav2_likelihood_rms = nav2_likelihood_data['rms']
                nav2_likelihood_mean = nav2_likelihood_data['mean']
                nav2_likelihood_std = nav2_likelihood_data['std']
                nav2_likelihood_max = nav2_likelihood_data['max']
                nav2_beam_rms = nav2_beam_data['rms']
                nav2_beam_mean = nav2_beam_data['mean']
                nav2_beam_std = nav2_beam_data['std']
                nav2_beam_max = nav2_beam_data['max']
            except:
                nav2_likelihood_rms = nav2_likelihood_mean = nav2_likelihood_std = nav2_likelihood_max = "???"
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Likelihood field',
                nrows=len(data_for_bagfile),
                table_name='hallway-localization-ape-comparison'))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                rms=beluga_likelihood_rms,
                mean=beluga_likelihood_mean,
                std=beluga_likelihood_std,
                max=beluga_likelihood_max))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                rms=nav2_likelihood_rms,
                mean=nav2_likelihood_mean,
                std=nav2_likelihood_std,
                max=nav2_likelihood_max))
            output.append(TABLE_HEADER_TEMPLATE.format(
                trajectory_name=bagfile,
                sensor_model='Beam',
                nrows=len(data_for_bagfile),
                table_name='hallway-localization-ape-comparison'))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Beluga AMCL",
                rms=beluga_beam_rms,
                mean=beluga_beam_mean,
                std=beluga_beam_std,
                max=beluga_beam_max))
            output.append(ROW_TEMPLATE.format(
                implementation_name="Nav2 AMCL",
                rms=nav2_beam_rms,
                mean=nav2_beam_mean,
                std=nav2_beam_std,
                max=nav2_beam_max))

    output = '\n\n'.join(output)

    with open('_generated/generated_section.inc', 'w') as f:
        f.write(output)

.. include:: _generated/generated_section.inc


Appendices
----------

Configuration
^^^^^^^^^^^^^

For this report, the following baseline configuration:

.. datatemplate:import-module:: ament_index_python

    {% set package_path = data.get_package_share_directory('beluga_vs_nav2_multi_dataset') %}

    .. literalinclude:: {{config.sysroot}}/{{ package_path }}/params/amcl.yaml
        :language: yaml

was modified for each benchmark case in terms of:

* the laser sensor model, to assess both beam and likelihood models (see sections 6.3 and 6.4 of Probabilistic Robotics, by Thrun et al);
* the execution policy, to compare single-threaded and multi-threaded performance. Note this feature is only provided by Beluga AMCL.

so as to have a reasonably complete picture of how both localization systems perform.
