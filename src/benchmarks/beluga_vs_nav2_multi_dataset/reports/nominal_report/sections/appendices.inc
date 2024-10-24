Appendix I: Lambkin Configuration
---------------------------------

For this report, the following benchmarking script was used:

.. datatemplate:import-module:: ament_index_python

    {% set package_path = data.get_package_share_directory('beluga_vs_nav2_multi_dataset') %}

    .. literalinclude:: {{config.sysroot}}/{{ package_path }}/scripts/nominal.robot
        :language: robotframework


Appendix II: AMCL Node Configuration
------------------------------------


In all cases, the configuration of both the Beluga AMCL and Nav2 AMCL nodes was kept as close as
possible to the default configuration provided by the respective packages. This configuration is
detailed below.

The only deviations on top of this configuration were the following settings, which were modified
according to the requirements of each particular dataset and sensor model configuration used:

- The laser sensor model, to assess both beam and likelihood models.
- The name of the base link frame.
- The name of the map frame.
- The name of the odom frame.
- The 2D scan topic name.

.. datatemplate:import-module:: ament_index_python

    {% set package_path = data.get_package_share_directory('beluga_vs_nav2_multi_dataset') %}

    .. literalinclude:: {{config.sysroot}}/{{ package_path }}/params/amcl.yaml
        :language: yaml

