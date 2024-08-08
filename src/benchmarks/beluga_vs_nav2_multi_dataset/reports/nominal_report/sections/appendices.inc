Appendix I: Lambkin Configuration
---------------------------------

For this report, the following benchmarking script was used:

.. datatemplate:import-module:: ament_index_python

    {% set package_path = data.get_package_share_directory('beluga_vs_nav2_multi_dataset') %}

    .. literalinclude:: {{config.sysroot}}/{{ package_path }}/scripts/nominal.robot
        :language: robotframework


Appendix II: AMCL Node Configuration
------------------------------------

For this report, the following baseline configuration:

.. datatemplate:import-module:: ament_index_python

    {% set package_path = data.get_package_share_directory('beluga_vs_nav2_multi_dataset') %}

    .. literalinclude:: {{config.sysroot}}/{{ package_path }}/params/amcl.yaml
        :language: yaml

was modified for each benchmark case in terms of:

* the laser sensor model, to assess both beam and likelihood models (see sections 6.3 and 6.4 of Probabilistic Robotics, by Thrun et al);
* the execution policy, to compare single-threaded and multi-threaded performance. Note this feature is only provided by Beluga AMCL.

so as to have a reasonably complete picture of how both localization systems perform.
