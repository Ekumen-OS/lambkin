Methodology
-----------

TBD

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

