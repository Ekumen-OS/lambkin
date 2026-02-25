"""Launch file for the Beluga AMCL benchmarking environment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generates the launch description for the Beluga AMCL benchmarking environment.

    Declares launch arguments for the map path, laser model type,
    and maximum particles,and configures the required ROS 2 nodes:
    beluga_amcl, map_server, and lifecycle_manager.

    Returns:
        LaunchDescription: The complete ROS 2 launch description object.
    """
    config_file_arg = DeclareLaunchArgument(
        "beluga_config_path",
        default_value="/example/defaul_params.ros.yaml",  # Opcional por defecto
        description="Absolute YAML file path",
    )

    map_path_arg = DeclareLaunchArgument("map_path", description="Absolute map path")

    laser_model_arg = DeclareLaunchArgument(
        "laser_model_type",
        default_value="likelihood_field",
        description="Sensor model",
        choices=["likelihood_field", "beam"],
    )

    max_particles_arg = DeclareLaunchArgument(
        "max_particles", default_value="2000", description="Max number of particles"
    )

    beluga_node = Node(
        package="beluga_amcl",
        executable="amcl_node",
        name="beluga_amcl",
        output="screen",
        parameters=[
            {
                "laser_model_type": LaunchConfiguration("laser_model_type"),
                "max_particles": LaunchConfiguration("max_particles"),
            }
        ],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map_path")}],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server", "beluga_amcl"]}],
    )

    return LaunchDescription(
        [
            config_file_arg,
            map_path_arg,
            laser_model_arg,
            max_particles_arg,
            beluga_node,
            map_server_node,
            lifecycle_manager_node,
        ]
    )
