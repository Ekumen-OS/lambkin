cmake_minimum_required(VERSION 3.16)
project(lambkin_ros2)
find_package(ament_cmake REQUIRED)
find_package(beluga_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)



install(
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  launch
  launch_ros
)
ament_package()
