cmake_minimum_required(VERSION 3.16)
project(beluga_vs_nav2_unmapped_areas)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  PROGRAMS
    scripts/nominal.robot
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    config
    params
    launch
    reports
    scripts
  DESTINATION share/${PROJECT_NAME})


ament_package()
