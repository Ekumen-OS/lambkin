cmake_minimum_required(VERSION 3.16)
project(beluga_vs_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  PROGRAMS
    scripts/nominal.robot
    scripts/swept.robot
    scripts/ndt_swept.robot
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY
    config
    params
    launch
    reports
  DESTINATION share/${PROJECT_NAME})


ament_package()
