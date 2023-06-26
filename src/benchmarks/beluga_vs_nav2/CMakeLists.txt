cmake_minimum_required(VERSION 3.16)
project(beluga_vs_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY
    scripts
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY
    config
    launch
    reports
  DESTINATION share/${PROJECT_NAME})


ament_package()
