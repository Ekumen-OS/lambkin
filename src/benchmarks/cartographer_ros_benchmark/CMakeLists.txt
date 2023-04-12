cmake_minimum_required(VERSION 3.0.2)
project(cartographer_ros_benchmark)

find_package(catkin REQUIRED)

catkin_package()

install(
  PROGRAMS
    scripts/magazino_benchmark.robot
    scripts/mars_benchmark.robot
    scripts/rawseeds_benchmark.robot
    scripts/tum_benchmark.robot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY config launch reports
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
