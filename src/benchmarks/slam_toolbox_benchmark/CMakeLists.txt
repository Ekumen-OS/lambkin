cmake_minimum_required(VERSION 3.0.2)
project(slam_toolbox_benchmark)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(
  PROGRAMS
  scripts/magazino_benchmark.robot
  scripts/rawseeds_benchmark.robot
  scripts/tum_benchmark.robot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
