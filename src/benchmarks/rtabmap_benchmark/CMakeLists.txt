cmake_minimum_required(VERSION 3.0.2)
project(rtabmap_benchmark)

find_package(catkin REQUIRED)

catkin_package()

install(
  PROGRAMS
    scripts/tum_benchmark.robot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY config launch reports
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
