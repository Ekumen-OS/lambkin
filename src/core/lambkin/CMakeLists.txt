cmake_minimum_required(VERSION 3.0.2)
project(lambkin)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(
  FILES src/lambkin.resource
  DESTINATION ${PYTHON_INSTALL_DIR})
