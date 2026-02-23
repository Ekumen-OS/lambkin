cmake_minimum_required(VERSION 3.16)
project(lambkin)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

  
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME})