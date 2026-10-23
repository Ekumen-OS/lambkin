# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(external_vio_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wconversion
    -Wextra
    -Werror
    -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/planar_odometry_projection.cpp)
target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core nav_msgs)

add_executable(planar_odometry_adapter src/main.cpp
                                       src/planar_odometry_adapter.cpp)
target_link_libraries(planar_odometry_adapter ${PROJECT_NAME}_core)
ament_target_dependencies(
  planar_odometry_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}_core planar_odometry_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(planar_odometry_projection_test
                  test/planar_odometry_projection_test.cpp)
  target_link_libraries(planar_odometry_projection_test ${PROJECT_NAME}_core)
  ament_target_dependencies(planar_odometry_projection_test nav_msgs)
endif()

ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros)
ament_export_include_directories(include)
ament_package()
