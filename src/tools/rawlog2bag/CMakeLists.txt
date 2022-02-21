cmake_minimum_required(VERSION 3.0.2)
project(rawlog2bag)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rosbag
  roscpp
  roslint
  sensor_msgs
  tf2_msgs
)

find_package(mrpt-math REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-rtti REQUIRED)
find_package(mrpt-system REQUIRED)
find_package(mrpt-tclap REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rawlog2bag
  src/cli.cpp
  src/conversions.cpp
  src/groundtruth_stream.cpp
  src/junction_stream.cpp
  src/rawlog_stream.cpp
)

target_link_libraries(rawlog2bag
  mrpt::math
  mrpt::obs
  mrpt::rtti
  mrpt::tclap
  mrpt::system
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rawlog2bag
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############

set(ROSLINT_CPP_OPTS
  --filter=-whitespace/braces,-legal/copyright
)
roslint_cpp()

roslint_add_test()
