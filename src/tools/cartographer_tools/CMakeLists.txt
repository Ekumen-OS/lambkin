cmake_minimum_required(VERSION 3.5)
project(cartographer_tools)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(cartographer REQUIRED)
find_package(gflags REQUIRED)
# NOTE(hidmic): cartographer depends on Boost targets
# but does not bring them as package dependencies...
find_package(Boost REQUIRED COMPONENTS iostreams)

add_executable(pbstream_to_tum_trajectory src/pbstream_to_tum_trajectory_main.cpp)
target_link_libraries(pbstream_to_tum_trajectory cartographer gflags)

install(TARGETS
  pbstream_to_tum_trajectory
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
