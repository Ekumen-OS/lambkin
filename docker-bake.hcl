# Copyright 2023 Ekumen, Inc.
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

group "default" {
  targets = ["lambkin-ubuntu-focal-dev"]
}

target "lambkin-ubuntu-focal-dev" {
  contexts = {
    baseimage = "docker-image://osrf/ros:noetic-desktop-full-focal"
  }
  args = { configuration = "{'exclude_components': ['lambkin-shepherd']}" }
  tags = ["ekumenlabs/lambkin:ubuntu-focal-dev"]
}

target "lambkin-ubuntu-focal" {
  contexts = {
    baseimage = "docker-image://osrf/ros:noetic-desktop-full-focal"
  }
  tags = ["ekumenlabs/lambkin:ubuntu-focal"]
}

target "lambkin-ubuntu-jammy-dev" {
  contexts = {
    baseimage = "docker-image://osrf/ros:humble-desktop-full-jammy"
  }
  args = { configuration = "{'include_components': ['ros-humble', 'latex', 'timemory']}" }
  tags = ["ekumenlabs/lambkin:ubuntu-jammy-dev"]
}

target "lambkin-ubuntu-jammy" {
  contexts = {
    baseimage = "docker-image://osrf/ros:humble-desktop-full-jammy"
  }
  args = { configuration = "{'include_components': ['ros-humble', 'lambkin-shepherd', 'latex', 'timemory']}" }
  tags = ["ekumenlabs/lambkin:ubuntu-jammy"]
}

target "slam-toolbox-benchmarks" {
  context = "src/benchmarks/slam_toolbox_benchmarks"
  contexts = {
    lambkin = "target:lambkin-ubuntu-focal"
  }
  args = { baseimage = "lambkin" }
  target = "release"

  tags = ["ekumenlabs/slam-toolbox-benchmarks:latest"]
}

target "cartographer-ros-benchmarks" {
  context = "src/benchmarks/cartographer_ros_benchmarks"
  contexts = {
    lambkin = "target:lambkin-ubuntu-focal"
  }
  args = { baseimage = "lambkin" }
  target = "release"

  tags = ["ekumenlabs/cartographer-ros-benchmarks:latest"]
}

target "multi-solution-benchmarks" {
  context = "src/benchmarks/multi_solution_benchmarks"
  contexts = {
    lambkin-ubuntu-jammy = "target:lambkin-ubuntu-jammy"
  }
  args = { baseimage = "lambkin-ubuntu-jammy" }
  target = "release"
  tags = ["ekumenlabs/multi-solution-benchmarks:latest"]
}
