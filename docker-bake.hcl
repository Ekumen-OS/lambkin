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
  args = { configuration = "{'exclude_components': ['lambkin']}" }
  tags = ["ekumenlabs/lambkin:ubuntu-focal-dev"]
}

target "lambkin-ubuntu-focal" {
  contexts = {
    baseimage = "docker-image://osrf/ros:noetic-desktop-full-focal"
  }
  tags = ["ekumenlabs/lambkin:ubuntu-focal"]
}

target "slam-toolbox-benchmarks" {
  context = "src/benchmarks/slam_toolbox_benchmark"
  contexts = {
    lambkin = "target:lambkin-ubuntu-focal"
  }
  args = { baseimage = "lambkin" }
  target = "release"

  tags = ["ekumenlabs/slam-toolbox-benchmarks:latest"]
}

target "cartographer-ros-benchmarks" {
  context = "src/benchmarks/cartographer_ros_benchmark"
  contexts = {
    lambkin = "target:lambkin-ubuntu-focal"
  }
  args = { baseimage = "lambkin" }
  target = "release"

  tags = ["ekumenlabs/cartographer-ros-benchmarks:latest"]
}
