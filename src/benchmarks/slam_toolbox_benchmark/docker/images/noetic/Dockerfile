# Copyright 2022 Ekumen, Inc.
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

FROM lambkin:noetic

WORKDIR ${BENCHMARK_WORKSPACE}

ENV BENCHMARK_PACKAGE=slam_toolbox_benchmark

COPY . ${BENCHMARK_WORKSPACE}/src/${BENCHMARK_PACKAGE}

RUN apt-get update && rosdep update \
  && rosdep install -i -y --from-path src \
  && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin_make_isolated --install

WORKDIR ${BENCHMARK_WORKING_DIRECTORY}
