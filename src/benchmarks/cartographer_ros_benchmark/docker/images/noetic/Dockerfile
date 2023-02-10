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

ADD https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
    cartographer_ros.rosinstall
RUN . /opt/ros/$ROS_DISTRO/setup.sh && wstool init src cartographer_ros.rosinstall

RUN apt-get update && apt-get install -y ninja-build stow && \
    src/cartographer/scripts/install_abseil.sh && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y libgmock-dev && \
    rosdep update && rosdep install -y -i --from-paths src --skip-keys libabsl-dev && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin_make_isolated --install

ENV BENCHMARK_PACKAGE=cartographer_ros_benchmark

COPY . ${BENCHMARK_WORKSPACE}/src/${BENCHMARK_PACKAGE}

RUN apt-get update && rosdep update && \
    rosdep install -y -i --from-paths src --skip-keys libabsl-dev && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin_make_isolated --install

WORKDIR ${BENCHMARK_WORKING_DIRECTORY}
