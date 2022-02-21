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

FROM ubuntu:20.04

# Setup OS environment
ENV ROS_DISTRO noetic

RUN apt-get update \
  && apt-get install --no-install-recommends -y locales \
  && locale-gen en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm

# Install common packages
RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    cm-super \
    dvipng \
    git \
    gnupg2 \
    lsb-release \
    mesa-utils \
    python3 \
    python3-dev \
    python3-pip \
    python3-tk \
    rst2pdf \
    sudo \
    texlive-latex-base \
    texlive-latex-extra \
    texlive-latex-recommended \
    texlive-fonts-recommended \
    wget \
  && rm -rf /var/lib/apt/lists/*

# Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add -

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-ros-base \
    python3-rosdep \
    python3-rosinstall \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# Install third party libraries and tools
WORKDIR /opt

RUN git clone -b develop https://github.com/NERSC/timemory.git timemory \
  && mkdir -p timemory/build  \
  && cd timemory/build \
  && git checkout 431912b360ff50d1a160d7826e2eea04fbd1037f \
  && cmake -DCMAKE_BUILD_TYPE=Release \
    -DTIMEMORY_INSTALL_HEADERS=OFF \
    -DTIMEMORY_INSTALL_CONFIG=OFF \
    -DTIMEMORY_INSTALL_ALL=OFF \
    -DTIMEMORY_BUILD_TIMEM=ON .. \
  && make -j1 \
  && make install \
  && ldconfig

RUN git clone -b v1.16.0 https://github.com/MichaelGrupp/evo.git evo \
  && pip install --editable evo --upgrade --no-binary evo

RUN pip install \
  numpy==1.18.5 \
  python-dateutil==2.8.1 \
  robotframework==4.1.3 \
  svglib==1.1.0

# Setup LAMBKIN environment
ENV HOME=/root
ENV BENCHMARK_WORKSPACE=${HOME}/ws
ENV BENCHMARK_WORKING_DIRECTORY=${HOME}/work
RUN mkdir -p $BENCHMARK_WORKSPACE $BENCHMARK_WORKING_DIRECTORY

COPY . ${BENCHMARK_WORKSPACE}/src/lambkin
RUN apt-get update \
  && rosdep install --from-paths ${BENCHMARK_WORKSPACE}/src -y \
  && rm -rf /var/lib/apt/lists/*

COPY docker/scripts/entrypoint.sh /boot/entrypoint.sh
ENTRYPOINT ["/boot/entrypoint.sh"]

WORKDIR ${BENCHMARK_WORKING_DIRECTORY}
