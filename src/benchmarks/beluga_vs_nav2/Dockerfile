# syntax=docker/dockerfile:1.4
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

ARG baseimage=ekumenlabs/lambkin:ubuntu-jammy
FROM ${baseimage} AS development



USER root
# Fix for rviz black screen
RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    software-properties-common \
  && add-apt-repository -y ppa:kisak/kisak-mesa \
  && apt-get install --no-install-recommends -y \
    libegl-mesa0 libglx-mesa0 \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspace/ && chown -R $USERNAME:$USERNAME /workspace
RUN mkdir -p /tmp/context && chown -R $USERNAME:$USERNAME /tmp/context
RUN mkdir -p /opt/ros/application && chown -R $USERNAME:$USERNAME /opt/ros/application

USER $USERNAME

# TODO: Pin to release.
RUN cd /workspace && git clone https://github.com/Ekumen-OS/beluga && mkdir -p src && mv beluga/* src/


RUN sudo apt update && sudo apt upgrade -y
RUN mkdir -p /tmp/context

RUN --mount=type=bind,source=.,target=/tmp/context/src,readonly cd /tmp/context && \
    sudo apt update && rosdep update && rosdep install -i -y --from-paths src /workspace/src --skip-keys 'lambkin-shepherd flatland_server beluga_example beluga_benchmarks flatland_plugins' && \
    sudo rm -rf /var/lib/apt/lists/* && colcon build --packages-up-to 'beluga_vs_nav2' --merge-install --install-base /opt/ros/application

FROM development AS release

RUN cat <<EOF > /opt/ros/application/entrypoint.bash
# Auto-generated with Docker Buildkit
source /opt/ros/application/setup.bash
if [ \$# -ne 0 ]; then
    ros2 run beluga_vs_nav2 \$@
else
    bash
fi
EOF


WORKDIR $HOME

ENTRYPOINT ["fixuid", "-q", "/bin/bash", "--login", "/opt/ros/application/entrypoint.bash"]
