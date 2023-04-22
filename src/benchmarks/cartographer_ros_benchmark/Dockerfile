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

ARG baseimage=ekumenlabs/lambkin:ubuntu-focal
FROM ${baseimage} AS development

# Switch to bash as shell
SHELL ["/bin/bash", "--login", "-c"]

RUN mkdir -p /tmp/ws
WORKDIR /tmp/ws

ADD --chown=$USERNAME:$USERNAME \
    https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
    cartographer_ros.rosinstall
RUN wstool init src cartographer_ros.rosinstall

RUN sudo apt-get update && sudo apt-get install -y libgmock-dev ninja-build stow && \
    sudo src/cartographer/scripts/install_abseil.sh && rosdep update && \
    rosdep install -y -i --from-paths src --skip-keys libabsl-dev && \
    sudo rm -rf /var/lib/apt/lists/*

USER root
RUN catkin_make_isolated --install-space /opt/ros/$ROS_DISTRO-extras --install --merge

RUN cat <<EOF >> /etc/profile.d/ros.sh
. /opt/ros/$ROS_DISTRO-extras/setup.sh
EOF

RUN rm -rf /tmp/ws

USER $USERNAME
WORKDIR $HOME

RUN --mount=type=bind,source=.,target=/tmp/ws/src/package,readonly cd /tmp/ws && \
    sudo apt update && rosdep update && rosdep install -i -y --from-path src --skip-keys 'lambkin' && \
    sudo rm -rf /var/lib/apt/lists/*

FROM development AS release

USER root
RUN --mount=type=bind,source=.,target=/tmp/ws/src/package,readonly cd /tmp/ws && \
    catkin_make_isolated --install --merge --install-space /opt/ros/application

RUN cat <<EOF > /opt/ros/application/entrypoint.bash
# Auto-generated with Docker Buildkit
source /opt/ros/application/setup.bash
if [ \$# -ne 0 ]; then
    rosrun cartographer_ros_benchmark \$@
else
    bash
fi
EOF

USER $USERNAME
WORKDIR $HOME

ENTRYPOINT ["fixuid", "-q", "/bin/bash", "--login", "/opt/ros/application/entrypoint.bash"]
