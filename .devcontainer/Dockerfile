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

FROM ekumenlabs/lambkin:ubuntu-focal-dev

RUN sudo pip3 install colcon-common-extensions poetry colcon-poetry-ros

## NOTE(hidmic): remove colcon-poetry-ros weird script relocation
RUN sudo sed -i '111,131d' `python3 -c "import colcon_poetry_ros.task.poetry.build as m; print(m.__file__)"`

RUN --mount=type=bind,source=.,target=/tmp/repository,readonly cd /tmp/repository && sudo apt update && \
    rosdep update && rosdep install -i -y --from-path src --skip-keys 'lambkin-shepherd cartographer_ros ament_cmake rosbag2_storage_default_plugins nav2_lifecycle_manager beluga_amcl launch_ros nav2_amcl' && \
    sudo rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source=.,target=/tmp/repository,readonly \
    find /tmp/repository -name requirements.txt -exec sudo pip3 install -r {} \;
