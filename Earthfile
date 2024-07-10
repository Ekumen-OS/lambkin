# Copyright 2024 Ekumen, Inc.
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

VERSION 0.8
IMPORT ./src/external/os AS os

ARG --global components="core/lambkin-shepherd core/lambkin-clerk external/ros2 external/typesetting/latex external/profiling/timemory"

embed-ubuntu-devel:
    ARG distro=jammy
    FROM os+ubuntu --distro=${distro}
    FOR component IN ${components}
        MERGE os+ubuntu.../src/${component}+embed-ubuntu-devel --distro=${distro}
    END
    RUN echo "echo 'Logged into development environment for ${components} on Ubuntu: ${distro}'" > /etc/profile.d/banner.sh

embed-ubuntu-release:
    ARG distro=jammy
    FROM os+ubuntu --distro=${distro}
    FOR component IN ${components}
        MERGE os+ubuntu.../src/${component}+embed-ubuntu-release --distro=${distro}
    END

ubuntu-devel:
    ARG distro=jammy
    ARG user=lambkin
    ARG uid=1000
    ARG gid=1000
    FROM +embed-ubuntu-devel --distro=${distro}
    DO os+ADDUSER --user=${user} --uid=${uid} --gid=${gid}
    SAVE IMAGE ekumenlabs/lambkin:ubuntu-${distro}-dev ekumenlabs/lambkin:dev

local-ubuntu-devel:
    LOCALLY
    ARG distro=jammy
    LET user = "$(whoami)"
    LET uid = "$(id -u)"
    LET gid = "$(id -g)"
    BUILD +ubuntu-devel --distro=${distro} --user=${user} --uid=${uid} --gid=${gid}

ubuntu-release:
    ARG distro=jammy
    FROM +embed-ubuntu-release --distro=${distro}
    SAVE IMAGE ekumenlabs/lambkin:ubuntu-${distro}
