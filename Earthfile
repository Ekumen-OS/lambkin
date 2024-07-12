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
    ARG distro
    FROM --pass-args os+ubuntu
    FOR component IN ${components}
        MERGE --pass-args os+ubuntu.../src/${component}+embed-ubuntu-devel
    END
    RUN echo "echo 'Logged into development environment for ${components} on Ubuntu: ${distro}'" > /etc/profile.d/banner.sh

embed-ubuntu-release:
    ARG distro
    FROM --pass-args os+ubuntu
    FOR component IN ${components}
        MERGE --pass-args os+ubuntu.../src/${component}+embed-ubuntu-release
    END

ubuntu-devel:
    ARG distro=jammy
    ARG user=lambkin
    ARG uid=1000
    ARG gid=1000
    FROM --pass-args +embed-ubuntu-devel
    DO os+ADDUSER --user=${user} --uid=${uid} --gid=${gid}
    ARG tag=ubuntu-${distro}-devel
    SAVE IMAGE ekumenlabs/lambkin:${tag} ekumenlabs/lambkin:devel

local-ubuntu-devel:
    LOCALLY
    ARG distro=jammy
    # NOTE(hidmic): do a two-step expansion to
    # avoid duplicate dependency builds (a bug
    # upstream? a bug in our patch?)
    LET local_user="$(whoami)"
    LET local_uid="$(id -u)"
    LET local_gid="$(id -g)"
    LET user="${local_user}"
    LET uid="${local_uid}"
    LET gid="${local_gid}"
    BUILD --pass-args +ubuntu-devel

ubuntu-release:
    ARG distro=jammy
    ARG tag=ubuntu-${distro}
    FROM --pass-args +embed-ubuntu-release
    SAVE IMAGE ekumenlabs/lambkin:${tag}
