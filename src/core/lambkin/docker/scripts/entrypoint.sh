#!/bin/bash
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

usage()
{
  echo "$BENCHMARK_PACKAGE-in-docker [-h|--help] [-r|--rebuild] EXECUTABLE ARGS"
}

OPTS=$(getopt -o 'r,h' -l 'rebuild,help' -n '$BENCHMARK_PACKAGE-in-docker' -- "$@")
if [ $? -ne 0 ]; then
    exit $?
fi
eval set -- "$OPTS"
unset OPTS

while true; do
    case "$1" in
        '-r'|'--rebuild')
            source /opt/ros/$ROS_DISTRO/setup.bash
            catkin_make_isolated --install -C $BENCHMARK_WORKSPACE
            shift
            continue
            ;;
        '-h'|'--help')
            usage
            exit 0
            ;;
        '--')
            shift
            break
            ;;
        *)
            echo 'Internal error!' >&2
            exit 1
            ;;
    esac
done

source $BENCHMARK_WORKSPACE/install_isolated/setup.bash ignored

if [ -x "$1" ] || [ "$(command -v $1)" ]; then
    exec "$@"
fi

rosrun $BENCHMARK_PACKAGE $@
