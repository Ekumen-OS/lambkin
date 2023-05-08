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

"""This module supplements RobotFramework ``evo`` library resource."""

from robot.api.deco import keyword

from lambkin.shepherd.data.evo import _to_evo_filestem


@keyword('Convert To EVO Filestem')
def convert_to_evo_filestem(name: str, name_format: str) -> str:
    """
    Convert `name` to a filestem like ``evo`` does.

    `name_format` may be either 'ros' to deal with ROS topic name
    semantics or 'path' to deal with filesystem semantics.
    """
    return _to_evo_filestem(name, name_format)
