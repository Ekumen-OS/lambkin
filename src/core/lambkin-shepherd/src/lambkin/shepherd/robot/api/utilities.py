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

"""This module supplements RobotFramework utilites library resource."""

from robot.api.deco import keyword
from robot.utils.robottypes import TRUE_STRINGS, FALSE_STRINGS

from typing import List


@keyword('Convert To Snake Case')
def convert_to_snake_case(value: str) -> str:
    """
    Convert string to snake case.

    See :py:func:`snake_case` documentation.
    """
    from lambkin.shepherd.utilities import snake_case
    return snake_case(value)


@keyword('Convert To Command Line Options')
def convert_to_command_line_options(**kwargs) -> List[str]:
    """
    Convert keyword arguments to a command line options list.

    All option names are prepended with a double dash (--).
    Boolean options are treated as flags i.e. added with
    with no argument when true. List options have their
    arguments added one after the other.
    """
    options: List[str] = []
    for name, value in kwargs.items():
        if value is None:
            continue
        if isinstance(value, str):
            if value.upper() in TRUE_STRINGS:
                value = True
            elif value.upper() in FALSE_STRINGS:
                value = False
        if isinstance(value, bool):
            if value:
                options.append('--' + name)
            continue
        options.append('--' + name)
        if isinstance(value, list):
            options.extend(map(str, value))
        else:
            options.append(str(value))
    return options
