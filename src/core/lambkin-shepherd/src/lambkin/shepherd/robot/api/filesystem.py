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

"""This module supplements RobotFramework filesystem library resource."""

import json
import os
import shutil
from typing import Any, Optional

from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn


@keyword('Find Executable Path')
def find_executable_path(name: str) -> Optional[str]:
    """
    Find executable in $PATH.

    :param name: executable basename.
    :return: path to executable if found.
    """
    return shutil.which(name)


@keyword('Resolve File Path')
def resolve_file_path(path: str) -> Optional[str]:
    """
    Resolve path to an existing file, if any.

    Relative paths are resolved w.r.t. the current working directory first,
    and w.r.t. the parent directory of the benchmark suite if the first
    does not resolve to an existing file.

    :returns: an absolute path, or None if `path` does not point to a file.
    """
    if os.path.exists(path):
        return path
    if os.path.isabs(path):
        return None
    ctx = BuiltIn()._get_context()
    dirpath = os.path.dirname(ctx.suite.source)
    path = os.path.join(dirpath, path)
    if not os.path.exists(path):
        return None
    return path


@keyword('Resolve File Path or Fail')
def resolve_file_path_or_fail(input_: str) -> Optional[str]:
    """
    Resolve path to an existing file, if any, failing otherwise.

    See py:func:`resolve_file_path` for further reference.
    """
    output = resolve_file_path(input_)
    if output is None:
        BuiltIn().fail(f'no {input_} file was found')
    return output


@keyword('Write JSON File')
def write_json_file(path: str, indent: Optional[int] = 2, **variables: Any):
    """
    Write a JSON file using keyword arguments as key-value pairs.

    Objects that are JSON serializable will be stringified to their
    representation (i.e. ``repr(obj)``).
    """
    with open(path, 'w') as f:
        json.dump(variables, f, indent=indent, default=repr)
