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

import importlib.util
import pathlib

from collections.abc import Mapping
from collections.abc import Sequence
from collections.abc import Set


def make_directory_tree(tree: Mapping, root: pathlib.Path):
    root.mkdir(parents=True, exist_ok=True)
    for name, subtree in tree.items():
        path = root / name
        if isinstance(subtree, Set):
            subtree = {item: '' for item in subtree if item is not None}
        elif isinstance(subtree, Sequence) and not isinstance(subtree, str):
            subtree = {str(i): item for i, item in enumerate(subtree)}
        if isinstance(subtree, Mapping):
            make_directory_tree(subtree, path)
        elif callable(subtree):
            subtree(path)
        else:
            path.write_text(str(subtree))
    return root


def module_missing(name: str):
    return importlib.util.find_spec(name) is None
