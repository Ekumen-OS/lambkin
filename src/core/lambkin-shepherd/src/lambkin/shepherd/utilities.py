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

"""This module provides common utility functions."""

import contextlib
import itertools
import re
import os
import yaml

from collections.abc import Mapping
from typing import Any, Iterable, Optional, Tuple


class deepdict(dict):
    """
    A `dict` with support for indexing arbitrarily nested values.

    Use `d[a, b, ...]` or `d[k]` with `k` a list for nested indexing.
    When setting values, intermediate mappings will be created to satisfy
    the request.
    """

    def __contains__(self, key: Any) -> bool:
        """Check if an item `key` exists."""
        if isinstance(key, (tuple, list)):
            if len(key) == 0:
                raise KeyError('no keys')
            if len(key) > 1:
                return super().__contains__(key[0]) and (
                    key[1:] in super().__getitem__(key[0]))
            key = key[0]
        return super().__contains__(key)

    def __getitem__(self, key: Any) -> Any:
        """Get an item at `key`."""
        if isinstance(key, (tuple, list)):
            if len(key) == 0:
                raise KeyError('no keys')
            if len(key) > 1:
                nested = super().__getitem__(key[0])
                return nested[key[1:]]
            key = key[0]
        return super().__getitem__(key)

    def __setitem__(self, key: Any, value: Any) -> None:
        """Set an item at `key`."""
        if isinstance(key, (tuple, list)):
            if len(key) == 0:
                raise KeyError('no keys')
            if len(key) > 1:
                if not super().__contains__(key[0]):
                    super().__setitem__(key[0], type(self)())
                nested = super().__getitem__(key[0])
                nested[key[1:]] = value
                return
            key = key[0]
        super().__setitem__(key, value)


yaml.add_representer(deepdict, lambda dumper, data: dumper.represent_dict(data))


def safe_merge(base: Mapping, head: Mapping) -> Mapping:
    """
    Perform a shallow merge of `head` into `base`.

    The operation is safe as it ensures no key is overwritten.

    :param base: target mapping for merge operation.
    :param head: source mapping for merge operation.
    :return: merged mapping.
    :raises ValueError: whenever a key would be overwritten
    """
    for key in head:
        if key in base:
            raise ValueError(f'{key} would be overwritten by merge')
    return {**base, **head}


def snake_case(string: str) -> str:
    """Transform the given `string` to be lower case alphanumeric and underscore characters only."""
    string = string.replace('-', '_').replace(' ', '_')
    string = re.sub(r'([^_0-9])([A-Z][a-z]+)', r'\1_\2', string)
    string = re.sub(r'([a-z])([A-Z])', r'\1_\2', string)
    return string.lower()


def peek_iterable(it: Iterable[Any]) -> Tuple[Any, Iterable[Any]]:
    """
    Peek first item in an iterable.

    :returns: a tuple of first item and iterable, where the first
      item may be a StopIteration exception object if the iterable
      is empty.
    """
    it = iter(it)
    try:
        item = next(it)
    except StopIteration:
        return StopIteration, it
    return item, itertools.chain([item], it)


def enforce_nonempty(
    it: Iterable[Any], msg: str = 'iterable is empty'
) -> Iterable[Any]:
    """
    Enforce iterable is not empty.

    :returns: iterable
    """
    item, it = peek_iterable(it)
    if item is StopIteration:
        raise ValueError(msg)
    return it


@contextlib.contextmanager
def environment(**envvars):
    """Binds environment variables to execution scope."""
    environ = dict(os.environ)
    try:
        os.environ.update({
            name: str(value) for name, value in envvars.items()
        })
        yield
    finally:
        os.environ = environ


def fqn(obj: Any) -> Optional[str]:
    """Compute the fully qualified name of a given object, if any."""
    if not hasattr(obj, '__qualname__'):
        return None
    name = obj.__qualname__
    if not hasattr(obj, '__module__'):
        return name
    module = obj.__module__
    return f'{module}.{name}'
