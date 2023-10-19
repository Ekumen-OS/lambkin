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

import pytest

from lambkin.shepherd.utilities import deepdict
from lambkin.shepherd.utilities import enforce_nonempty
from lambkin.shepherd.utilities import peek_iterable
from lambkin.shepherd.utilities import safe_merge
from lambkin.shepherd.utilities import snake_case


def test_deepdict():
    d = deepdict()
    d['a'] = None
    d['b', 'x'] = 1
    d['b', 'y'] = 2
    assert 'a' in d
    assert 'b' in d
    assert 'c' not in d
    assert ['b', 'x'] in d
    assert ['b', 'z'] not in d
    assert d['b'] == {'x': 1, 'y': 2}
    assert d == {'a': None, 'b': {'x': 1, 'y': 2}}


def test_enforce_empty():
    with pytest.raises(ValueError):
        enforce_nonempty([])
    enforce_nonempty([None])


def test_peek_iterable():
    item, iterable = peek_iterable([])
    assert item is StopIteration
    assert list(iterable) == []

    item, iterable = peek_iterable([1])
    assert item == 1
    assert list(iterable) == [1]

    item, iterable = peek_iterable([1, 2])
    assert item == 1
    assert list(iterable) == [1, 2]


def test_snake_case():
    assert snake_case('SomeExample') == 'some_example'
    assert snake_case('Some Example') == 'some_example'
    assert snake_case('some-example') == 'some_example'


def test_safe_merge():
    assert safe_merge({'a': 1}, {'b': 2}) == {'a': 1, 'b': 2}

    with pytest.raises(ValueError):
        safe_merge({'a': 1}, {'a': 2})
