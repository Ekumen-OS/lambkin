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

import json
import pathlib
import pytest

from lambkin.data.access import benchmark
from lambkin.data.access import current_path
from lambkin.data.access import metadata

from lambkin.data.access import cases
from lambkin.data.access import variations
from lambkin.data.access import iterations
from lambkin.data.access import Location

from lambkin.utilities import environment

from utilities import make_directory_tree


def test_metadata(tmp_path):
    assert metadata(tmp_path) == {}

    assert metadata(tmp_path, i=1) == {'i': 1}

    m = {'a': 1, 'b': 2}
    metadata_path = tmp_path / 'metadata.json'
    metadata_path.write_text(json.dumps(m))
    assert metadata(tmp_path) == m

    assert metadata(tmp_path, c=3) == {**m, 'c': 3}


def test_benchmark_path(tmp_path):
    foo_path = tmp_path / 'foo'
    bar_path = tmp_path / 'bar'
    baz_path = tmp_path / 'baz'

    foo_path.mkdir()
    bar_path.mkdir()
    baz_path.mkdir()

    with environment(BENCHMARK_ROOT=foo_path):
        with benchmark(bar_path):
            with benchmark(baz_path):
                assert current_path() == baz_path
            assert current_path() == bar_path
        assert current_path() == foo_path
    assert current_path() == pathlib.Path.cwd()


@pytest.fixture
def testing_data_path(tmp_path_factory):
    path = tmp_path_factory.mktemp('data')
    return make_directory_tree({
        'cases': {
            'bar': {
                'metadata.json': json.dumps({'name': 'Bar'}),
                'variations': [{
                    'metadata.json': json.dumps({
                        'parameters': {'p': 'a'}
                    }),
                    'iterations': [{None}] * 2
                }]
            },
            'foo': {
                'metadata.json': json.dumps({'name': 'Foo'}),
                'variations': [{
                    'metadata.json': json.dumps({
                        'parameters': {'p': 'b'}
                    }),
                    'iterations': [{None}] * 2
                }]
            }
        }
    }, path)


@pytest.fixture()
def testing_benchmark(testing_data_path):
    with benchmark(testing_data_path):
        yield


def test_cases_access(testing_benchmark):
    assert sorted(cases(), key=lambda loc: loc.path) == [
        Location(
            current_path() / 'cases' / 'bar', {'case': {
                'name': 'Bar', 'root': current_path() / 'cases' / 'bar'
            }}
        ),
        Location(
            current_path() / 'cases' / 'foo', {'case': {
                'name': 'Foo', 'root': current_path() / 'cases' / 'foo'
            }}
        )
    ]


def test_variations_access(testing_benchmark):
    bar_path = current_path() / 'cases' / 'bar'
    foo_path = current_path() / 'cases' / 'foo'

    assert sorted(variations(), key=lambda loc: loc.path) == [
        Location(bar_path / 'variations' / '0', {
            'case': {'name': 'Bar', 'root': bar_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'a'}
            }
        }),
        Location(foo_path / 'variations' / '0', {
            'case': {'name': 'Foo', 'root': foo_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'b'}
            }
        })
    ]


def test_iterations_access(testing_benchmark):
    foo_path = current_path() / 'cases' / 'foo'
    foo_vpath = foo_path / 'variations' / '0'
    bar_path = current_path() / 'cases' / 'bar'
    bar_vpath = bar_path / 'variations' / '0'

    assert sorted(iterations(), key=lambda loc: loc.path) == [
        Location(bar_vpath / 'iterations' / '0', {
            'case': {'name': 'Bar', 'root': bar_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'a'}
            }, 'iteration': {'index': 0}
        }),
        Location(bar_vpath / 'iterations' / '1', {
            'case': {'name': 'Bar', 'root': bar_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'a'}
            }, 'iteration': {'index': 1}
        }),
        Location(foo_vpath / 'iterations' / '0', {
            'case': {'name': 'Foo', 'root': foo_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'b'}
            }, 'iteration': {'index': 0}
        }),
        Location(foo_vpath / 'iterations' / '1', {
            'case': {'name': 'Foo', 'root': foo_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'b'}
            }, 'iteration': {'index': 1}
        })
    ]


def test_filtered_access(testing_benchmark):
    foo_path = current_path() / 'cases' / 'foo'
    foo_vpath = foo_path / 'variations' / '0'

    assert sorted(iterations(
        v for v in variations(cases(current_path())) if
        v.metadata['variation']['parameters']['p'] == 'b'
    ), key=lambda loc: loc.path) == [
        Location(foo_vpath / 'iterations' / '0', {
            'case': {'name': 'Foo', 'root': foo_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'b'}
            }, 'iteration': {'index': 0}
        }),
        Location(foo_vpath / 'iterations' / '1', {
            'case': {'name': 'Foo', 'root': foo_path},
            'variation': {
                'index': 0, 'parameters': {'p': 'b'}
            }, 'iteration': {'index': 1}
        }),
    ]
