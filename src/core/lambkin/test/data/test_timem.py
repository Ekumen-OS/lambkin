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

import functools
import json
import pytest

from lambkin.data import access
from lambkin.data.timem import history
from lambkin.data.timem import summary
from lambkin.data.units import ureg

from utilities import make_directory_tree


@pytest.fixture
def testing_data_path(tmp_path_factory):
    path = tmp_path_factory.mktemp('data')

    def make_timem_data(path, iteration):
        path.mkdir()

        with open(path / 'some-process.timem.json', 'w') as f:
            json.dump({
                'timemory': {
                    'command_line': ['timem', 'some-process'],
                    'config': {'verbose': 0, 'shell': '/bin/sh'},
                    'timem': [{
                        'cpu_util': {
                            'value': 12.5,
                            'unit_value': 1,
                            'unit_repr': '%'
                        },
                        'peak_rss': {
                            'value': 35.8,
                            'unit_value': 1000000,
                            'unit_repr': 'MB'
                        },
                        'written_bytes': {
                            'value': {
                                'value0': 0.1,
                                'value1': 1e-3
                            },
                            'unit_value': {
                                'value0': 1000000,
                                'value1': 1000000
                            },
                            'unit_repr': {
                                'value0': 'MB',
                                'value1': 'MB/sec'
                            }
                        },
                        'history': [{
                            'sample_timestamp': {
                                'time_since_epoch': 10 * iteration * 1e9
                            },
                            'cpu_util': {
                                'value': 2.,
                                'unit_value': 1,
                                'unit_repr': '%'
                            },
                            'peak_rss': {
                                'value': 10.,
                                'unit_value': 1000000,
                                'unit_repr': 'MB'
                            },
                            'written_bytes': {
                                'value': {
                                    'value0': 0,
                                    'value1': 0
                                },
                                'unit_value': {
                                    'value0': 1000000,
                                    'value1': 1000000
                                },
                                'unit_repr': {
                                    'value0': 'MB',
                                    'value1': 'MB/sec'
                                }
                            }
                        }, {
                            'sample_timestamp': {
                                'time_since_epoch': (10 * iteration + 5) * 1e9
                            },
                            'cpu_util': {
                                'value': 20.,
                                'unit_value': 1,
                                'unit_repr': '%'
                            },
                            'peak_rss': {
                                'value': 28.6,
                                'unit_value': 1000000,
                                'unit_repr': 'MB'
                            },
                            'written_bytes': {
                                'value': {
                                    'value0': 0.05,
                                    'value1': 1e-3
                                },
                                'unit_value': {
                                    'value0': 1000000,
                                    'value1': 1000000
                                },
                                'unit_repr': {
                                    'value0': 'MB',
                                    'value1': 'MB/sec'
                                }
                            }
                        }]
                    }]
                }
            }, f)

    return make_directory_tree({
        'cases': {
            'foo': {
                'metadata.json': json.dumps({'name': 'Foo'}),
                'variations': [{
                    'metadata.json': json.dumps({
                        'parameters': {'scenario': 'nominal'}
                    }),
                    'iterations': [functools.partial(
                        make_timem_data, iteration=i
                    ) for i in range(3)]
                }]
            }
        }
    }, path)


@pytest.fixture(autouse=True)
def testing_context(testing_data_path):
    with access.benchmark(testing_data_path):
        yield


def test_wide_normalized_timem_summary():
    df = summary('some-process', normalization='wide')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',

        'some-process.summary.cpu_util',
        'some-process.summary.peak_rss',
        'some-process.summary.written_bytes.value0',
        'some-process.summary.written_bytes.value1',
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()
    assert df['some-process.summary.cpu_util'].eq(
        (12.5 * ureg.percent).to_base_units().magnitude).all()
    assert df['some-process.summary.peak_rss'].eq(
        (35.8 * ureg.MB).to_base_units().magnitude).all()
    assert df['some-process.summary.written_bytes.value0'].eq(
        (0.1 * ureg.MB).to_base_units().magnitude).all()
    assert df['some-process.summary.written_bytes.value1'].eq(
        (1 * ureg.kB / ureg.second).to_base_units().magnitude).all()


def test_long_normalized_timem_summary():
    df = summary('some-process', normalization='long')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',

        'process.name',
        'process.summary.cpu_util',
        'process.summary.peak_rss',
        'process.summary.written_bytes.value0',
        'process.summary.written_bytes.value1'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()
    assert df['process.name'].eq('some-process').all()
    assert df['process.summary.cpu_util'].eq(
        (12.5 * ureg.percent).to_base_units().magnitude
    ).all()
    assert df['process.summary.peak_rss'].eq(
        (35.8 * ureg.MB).to_base_units().magnitude
    ).all()
    assert df['process.summary.written_bytes.value0'].eq(
        (0.1 * ureg.MB).to_base_units().magnitude
    ).all()
    assert df['process.summary.written_bytes.value1'].eq(
        (1 * ureg.kB / ureg.second).to_base_units().magnitude
    ).all()


def test_denormalized_timem_summary():
    for metadata, values in summary('some-process', normalization=None):
        assert metadata['case']['name'] == 'Foo'
        parameters = metadata['variation']['parameters']
        assert parameters['scenario'] == 'nominal'
        assert values['cpu_util'] == (
            12.5 * ureg.percent
        ).to_base_units().magnitude
        assert values['peak_rss'] == (
            35.8 * ureg.MB
        ).to_base_units().magnitude
        assert values['written_bytes.value0'] == (
            0.1 * ureg.MB
        ).to_base_units().magnitude
        assert values['written_bytes.value1'] == (
            1 * ureg.kB / ureg.second
        ).to_base_units().magnitude


def test_wide_normalized_timem_history():
    df = history('some-process', normalization='wide')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',

        'some-process.series.time',
        'some-process.series.time_since_epoch',
        'some-process.series.cpu_util',
        'some-process.series.peak_rss',
        'some-process.series.written_bytes.value0',
        'some-process.series.written_bytes.value1'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()

    df = df.sort_values(by=[
        'variation.index',
        'iteration.index',
        'some-process.series.time_since_epoch'
    ])
    periods = df['some-process.series.time_since_epoch'].diff()[1:]
    assert periods.eq((5 * ureg.second).to_base_units().magnitude).all()
    assert df['some-process.series.cpu_util'].le(
        (20 * ureg.percent).to_base_units().magnitude
    ).all()
    assert df['some-process.series.peak_rss'].le(
        (28.6 * ureg.MB).to_base_units().magnitude
    ).all()
    assert df['some-process.series.written_bytes.value0'].le(
        (0.05 * ureg.MB).to_base_units().magnitude
    ).all()
    assert df['some-process.series.written_bytes.value1'].le(
        (1 * ureg.kB / ureg.second).to_base_units().magnitude
    ).all()


def test_long_normalized_timem_history():
    df = history('some-process', normalization='long')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',

        'process.name',
        'process.series.time',
        'process.series.time_since_epoch',
        'process.series.cpu_util',
        'process.series.peak_rss',
        'process.series.written_bytes.value0',
        'process.series.written_bytes.value1'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()

    df = df.sort_values(by=[
        'variation.index',
        'iteration.index',
        'process.series.time_since_epoch'
    ])
    assert df['process.name'].eq('some-process').all()
    periods = df['process.series.time_since_epoch'].diff()[1:]
    print(df['process.series.time_since_epoch'])
    assert periods.eq((5 * ureg.second).to_base_units().magnitude).all()
    assert df['process.series.cpu_util'].le(
        (20 * ureg.percent).to_base_units().magnitude
    ).all()
    assert df['process.series.peak_rss'].le(
        (28.6 * ureg.MB).to_base_units().magnitude
    ).all()
    assert df['process.series.written_bytes.value0'].le(
        (0.05 * ureg.MB).to_base_units().magnitude
    ).all()
    assert df['process.series.written_bytes.value1'].le(
        (1 * ureg.kB / ureg.second).to_base_units().magnitude
    ).all()


def test_denormalized_timem_history():
    for metadata, values in history(
        'some-process', normalization=None
    ):
        assert metadata['case']['name'] == 'Foo'
        parameters = metadata['variation']['parameters']
        assert parameters['scenario'] == 'nominal'

        assert values['cpu_util'] <= (
            (20 * ureg.percent).to_base_units().magnitude
        )
        assert values['peak_rss'] <= (
            (28.6 * ureg.MB).to_base_units().magnitude
        )
        assert values['written_bytes.value0'] <= (
            (0.05 * ureg.MB).to_base_units().magnitude
        )
        assert values['written_bytes.value1'] <= (
            (1 * ureg.kB / ureg.second).to_base_units().magnitude
        )
