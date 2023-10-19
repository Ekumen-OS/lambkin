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

import math
import json
import random

import numpy as np
import numpy.testing

from importlib import import_module

from lambkin.shepherd.data import access
from lambkin.shepherd.data.evo import series
from lambkin.shepherd.data.evo import stats
from lambkin.shepherd.data.evo import trajectory

from utilities import make_directory_tree
from utilities import module_missing


@pytest.fixture
def testing_data_path(tmp_path_factory):
    path = tmp_path_factory.mktemp('data')

    def make_evo_data(path):
        np = import_module('numpy')
        file_interface = import_module(
            'evo.tools.file_interface')
        result = import_module('evo.core.result')
        trajectory = import_module('evo.core.trajectory')

        time = np.arange(0, 10, 0.1)
        gen = random.Random(hash(path))

        path.mkdir()

        ape = result.Result()
        ape.add_stats({'mean': gen.random(), 'std': gen.random()})
        ape.add_np_array('seconds_from_start', time)
        ape.add_np_array('error_array', np.array([
            gen.uniform(0, 1) for _ in range(len(time))]))
        file_interface.save_res_file(path / 'tf_map.base_link.ape.zip', ape)

        rpe = result.Result()
        rpe.add_stats({'mean': gen.random(), 'std': gen.random()})
        rpe.add_np_array('seconds_from_start', time)
        rpe.add_np_array('error_array', np.array([
            gen.uniform(0, 1) for _ in range(len(time))]))
        file_interface.save_res_file(path / 'tf_map.base_link.rpe.zip', rpe)
        odom_to_base_traj = trajectory.PoseTrajectory3D(
            timestamps=time,
            positions_xyz=np.hstack((
                np.linspace(
                    0, 10, len(time), endpoint=False
                )[:, np.newaxis],
                np.zeros((len(time), 2))
            )),
            orientations_quat_wxyz=np.hstack((
                np.ones((len(time), 1)),
                np.zeros((len(time), 3))
            ))
        )  # straight, x-axis aligned
        file_interface.write_tum_trajectory_file(
            path / 'tf_map.base_link.tum', odom_to_base_traj
        )

    return make_directory_tree({
        'cases': {
            'foo': {
                'metadata.json': json.dumps({'name': 'Foo'}),
                'variations': [{
                    'metadata.json': json.dumps({
                        'parameters': {'scenario': 'nominal'}
                    }),
                    'iterations': [make_evo_data] * 2
                }]
            }
        }
    }, path)


@pytest.fixture(autouse=True)
def testing_context(testing_data_path):
    with access.benchmark(testing_data_path):
        yield


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_wide_normalized_evo_stats():
    df = stats('/tf:map.base_link', 'ape', normalization='wide')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',
        '/tf:map.base_link.ape.mean',
        '/tf:map.base_link.ape.std'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()
    assert df['/tf:map.base_link.ape.mean'].mean() <= 1.
    assert df['/tf:map.base_link.ape.std'].mean() <= 1.


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_long_normalized_evo_stats():
    df = stats('/tf:map.base_link', 'ape', normalization='long')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',
        'trajectory.name',
        'metric.name',
        'metric.mean',
        'metric.std'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()
    assert df['trajectory.name'].eq('/tf:map.base_link').all()
    assert df['metric.name'].eq('ape').all()
    assert df['metric.mean'].mean() <= 1.
    assert df['metric.std'].mean() <= 1.


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_denormalized_evo_stats():
    for metadata, statistics in stats(
        '/tf:map.base_link', 'ape', normalization=None
    ):
        assert metadata['case']['name'] == 'Foo'
        parameters = metadata['variation']['parameters']
        assert parameters['scenario'] == 'nominal'
        assert statistics['mean'] <= 1.
        assert statistics['std'] <= 1.


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_wide_normalized_evo_series():
    df = series('/tf:map.base_link', 'rpe', normalization='wide')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',
        '/tf:map.base_link.rpe.series.time',
        '/tf:map.base_link.rpe.series.value',
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()
    df = df.loc[df['iteration.index'] == 0]
    periods = df['/tf:map.base_link.rpe.series.time'].diff()[1:]
    assert periods.apply(math.isclose, b=0.1).all()
    assert df['/tf:map.base_link.rpe.series.value'].le(1).all()


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_long_normalized_evo_series():
    df = series('/tf:map.base_link', 'rpe', normalization='long')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',
        'trajectory.name',
        'metric.name',
        'metric.series.time',
        'metric.series.value'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()
    assert df['trajectory.name'].eq('/tf:map.base_link').all()
    assert df['metric.name'].eq('rpe').all()
    df = df.loc[df['iteration.index'] == 0]
    periods = df['metric.series.time'].diff()[1:]
    assert periods.apply(math.isclose, b=0.1).all()
    assert df['metric.series.value'].le(1).all()


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_denormalized_evo_series():
    for metadata, time, values in series(
        '/tf:map.base_link', 'rpe', normalization=None
    ):
        assert metadata['case']['name'] == 'Foo'
        parameters = metadata['variation']['parameters']
        assert parameters['scenario'] == 'nominal'
        np.testing.assert_allclose(np.diff(time)[1:], 0.1)
        np.testing.assert_array_less(values, np.ones_like(values))


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_wide_normalized_evo_trajectory():
    df = trajectory('/tf:map.base_link', normalization='wide')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',

        '/tf:map.base_link.time',
        '/tf:map.base_link.time_since_epoch',
        '/tf:map.base_link.x',
        '/tf:map.base_link.y',
        '/tf:map.base_link.z',
        '/tf:map.base_link.qx',
        '/tf:map.base_link.qy',
        '/tf:map.base_link.qz',
        '/tf:map.base_link.qw',
        '/tf:map.base_link.roll',
        '/tf:map.base_link.pitch',
        '/tf:map.base_link.yaw'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()

    df = df.loc[df['iteration.index'] == 0]
    assert df['/tf:map.base_link.y'].eq(0).all()
    assert df['/tf:map.base_link.z'].eq(0).all()
    velocity = df['/tf:map.base_link.x'].diff().divide(
        df['/tf:map.base_link.time'].diff(), axis=0
    )[1:]  # drop first NaN
    assert velocity.apply(math.isclose, b=1).all()
    assert df['/tf:map.base_link.qx'].eq(0).all()
    assert df['/tf:map.base_link.qy'].eq(0).all()
    assert df['/tf:map.base_link.qz'].eq(0).all()
    assert df['/tf:map.base_link.qw'].eq(1).all()


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_long_normalized_evo_trajectory():
    df = trajectory('/tf:map.base_link', normalization='long')
    assert sorted(df.columns) == sorted([
        'case.name',
        'case.root',
        'iteration.index',
        'variation.index',
        'variation.parameters.scenario',

        'trajectory.name',
        'trajectory.time',
        'trajectory.time_since_epoch',
        'trajectory.x',
        'trajectory.y',
        'trajectory.z',
        'trajectory.qx',
        'trajectory.qy',
        'trajectory.qz',
        'trajectory.qw',
        'trajectory.roll',
        'trajectory.pitch',
        'trajectory.yaw'
    ])
    assert df['case.name'].eq('Foo').all()
    scenarios = df['variation.parameters.scenario']
    assert scenarios.eq('nominal').all()
    assert df['trajectory.name'].eq('/tf:map.base_link').all()

    df = df.loc[df['iteration.index'] == 0]
    assert df['trajectory.y'].eq(0).all()
    assert df['trajectory.z'].eq(0).all()
    velocity = df['trajectory.x'].diff().divide(
        df['trajectory.time'].diff(), axis=0
    )[1:]  # drop first NaN
    assert velocity.apply(math.isclose, b=1).all()
    assert df['trajectory.qx'].eq(0).all()
    assert df['trajectory.qy'].eq(0).all()
    assert df['trajectory.qz'].eq(0).all()
    assert df['trajectory.qw'].eq(1).all()


@pytest.mark.skipif(
    module_missing('evo'),
    reason='evo is not installed')
def test_denormalized_evo_trajectory():
    for metadata, traj in trajectory(
        '/tf:map.base_link', normalization=None
    ):
        assert metadata['case']['name'] == 'Foo'
        parameters = metadata['variation']['parameters']
        assert parameters['scenario'] == 'nominal'
        dp_xyz = np.diff(traj.positions_xyz, axis=0)[1:, :]
        dt = np.diff(traj.timestamps)[1:][:, np.newaxis]
        velocity_xyz = dp_xyz / dt
        np.testing.assert_allclose(velocity_xyz[:, 0], 1)
        np.testing.assert_equal(velocity_xyz[:, 1], 0.)
        np.testing.assert_equal(velocity_xyz[:, 2], 0.)
