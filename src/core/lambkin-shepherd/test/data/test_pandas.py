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

import numpy as np
import pandas as pd

from lambkin.shepherd.data.access import benchmark

from lambkin.shepherd.data.pandas import inner_join
from lambkin.shepherd.data.pandas import rescale
from lambkin.shepherd.data.pandas import cache

from lambkin.shepherd.data.pandas import cache_storage_path

import pytest


def test_inner_join():
    lvalue = pd.DataFrame({
        'sample_index': np.arange(100), 'device': 'thm0',
        'temperature': 25 + np.random.uniform(-2, 2, 100)
    })
    rvalue = pd.DataFrame({
        'sample_index': np.arange(100), 'device': 'bar0',
        'pressure': 101325 + np.random.uniform(-100, 100, 100)
    })
    mvalue = inner_join([lvalue, rvalue], on=['sample_index'])
    mvalue = mvalue.drop(columns=['device', 'device_1'])
    assert sorted(mvalue.columns) == [
        'pressure', 'sample_index', 'temperature']
    indexed_mvalue = mvalue.set_index('sample_index')

    indexed_lvalue = lvalue.set_index('sample_index')
    assert (indexed_mvalue['temperature'] == indexed_lvalue['temperature']).all()

    indexed_rvalue = rvalue.set_index('sample_index')
    assert (indexed_mvalue['pressure'] == indexed_rvalue['pressure']).all()


def test_rescale():
    data = pd.DataFrame({
        'sample_index': np.arange(100),
        'distance': 1000.0 * np.arange(100)})
    scaled_data = rescale(data, {'distance': 0.001})
    assert (scaled_data['distance'] == np.arange(100)).all()


@pytest.fixture
def mock_function(mocker):
    stub = mocker.stub(name='noop')
    stub.__name__ = 'noop'
    stub.__qualname__ = 'noop'
    stub.__annotations__ = None
    return stub


def test_cached_function_returns_cached_dataframe(tmp_path, mock_function):
    mock_function.return_value = pd.DataFrame({'index': [0]})
    cached_function = cache(mock_function)

    with benchmark(tmp_path):
        data = cached_function()
        assert data is mock_function.return_value
        storage_path = cache_storage_path()
        assert storage_path.exists()
        with pd.HDFStore(storage_path) as store:
            assert len(store) == 1
        data = cached_function()
        assert data is not mock_function.return_value
        assert data.equals(mock_function.return_value)


def test_cache_function_handles_unhashable_args(tmp_path, mock_function):
    mock_function.return_value = pd.DataFrame({'index': [0]})
    cached_function = cache(mock_function)

    with benchmark(tmp_path):
        with pytest.warns(UserWarning, match='unhashable'):
            data = cached_function(lambda: 0)
        assert data is mock_function.return_value
        assert not cache_storage_path().exists()


def test_cache_function_ignores_regular_objects(tmp_path, mock_function):
    mock_function.return_value = {'index': [0]}
    cached_function = cache(mock_function)

    with benchmark(tmp_path):
        data = cached_function()
        assert data is mock_function.return_value
        storage_path = cache_storage_path()
        assert storage_path.exists()
        with pd.HDFStore(storage_path) as store:
            assert len(store) == 0


def test_cache_disabled_function_skips_cache(tmp_path, mock_function):
    mock_function.return_value = pd.DataFrame({'index': [0]})
    cached_function = cache(mock_function)

    with benchmark(tmp_path):
        data = cached_function(nocache=True)
        assert data is mock_function.return_value
        assert not cache_storage_path().exists()
