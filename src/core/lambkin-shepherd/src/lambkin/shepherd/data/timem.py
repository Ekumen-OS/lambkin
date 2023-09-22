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

"""This module provides APIs to access timemory-timem output in benchmarks."""

import json
import warnings

from collections.abc import Mapping
from typing import Any, Dict, Iterable, NamedTuple, Optional, Union

from lambkin.shepherd.data import access
from lambkin.shepherd.data import pandas
from lambkin.shepherd.data.access import Locations
from lambkin.shepherd.data.units import base_unit_scale
from lambkin.shepherd.utilities import enforce_nonempty, safe_merge

import pandas as pd


def _parse_metric_data(
    basename: str, data: Dict[str, Any]
) -> Dict[str, float]:
    """
    Parse timemory-timem metric data.

    :param name: metric basename.
    :param data: metric semi-structured data.
    :returns: returns mapping of metric names to values in SI units.
    """
    if isinstance(data['value'], dict):
        return {
            f'{basename}.{suffix}': (
                data['value'][suffix] * base_unit_scale(data['unit_repr'][suffix])
            ) for suffix in data['value'].keys()
        }
    return {basename: data['value'] * base_unit_scale(data['unit_repr'])}


Measurement = NamedTuple('Measurement', [
    ('metadata', Mapping),
    ('values', Dict[str, float])
])


@pandas.cache
def history(
    process_name: str, *metric_names: str,
    target_iterations: Optional[Locations] = None,
    normalization: Optional[str] = 'wide'
) -> Union[Iterable[Measurement], pd.DataFrame]:
    """
    Yield process performance records per benchmark iteration as reported by ``timem``.

    Records must have been collected during benchmark execution. See resource usage
    benchmarks' documentation on how to do this.

    :param process_name: name of the process of interest.
    :param metric_names: performance metric names to return.
      If none is provided, all available metrics will be returned.
    :param target_iterations: locations of benchmark case iterations
      to target. If not provided, it defaults to all locations as
      returned by py:func:`lambkin.data.access.iterations()`.
    :param normalization: style for data normalization. Both 'wide'
      and 'long' styles are supported. If `None`, data normalization
      is disabled.
    :return: if normalized, a data frame. Otherwise, a generator that
      yields performance measurements ie. a tuple of semi-structured
      metadata and a mapping bearing named performance figures and a
      timestamp.
    """
    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')

    def _denormalized_impl():
        for path, metadata in target_iterations:
            data_path = path / f'{process_name}.timem.json'
            if not data_path.exists():
                warnings.warn(f'{data_path} is missing')
                continue
            with data_path.open('r') as f:
                data = json.load(f)
            data = data['timemory']
            data = data['timem'][0]
            if not data['history']:
                break
            entry = data['history'][0]
            sample_timestamp = entry['sample_timestamp']
            start_time = sample_timestamp['time_since_epoch'] / 1e9
            for sample in data['history']:
                sample_timestamp = sample.pop('sample_timestamp')
                time_since_epoch = sample_timestamp['time_since_epoch'] / 1e9
                values = dict(
                    time=time_since_epoch - start_time,
                    time_since_epoch=time_since_epoch
                )
                for metric_name, metric_data in sample.items():
                    if not metric_names or metric_name in metric_names:
                        values.update(_parse_metric_data(metric_name, metric_data))
                yield Measurement(metadata, values)

    if normalization is not None:
        if normalization == 'wide':
            return pd.json_normalize([
                safe_merge(metadata, {
                    process_name: {'series': record}
                }) for metadata, record in _denormalized_impl()
            ], sep='.')
        if normalization == 'long':
            return pd.json_normalize([
                safe_merge(metadata, {
                    'process': {'name': process_name, 'series': record},
                }) for metadata, record in _denormalized_impl()
            ], sep='.')
        raise ValueError(f'unknown normalization style: {normalization}')
    return _denormalized_impl()


@pandas.cache
def summary(
    process_name: str, *metric_names: str,
    target_iterations: Optional[Locations] = None,
    normalization: Optional[str] = 'wide'
) -> Union[Iterable[Measurement], pd.DataFrame]:
    """
    Yield process performance summary per benchmark case iteration, as reported by ``timem``.

    Records must have been collected during benchmark execution. See resource usage
    benchmarks' documentation on how to do this.

    :param process_name: name of the process of interest.
    :param metric_names: performance metric names to return.
      If none is provided, all available metrics will be returned.
    :param target_iterations: locations of benchmark case iterations
      to target. If not provided, it defaults to all locations as
      returned by py:func:`lambkin.data.access.iterations()`.
    :param normalization: style for data normalization. Both 'wide'
      and 'long' styles are supported. If `None`, data normalization
      is disabled.
    :return: if normalized, a data frame. Otherwise, a generator that
      yields performance measurements ie. a tuple of semi-structured
      metadata and a performance figures' summary as a mapping.
    """
    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')

    def _denormalized_impl():
        for path, metadata in target_iterations:
            data_path = path / f'{process_name}.timem.json'
            if not data_path.exists():
                warnings.warn(f'{data_path} is missing')
                continue
            with data_path.open('r') as f:
                data = json.load(f)
                data = data['timemory']
                data = data['timem'][0]
                del data['history']
            values = {}
            for metric_name, metric_data in data.items():
                if not metric_names or metric_name in metric_names:
                    values.update(_parse_metric_data(metric_name, metric_data))
            yield Measurement(metadata, values)

    if normalization is not None:
        if normalization == 'wide':
            return pd.json_normalize([
                safe_merge(metadata, {
                    process_name: {'summary': values}
                }) for metadata, values in _denormalized_impl()
            ], sep='.')
        if normalization == 'long':
            return pd.json_normalize([
                safe_merge(metadata, {'process': {
                    'name': process_name,
                    'summary': values
                }}) for metadata, values in _denormalized_impl()
            ], sep='.')
        raise ValueError(f'unknown normalization style: {normalization}')
    return _denormalized_impl()
