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

"""This module provides APIs to access ``evo`` output in benchmarks."""

import os
import warnings

from importlib import import_module
from collections.abc import Mapping
from typing import Iterable, Optional, Protocol, Tuple, Union

import numpy as np
import pandas

from lambkin.data import access
from lambkin.data.access import Locations
from lambkin.utilities import enforce_nonempty, safe_merge


def _to_evo_filestem(name: str, name_format: str) -> str:
    """Convert `name` to a filestem like ``evo`` does."""
    if name_format == 'ros':
        return name.lstrip('/').replace(':', '/').replace('/', '_')
    if name_format != 'path':
        raise ValueError(f'unknown name format: {name_format}')
    return os.path.splitext(os.path.basename(name))[0]


def stats(
    trajectory_name: str, metric_name: str,
    target_iterations: Optional[Locations] = None, /,
    trajectory_name_format: str = 'ros',
    normalization: Optional[str] = 'wide'
) -> Union[Iterable[Tuple[Mapping, Mapping]], pandas.DataFrame]:
    """
    Yield trajectory metric statistics per benchmark case iteration, as reported by ``evo``.

    Metric statistics must have been collected during execution. See SLAM system benchmarks'
    documentation on how to do this.

    :param trajectory_name: name of the trajectory of interest,
    :param metric_name: name of the metric of interest (e.g. ape, rpe).
    :param target_iterations: locations of benchmark case iterations to target. If not provided,
      it defaults to all locations as returned by py:func:`lambkin.data.access.iterations()`.
    :param trajectory_name_format: name format for trajectories as used by ``evo``. Both 'ros'
      (i.e. topic based) and 'path' formats are supported.
    :param normalization: style for data normalization. Both 'wide' and 'long' styles are
      supported. If `None`, data normalization is disabled.
    :return: if normalized, a data frame. Otherwise, a generator that yields a tuple of
      semi-structured metadata and an statistics mapping per benchmark case iteration.
    """
    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')

    trajectory_filestem = \
        _to_evo_filestem(trajectory_name, trajectory_name_format)
    result_filename = f'{trajectory_filestem}.{metric_name}.zip'

    def _denormalized_impl():
        file_interface = import_module('evo.tools.file_interface')
        for path, metadata in target_iterations:
            result_path = path / result_filename
            if not result_path.exists():
                warnings.warn(f'{result_path} is missing')
                continue
            result = file_interface.load_res_file(result_path)
            yield metadata, result.stats

    if normalization is not None:
        if normalization == 'wide':
            normalized_metric_name = f'{trajectory_name}.{metric_name}'
            return pandas.json_normalize([
                safe_merge(metadata, {
                    normalized_metric_name: statistics
                }) for metadata, statistics in _denormalized_impl()
            ], sep='.').reset_index(drop=True)
        if normalization == 'long':
            return pandas.json_normalize([
                safe_merge(metadata, {
                    'trajectory': {'name': trajectory_name},
                    'metric': {'name': metric_name, **statistics}
                }) for metadata, statistics in _denormalized_impl()
            ], sep='.').reset_index(drop=True)
        raise ValueError(f'unknown normalization style: {normalization}')
    return _denormalized_impl()


def series(
    trajectory_name: str, metric_name: str,
    target_iterations: Optional[Locations] = None, /,
    trajectory_name_format: str = 'ros',
    normalization: Optional[str] = 'wide'
) -> Union[Iterable[Tuple[Mapping, np.ndarray, np.ndarray]], pandas.DataFrame]:
    """
    Yield trajectory metric timeseries per benchmark case iteration, as reported by ``evo``.

    Metric timeseries must have been collected during execution. See SLAM system benchmarks'
    documentation on how to do this.

    :param trajectory_name: name of the trajectory of interest,
    :param metric_name: name of the metric of interest (e.g. ape, rpe).
    :param target_iterations: locations of benchmark case iterations to target. If not provided,
      it defaults to all locations as returned by py:func:`lambkin.data.access.iterations()`.
    :param trajectory_name_format: name format for trajectories as used by ``evo``. Both 'ros'
      (i.e. topic based) and 'path' formats are supported.
    :param normalization: style for data normalization. Both 'wide' and 'long' styles are
      supported. If `None`, data normalization is disabled.
    :return: if normalized, a data frame. Otherwise, a generator that yields a tuple of
      semi-structured metadata, array of timestamps, and array of values per benchmark case
      iteration.
    """
    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')

    trajectory_filestem = \
        _to_evo_filestem(trajectory_name, trajectory_name_format)
    metric_filename = f'{trajectory_filestem}.{metric_name}.zip'

    def _denormalized_impl():
        file_interface = import_module('evo.tools.file_interface')
        for path, metadata in target_iterations:
            result_path = path / metric_filename
            if not result_path.exists():
                warnings.warn(f'{result_path} is missing')
                continue
            result = file_interface.load_res_file(result_path)
            time = result.np_arrays['seconds_from_start']
            value = result.np_arrays['error_array']
            yield metadata, time, value

    if normalization is not None:
        if normalization == 'wide':
            normalized_metric_name = f'{trajectory_name}.{metric_name}'
            df = pandas.json_normalize([
                safe_merge(metadata, {normalized_metric_name: {
                    'series': {'time': time, 'value': value}
                }}) for metadata, time, value in _denormalized_impl()
            ], sep='.')
            return df.explode(list(df.columns[
                df.columns.str.startswith(normalized_metric_name)
            ])).reset_index(drop=True)
        if normalization == 'long':
            df = pandas.json_normalize([
                safe_merge(metadata, {
                    'trajectory': {'name': trajectory_name},
                    'metric': {
                        'name': metric_name, 'series': {
                            'time': time, 'value': value
                        }
                    }
                }) for metadata, time, value in _denormalized_impl()
            ], sep='.')
            return df.explode(list(df.columns[
                df.columns.str.startswith('metric.series')
            ])).reset_index(drop=True)
        raise ValueError(f'unknown normalization style: {normalization}')
    return _denormalized_impl()


class PosePath3DType(Protocol):
    """py:class:`evo.core.trajectory.PosePath3D` type stub."""

    positions_xyz: np.ndarray
    orientations_quat_wxyz: np.ndarray


class PoseTrajectory3DType(PosePath3DType):
    """py:class:`evo.core.trajectory.PoseTrajectory3D` type stub."""

    timestamps: np.ndarray


SomeTrajectoryType = Union[PosePath3DType, PoseTrajectory3DType]


def _to_dict(trajectory: SomeTrajectoryType) -> Mapping:
    """Convert `trajectory` to a semi-structured mapping."""
    if hasattr(trajectory, 'timestamps'):
        time_since_epoch = trajectory.timestamps
        time = time_since_epoch - time_since_epoch[0]
    else:
        time = time_since_epoch = np.full_like(
            trajectory.positions_xyz[:, 0], np.nan)
    return {
        'time': time,
        'time_since_epoch': time_since_epoch,
        'x': trajectory.positions_xyz[:, 0],
        'y': trajectory.positions_xyz[:, 1],
        'z': trajectory.positions_xyz[:, 2],
        'qw': trajectory.orientations_quat_wxyz[:, 0],
        'qx': trajectory.orientations_quat_wxyz[:, 1],
        'qy': trajectory.orientations_quat_wxyz[:, 2],
        'qz': trajectory.orientations_quat_wxyz[:, 3]
    }


def trajectory(
    trajectory_name: str, target_iterations: Optional[Locations] = None, /,
    trajectory_name_format: str = 'ros', trajectory_file_format: str = 'tum',
    normalization: Optional[str] = 'long'
) -> Union[Iterable[Tuple[Mapping, SomeTrajectoryType]], pandas.DataFrame]:
    """
    Yield a trajectory per benchmark case iteration, as reported by ``evo``.

    Trajectories must have been collected during execution. See SLAM system
    benchmarks' documentation on how to do this.

    :param trajectory_name: name of the trajectory of interest.
    :param target_iterations: locations of benchmark case iterations to target.
      If not provided, it defaults to all locations as
      returned by py:func:`lambkin.data.access.iterations()`.
    :param trajectory_name_format: name format for trajectories as used by ``evo``.
      Both 'ros' (i.e. topic based) and 'path' formats are supported.
    :param trajectory_file_format: file format for trajectory data as used by ``evo``.
      Both 'tum' and 'kitti' files are supported.
    :param normalization: style for data normalization. Both 'wide' and 'long' styles
      are supported. If `None`, data normalization is disabled.
    :return: if normalized, a data frame. Otherwise, a generator that yields a tuple of
      semi-structured metadata and trajectory.
    """
    if target_iterations is None:
        target_iterations = access.iterations()
    target_iterations = enforce_nonempty(
        target_iterations, 'no target iterations')

    file_interface = import_module('evo.tools.file_interface')

    trajectory_file_format = trajectory_file_format.lower()
    if trajectory_file_format == 'tum':
        do_read_trajectory = file_interface.read_tum_trajectory_file
    elif trajectory_file_format == 'kitti':
        do_read_trajectory = file_interface.read_kitti_poses_file
    else:
        raise ValueError(f'unknown file format: {trajectory_file_format}')

    trajectory_filestem = \
        _to_evo_filestem(trajectory_name, trajectory_name_format)
    trajectory_filename = f'{trajectory_filestem}.{trajectory_file_format}'

    def _denormalized_impl():
        for path, metadata in target_iterations:
            trajectory_path = path / trajectory_filename
            if not trajectory_path.exists():
                warnings.warn(f'{trajectory_path} is missing')
                continue
            yield metadata, do_read_trajectory(trajectory_path)

    if normalization is not None:
        if normalization == 'wide':
            df = pandas.json_normalize([
                safe_merge(metadata, {
                    trajectory_name: _to_dict(trajectory)
                }) for metadata, trajectory in _denormalized_impl()
            ], sep='.')
            return df.explode(list(df.columns[
                df.columns.str.startswith(trajectory_name)
            ])).reset_index(drop=True)
        if normalization == 'long':
            df = pandas.json_normalize([
                safe_merge(metadata, {'trajectory': {
                    'name': trajectory_name, **_to_dict(trajectory)
                }}) for metadata, trajectory in _denormalized_impl()
            ], sep='.')
            return df.explode(list(df.columns[
                df.columns.str.startswith('trajectory') & (df.columns != 'trajectory.name')
            ])).reset_index(drop=True)
        raise ValueError(f'unknown normalization style: {normalization}')
    return _denormalized_impl()
