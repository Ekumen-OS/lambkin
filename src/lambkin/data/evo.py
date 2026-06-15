# Copyright 2026 Ekumen, Inc.
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

"""Data access module for evo_ape results.

Provides utilities to collect and aggregate evo_ape result files
produced by benchmark runs.
"""

import warnings
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace
from typing import overload

from lambkin.data import access


@overload
def series(ctx_or_path: Path, filename: str) -> list: ...


@overload
def series(ctx_or_path: object, filename: str) -> list: ...


def series(ctx_or_path, filename):
    """Collect evo_ape timeseries results across all iterations.

    Walks all iteration directories and collects the evo_ape result
    zip file matching ``filename`` from each one.

    Args:
        ctx_or_path: a benchmark context or a :class:`~pathlib.Path` to the
            benchmark base directory.
        filename: name of the evo_ape result zip file (e.g. ``"output.ape.zip"``).

    Returns:
        A list of :class:`~types.SimpleNamespace` objects, one per iteration,
        each with the following attributes:

        - ``iter_dir``: :class:`~pathlib.Path` to the iteration directory.
        - ``variant``: variant directory name.
        - ``iteration``: iteration index.
        - ``params``: variant parameters as a :class:`~types.SimpleNamespace`.
        - ``time``: array of timestamps from start in seconds.
        - ``ape``: array of APE error values in meters.
        - ``distance``: array of distances from start in meters (may be None).
    """
    file_interface = import_module("evo.tools.file_interface")
    results = []
    for entry in access.iterations(ctx_or_path):
        result_path = entry.iter_dir / filename
        if not result_path.exists():
            warnings.warn(f"{result_path} is missing", stacklevel=2)
            continue
        result = file_interface.load_res_file(result_path)
        results.append(
            SimpleNamespace(
                iter_dir=entry.iter_dir,
                variant=entry.variant,
                iteration=entry.iteration,
                params=entry.params,
                time=result.np_arrays.get("seconds_from_start"),
                ape=result.np_arrays.get("error_array"),
                distance=result.np_arrays.get("distances_from_start"),
            )
        )
    return results


@overload
def stats(ctx_or_path: Path, filename: str) -> list: ...


@overload
def stats(ctx_or_path: object, filename: str) -> list: ...


def stats(ctx_or_path, filename):
    """Collect evo_ape statistics across all iterations.

    Walks all iteration directories and collects the evo_ape result
    zip file matching ``filename`` from each one.

    Args:
        ctx_or_path: a benchmark context or a :class:`~pathlib.Path` to the
            benchmark base directory.
        filename: name of the evo_ape result zip file (e.g. ``"output.ape.zip"``).

    Returns:
        A list of :class:`~types.SimpleNamespace` objects, one per iteration,
        each with the following attributes:

        - ``iter_dir``: :class:`~pathlib.Path` to the iteration directory.
        - ``variant``: variant directory name.
        - ``iteration``: iteration index.
        - ``params``: variant parameters as a :class:`~types.SimpleNamespace`.
        - ``rmse``: root mean square error.
        - ``mean``: mean error.
        - ``median``: median error.
        - ``std``: standard deviation.
        - ``min``: minimum error.
        - ``max``: maximum error.
        - ``sse``: sum of squared errors.
    """
    file_interface = import_module("evo.tools.file_interface")
    results = []
    for entry in access.iterations(ctx_or_path):
        result_path = entry.iter_dir / filename
        if not result_path.exists():
            warnings.warn(f"{result_path} is missing", stacklevel=2)
            continue
        result = file_interface.load_res_file(result_path)
        results.append(
            SimpleNamespace(
                iter_dir=entry.iter_dir,
                variant=entry.variant,
                iteration=entry.iteration,
                params=entry.params,
                **result.stats,
            )
        )
    return results
