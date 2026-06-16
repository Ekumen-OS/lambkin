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

"""Data access module for evo results.

Provides utilities to collect and aggregate result files produced by evo
tools (e.g. evo_ape, evo_rpe) across benchmark iterations.
"""

import logging
from pathlib import Path
from types import SimpleNamespace

from evo.tools import file_interface  # type: ignore[import-untyped]

from lambkin.data import access

logger = logging.getLogger(__name__)


def series(source: Path | str | object, filename: str) -> list:
    """Collect evo timeseries results across all iterations.

    Walks all iteration directories and collects the evo result
    zip file matching ``filename`` from each one.

    Args:
        source: benchmark context, :class:`~pathlib.Path`, or path string
            pointing to the benchmark base directory.
        filename: name of the evo result zip file (e.g. ``"output.ape.zip"``).

    Returns:
        A list of :class:`~types.SimpleNamespace` objects, one per iteration,
        each with the following attributes:

        - ``iter_dir``: :class:`~pathlib.Path` to the iteration directory.
        - ``variant``: variant directory name.
        - ``iteration``: iteration index.
        - ``params``: variant parameters as a :class:`~types.SimpleNamespace`.
        - ``time``: array of timestamps from start in seconds.
        - ``error``: array of error values in meters.
        - ``distance``: array of distances from start in meters (may be None).
    """
    results = []
    for entry in access.iterations(source):
        result_path = entry.iter_dir / filename
        if not result_path.exists():
            logger.warning("%s is missing", result_path)
            continue
        result = file_interface.load_res_file(result_path)
        results.append(
            SimpleNamespace(
                iter_dir=entry.iter_dir,
                variant=entry.variant,
                iteration=entry.iteration,
                params=entry.params,
                time=result.np_arrays.get("seconds_from_start"),
                error=result.np_arrays.get("error_array"),
                distance=result.np_arrays.get("distances_from_start"),
            )
        )
    return results


def stats(source: Path | str | object, filename: str) -> list:
    """Collect evo statistics across all iterations.

    Walks all iteration directories and collects the evo result
    zip file matching ``filename`` from each one.

    Args:
        source: a benchmark context or a :class:`~pathlib.Path` to the
            benchmark base directory.
        filename: name of the evo result zip file (e.g. ``"output.ape.zip"``).

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
    results = []
    for entry in access.iterations(source):
        result_path = entry.iter_dir / filename
        if not result_path.exists():
            logger.warning("%s is missing", result_path)
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
