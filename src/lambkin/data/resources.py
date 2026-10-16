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

"""Data access module for resource measurement results.

Provides utilities to collect and aggregate the artifacts written by
``background(..., measure=...)`` across benchmark iterations.
"""

import csv
import logging
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from lambkin.data import access

logger = logging.getLogger(__name__)


def _summary_path(iter_dir: Path, process: str) -> Path:
    """Return the path of a process' summary file within an iteration."""
    return iter_dir / f"{process}.resources.yaml"


def _series_path(iter_dir: Path, process: str) -> Path:
    """Return the path of a process' time series file within an iteration."""
    return iter_dir / f"{process}.resources.csv"


def summary(source: Path | str | object, process: str) -> list:
    """Collect resource usage summaries across all iterations.

    Walks all iteration directories and reads the summary written for
    ``process`` by ``background(..., measure=...)``.

    Args:
        source: benchmark context, :class:`~pathlib.Path`, or path string
            pointing to the benchmark base directory.
        process: name of the measured process, as passed to ``measure``.

    Returns:
        A list of :class:`~types.SimpleNamespace` objects, one per iteration,
        each with ``iter_dir``, ``variant``, ``iteration`` and ``params``, plus
        every field of the summary file:

        - ``pid`` and ``pid_found``: the measured process, if it was found.
        - ``samples``: number of samples taken.
        - ``peak_rss_mib`` and ``final_rss_mib``: memory, in mebibytes.
        - ``cpu_user_s``, ``cpu_sys_s``, ``cpu_total_s``: CPU time, in seconds.
        - ``max_threads``: highest thread count observed.
        - ``exited_early`` and ``pid_reused``: measurement caveats.
    """
    results = []
    for entry in access.iterations(source):
        path = _summary_path(entry.iter_dir, process)
        if not path.exists():
            logger.warning("%s is missing", path)
            continue
        data = yaml.safe_load(path.read_text()) or {}
        results.append(
            SimpleNamespace(
                iter_dir=entry.iter_dir,
                variant=entry.variant,
                iteration=entry.iteration,
                params=entry.params,
                **data,
            )
        )
    return results


def series(source: Path | str | object, process: str) -> list:
    """Collect resource usage time series across all iterations.

    Walks all iteration directories and reads the time series written for
    ``process`` by ``background(..., measure=...)``.

    Args:
        source: benchmark context, :class:`~pathlib.Path`, or path string
            pointing to the benchmark base directory.
        process: name of the measured process, as passed to ``measure``.

    Returns:
        A list of :class:`~types.SimpleNamespace` objects, one per iteration,
        each with the following attributes:

        - ``iter_dir``, ``variant``, ``iteration``, ``params``: as returned by
          :func:`lambkin.data.access.iterations`.
        - ``time_s``: seconds since measurement started.
        - ``rss_mib``: resident set size, in mebibytes.
        - ``cpu_user_s``, ``cpu_sys_s``, ``cpu_total_s``: cumulative CPU time,
          in seconds.
        - ``threads``: thread count.
    """
    results = []
    for entry in access.iterations(source):
        path = _series_path(entry.iter_dir, process)
        if not path.exists():
            logger.warning("%s is missing", path)
            continue
        columns = _read_series(path)
        if columns is None:
            continue
        results.append(
            SimpleNamespace(
                iter_dir=entry.iter_dir,
                variant=entry.variant,
                iteration=entry.iteration,
                params=entry.params,
                cpu_total_s=columns["cpu_user_s"] + columns["cpu_sys_s"],
                **columns,
            )
        )
    return results


def _read_series(path: Path) -> dict | None:
    """Read a resource time series CSV into one numpy array per column.

    Args:
        path: Path to the time series file.

    Returns:
        A mapping of column name to array, or None if the file holds no rows.
    """
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        logger.warning("%s has no samples", path)
        return None
    return {
        name: np.asarray([float(row[name]) for row in rows])
        for name in rows[0]
        if name is not None
    }
