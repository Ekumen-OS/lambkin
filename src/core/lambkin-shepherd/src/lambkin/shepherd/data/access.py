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

"""This module abstracts access to benchmarks' base filesystem layout."""

import contextlib
import pathlib
import json
import os

from collections.abc import Mapping
from typing import Any, Iterable, NamedTuple, Optional, Protocol

from lambkin.shepherd.utilities import enforce_nonempty, safe_merge


class BenchmarkLike(Protocol):
    """Stateful py:func:`benchmark` context manager type annotation."""

    path: Optional[pathlib.Path]


def benchmark_like(func: Any) -> BenchmarkLike:
    """Decorate generator based context manager with `BenchmarkLike` type annotation."""
    return func


@benchmark_like
@contextlib.contextmanager
def benchmark(path: Optional[pathlib.Path]):
    """Bind lambkin settings to execution scope."""
    path, benchmark.path = benchmark.path, path
    try:
        yield
    finally:
        benchmark.path = path


benchmark.path = None


def current_path() -> pathlib.Path:
    """
    Return the path to the current benchmark.

    This path is computed following the precedence order below:

    - the path set by the innermost py:func:`benchmark()` context manager, or
    - the path set by the $LAMBKIN_BENCHMARK environment variable, or
    - the current working directory.
    """
    return (
        benchmark.path or pathlib.Path(
            os.environ.get('BENCHMARK_ROOT', '.'))
    ).resolve()


def metadata(path: pathlib.Path, **extras) -> Mapping:
    """Return semi-structured metadata at output `path`, if any."""
    metadata_path = path / 'metadata.json'
    if not metadata_path.exists():
        return extras or {}
    with metadata_path.open('r') as metadata_file:
        return safe_merge(json.load(metadata_file), extras)


class Location(NamedTuple):
    """A path with associated semi-structured metadata."""

    path: pathlib.Path
    metadata: Mapping


Locations = Iterable[Location]


def cases(
    benchmark_path: Optional[pathlib.Path] = None
) -> Iterable[Location]:
    """
    Iterate over output data for each case in a benchmark.

    :param benchmark_path: path to the the target benchmark.
        If not provided, it defaults to the current `root()`.
    :returns: an iterable over data locations.
    """
    if benchmark_path is None:
        benchmark_path = current_path()
    cases_path = benchmark_path / 'cases'
    if not cases_path.exists():
        raise ValueError(f'no cases found in {benchmark_path}')
    for path in cases_path.glob('*'):
        yield Location(path, {'case': metadata(path, root=str(path))})


def variations(
    target_cases: Optional[Iterable[Location]] = None
) -> Iterable[Location]:
    """
    Iterate over output data for each variation of each case in a benchmark.

    :param target_cases: locations of benchmark cases to target. If not provided,
      it defaults to all locations returned by py:func:`lambkin.data.access.cases()`.
    :returns: an iterable over data locations.
    """
    if target_cases is None:
        target_cases = cases()
    target_cases = enforce_nonempty(target_cases, 'no target cases')
    assert target_cases is not None
    for root_path, root_metadata in target_cases:
        variations_path = root_path / 'variations'
        if not variations_path.exists():
            raise ValueError(f'no variations found in {root_path}')
        for path in variations_path.glob('*'):
            yield Location(path, safe_merge(root_metadata, {
                'variation': metadata(path, index=int(path.name))
            }))


def iterations(
    target_variations: Optional[Iterable[Location]] = None
) -> Iterable[Location]:
    """
    Iterate over output data for each iteration in each variation of each case in a benchmark.

    :param target_variations: locations of benchmark case variations to target. If not provided,
      it defaults to all locations as returned by py:func:`lambkin.data.access.variations()`.
    :returns: an iterable over data locations.
    """
    if target_variations is None:
        target_variations = variations()
    target_variations = enforce_nonempty(
        target_variations, 'no target cases')
    assert target_variations is not None
    for root_path, root_metadata in target_variations:
        iterations_path = root_path / 'iterations'
        if not iterations_path.exists():
            raise ValueError(f'no iterations found in {root_path}')
        for path in iterations_path.glob('*'):
            yield Location(path, safe_merge(root_metadata, {
                'iteration': metadata(path, index=int(path.name))
            }))
