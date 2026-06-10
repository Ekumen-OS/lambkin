"""Abstracts access to the new SDK's benchmark filesystem layout."""

import pathlib
from collections.abc import Iterable, Mapping
from typing import NamedTuple

import yaml


class Location(NamedTuple):
    """A path with associated semi-structured metadata."""

    path: pathlib.Path
    metadata: Mapping


Locations = Iterable[Location]


def iterations(base_dir: pathlib.Path) -> Locations:
    """Iterate over all iteration output directories in a benchmark.

    Args:
        base_dir: path to the benchmark base directory (ctx.paths.base_dir).

    Yields:
        Location tuples of (iteration_path, metadata) for each iteration found.
    """
    for meta_path in sorted(base_dir.glob("var_*/iter_*/lambkin_metadata.yaml")):
        iteration_path = meta_path.parent
        with open(meta_path) as f:
            meta = yaml.safe_load(f)
        yield Location(iteration_path, meta)
