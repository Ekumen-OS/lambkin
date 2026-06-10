"""Data access module for evo_ape results.

Provides utilities to collect and aggregate evo_ape result files
produced by benchmark runs.
"""

import os
import warnings
from collections.abc import Iterable, Mapping
from importlib import import_module

import numpy as np
import pandas as pd

from lambkin.common.utilities import enforce_nonempty, safe_merge
from lambkin.data import access
from lambkin.data.access import Locations


def _to_evo_filestem(name: str, name_format: str) -> str:
    """Convert name to a filestem like evo does.

    Args:
        name: trajectory or file name.
        name_format: format of the name ('ros' or 'path').

    Returns:
        A filestem string.

    Raises:
        ValueError: if name_format is unknown.
    """
    if name_format == "ros":
        return name.lstrip("/").replace(":", "/").replace("/", "_")
    if name_format == "path":
        return os.path.splitext(os.path.basename(name))[0]
    raise ValueError(f"unknown name format: {name_format}")


def series(ctx, filename):
    """Collect evo_ape results across all variations and iterations.

    Convenience function for use in ``@nominal.output`` hooks. Walks all
    iteration directories under ``ctx.paths.base_dir`` and collects all
    available data from each evo_ape result zip file.

    Args:
        ctx: context of the last benchmark iteration.
        filename: name of the evo_ape result zip file (e.g. ``"output.ape.zip"``).

    Returns:
        A dict with keys ``"time"``, ``"ape"``, ``"distance"``, ``"stats"``,
        and ``"info"``, each a list with one entry per iteration.
    """
    result_data = {"time": [], "ape": [], "distance": [], "stats": [], "info": []}
    file_interface = import_module("evo.tools.file_interface")
    for path, _ in access.iterations(ctx.paths.base_dir):
        result_path = path / filename
        if not result_path.exists():
            warnings.warn(f"{result_path} is missing", stacklevel=2)
            continue
        result = file_interface.load_res_file(result_path)
        result_data["time"].append(result.np_arrays.get("seconds_from_start"))
        result_data["ape"].append(result.np_arrays.get("error_array"))
        result_data["distance"].append(result.np_arrays.get("distances_from_start"))
        result_data["stats"].append(result.stats)
        result_data["info"].append(result.info)
    return result_data


def _series(
    trajectory_name: str,
    metric_name: str,
    *,
    target_iterations: Locations,
    trajectory_name_format: str = "ros",
    normalization: str | None = "wide",
) -> Iterable[tuple[Mapping, np.ndarray, np.ndarray]] | pd.DataFrame:
    """Yield trajectory metric timeseries per benchmark iteration.

    Args:
        trajectory_name: name of the trajectory of interest.
        metric_name: name of the metric of interest (e.g. ape, rpe).
        target_iterations: iteration locations to target.
        trajectory_name_format: name format for trajectories as used by evo.
        normalization: style for data normalization ('wide', 'long', or None).

    Returns:
        A DataFrame if normalized, otherwise a generator of (metadata, time,
        value) tuples.

    Raises:
        ValueError: if normalization style is unknown.
    """
    target_iterations = enforce_nonempty(target_iterations, "no target iterations")

    trajectory_filestem = _to_evo_filestem(trajectory_name, trajectory_name_format)
    metric_filename = f"{trajectory_filestem}.{metric_name}.zip"

    def _denormalized_impl():
        file_interface = import_module("evo.tools.file_interface")
        for path, metadata in target_iterations:
            result_path = path / metric_filename
            if not result_path.exists():
                warnings.warn(f"{result_path} is missing", stacklevel=2)
                continue
            result = file_interface.load_res_file(result_path)
            time = result.np_arrays["seconds_from_start"]
            value = result.np_arrays["error_array"]
            yield metadata, time, value

    if normalization == "wide":
        normalized_metric_name = f"{trajectory_name}.{metric_name}"
        df = pd.json_normalize(
            [
                safe_merge(
                    metadata,
                    {
                        normalized_metric_name: {
                            "series": {"time": time, "value": value}
                        }
                    },
                )
                for metadata, time, value in _denormalized_impl()
            ],
            sep=".",
        )
        return (
            df
            .explode(
                list(df.columns[df.columns.str.startswith(normalized_metric_name)])
            )
            .reset_index(drop=True)
            .infer_objects()
        )
    if normalization == "long":
        df = pd.json_normalize(
            [
                safe_merge(
                    metadata,
                    {
                        "trajectory": {"name": trajectory_name},
                        "metric": {
                            "name": metric_name,
                            "series": {"time": time, "value": value},
                        },
                    },
                )
                for metadata, time, value in _denormalized_impl()
            ],
            sep=".",
        )
        return (
            df
            .explode(list(df.columns[df.columns.str.startswith("metric.series")]))
            .reset_index(drop=True)
            .infer_objects()
        )
    if normalization is None:
        return _denormalized_impl()
    raise ValueError(f"unknown normalization style: {normalization}")


def _stats(
    trajectory_name: str,
    metric_name: str,
    *,
    target_iterations: Locations,
    trajectory_name_format: str = "ros",
    normalization: str | None = "wide",
) -> Iterable[tuple[Mapping, Mapping]] | pd.DataFrame:
    """Yield trajectory metric statistics per benchmark iteration.

    Args:
        trajectory_name: name of the trajectory of interest.
        metric_name: name of the metric of interest (e.g. ape, rpe).
        target_iterations: iteration locations to target.
        trajectory_name_format: name format for trajectories as used by evo.
        normalization: style for data normalization ('wide', 'long', or None).

    Returns:
        A DataFrame if normalized, otherwise a generator of (metadata, stats) tuples.

    Raises:
        ValueError: if normalization style is unknown.
    """
    target_iterations = enforce_nonempty(target_iterations, "no target iterations")

    trajectory_filestem = _to_evo_filestem(trajectory_name, trajectory_name_format)
    result_filename = f"{trajectory_filestem}.{metric_name}.zip"

    def _denormalized_impl():
        file_interface = import_module("evo.tools.file_interface")
        for path, metadata in target_iterations:
            result_path = path / result_filename
            if not result_path.exists():
                warnings.warn(f"{result_path} is missing", stacklevel=2)
                continue
            result = file_interface.load_res_file(result_path)
            yield metadata, result.stats

    if normalization == "wide":
        normalized_metric_name = f"{trajectory_name}.{metric_name}"
        return (
            pd
            .json_normalize(
                [
                    safe_merge(metadata, {normalized_metric_name: statistics})
                    for metadata, statistics in _denormalized_impl()
                ],
                sep=".",
            )
            .reset_index(drop=True)
            .infer_objects()
        )
    if normalization == "long":
        return (
            pd
            .json_normalize(
                [
                    safe_merge(
                        metadata,
                        {
                            "trajectory": {"name": trajectory_name},
                            "metric": {"name": metric_name, **statistics},
                        },
                    )
                    for metadata, statistics in _denormalized_impl()
                ],
                sep=".",
            )
            .reset_index(drop=True)
            .infer_objects()
        )
    if normalization is None:
        return _denormalized_impl()
    raise ValueError(f"unknown normalization style: {normalization}")
