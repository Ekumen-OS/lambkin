"""Abstracts access to the benchmark filesystem layout.

Provides utilities to traverse benchmark output directories and
read iteration metadata produced by the SDK.
"""

from pathlib import Path
from types import SimpleNamespace

import yaml


def iterations(ctx_or_path: object | Path) -> list:
    """Traverse the benchmark output tree and return all iteration entries.

    Accepts either a benchmark context (exposing ``paths.base_dir``) or a
    plain :class:`~pathlib.Path` to the benchmark base directory, so that
    users writing custom reports can call this function directly without
    needing a live context.

    Args:
        ctx_or_path: a benchmark context or a :class:`~pathlib.Path` to the
            benchmark base directory.

    Returns:
        A list of :class:`~types.SimpleNamespace` objects, one per iteration,
        each with the following attributes:

        - ``iter_dir``: :class:`~pathlib.Path` to the iteration directory.
        - ``variant``: variant directory name (e.g. ``"var_1"``).
        - ``iteration``: iteration index (int).
        - ``params``: :class:`~types.SimpleNamespace` of variant parameters.
    """
    root = (
        ctx_or_path.paths.base_dir if not isinstance(ctx_or_path, Path) else ctx_or_path
    )
    results = []
    for meta_path in sorted(root.glob("var_*/iter_*/lambkin_metadata.yaml")):
        iter_dir = meta_path.parent
        variant_name = iter_dir.parent.name
        with open(meta_path) as f:
            meta = yaml.safe_load(f)
        results.append(
            SimpleNamespace(
                iter_dir=iter_dir,
                variant=variant_name,
                iteration=meta.get("iteration", 0),
                params=SimpleNamespace(**meta.get("variant", {})),
            )
        )
    return results
