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

"""Abstracts access to the benchmark filesystem layout.

Provides utilities to traverse benchmark output directories and
read iteration metadata produced by the SDK.
"""

import logging
from pathlib import Path
from types import SimpleNamespace

import yaml

from lambkin.common import defaults

logger = logging.getLogger(__name__)


def iterations(source: Path | str | object) -> list:
    """Traverse the benchmark output tree and return all iteration entries.

    Works without a live context, so it can be called from notebooks or
    standalone reporting scripts.

    Args:
        source: benchmark context, :class:`~pathlib.Path`, or path string
            pointing to the benchmark base directory.

    Returns:
        A list of :class:`~types.SimpleNamespace` objects, one per iteration,
        each with the following attributes:

        - ``iter_dir``: :class:`~pathlib.Path` to the iteration directory.
        - ``variant``: variant directory name (e.g. ``"var_1"``).
        - ``iteration``: iteration index (int).
        - ``params``: :class:`~types.SimpleNamespace` of variant parameters.
    """
    root = (
        Path(source) if isinstance(source, (Path, str)) else source.base_dir  # type: ignore[attr-defined]
    )
    results = []
    for meta_path in sorted(
        root.glob(f"var_*/iter_*/{defaults.METADATA_FILENAME}"),
        key=lambda p: (int(p.parent.parent.name[4:]), int(p.parent.name[5:])),
    ):
        iter_dir = meta_path.parent
        with open(meta_path) as f:
            meta = yaml.safe_load(f)
        if "completed_at" not in meta:
            logger.warning(
                "%s: iteration did not complete successfully, skipping", iter_dir
            )
            continue
        results.append(
            SimpleNamespace(
                iter_dir=iter_dir,
                variant=f"var_{meta['variant_index'] + 1}",
                iteration=meta["iteration"],
                params=SimpleNamespace(**meta["variant"]),
            )
        )
    return results
