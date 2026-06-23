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

"""Report generation for evo benchmark results.

Provides :func:`generate` to produce a self-contained Jupyter notebook
summarising APE results across all variants and iterations of a benchmark run.
The notebook is written to disk with code cells ready to execute; no Jupyter
installation is required to generate it.
"""

import json
import logging
from collections import defaultdict
from pathlib import Path

from lambkin.data import evo as evo_data

logger = logging.getLogger(__name__)

_NOTEBOOK_VERSION = (4, 5)


def _resolve_paths(
    source: Path | str | object,
    output_dir: Path | str | None,
) -> tuple[Path, Path]:
    """Return (results_dir, output_dir) from a source and optional output_dir.

    Args:
        source: benchmark context, :class:`~pathlib.Path`, or path string
            pointing to the benchmark base directory.
        output_dir: directory where the report notebook will be written.
            If ``None``, defaults to the same directory as ``source``.

    Returns:
        A tuple of ``(results_dir, output_dir)`` as :class:`~pathlib.Path`.
    """
    if isinstance(source, (Path, str)):
        results_dir = Path(source)
    else:
        results_dir = source.base_dir  # type: ignore[attr-defined]

    out = Path(output_dir) if output_dir is not None else results_dir
    return results_dir, out


def _code_cell(source: str) -> dict:
    """Return a minimal nbformat v4 code cell dict."""
    return {
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": source,
    }


def _markdown_cell(source: str) -> dict:
    """Return a minimal nbformat v4 markdown cell dict."""
    return {
        "cell_type": "markdown",
        "metadata": {},
        "source": source,
    }


def _variant_label(params: object) -> str:
    """Build a human-readable label from variant params."""
    return ", ".join(f"{k}={v}" for k, v in sorted(vars(params).items()))


def generate(
    source: Path | str | object,
    filename: str = "output.ape.zip",
    output_dir: Path | str | None = None,
) -> Path:
    """Generate a Jupyter notebook report from evo APE benchmark results.

    Reads all completed iterations under ``source`` and writes a
    ``report.ipynb`` notebook to ``output_dir`` (or ``source`` if not
    specified). The notebook contains:

    - An APE timeseries plot per variant (individual iterations + mean).
    - A stats summary table (RMSE, mean, max aggregated across iterations).
    - An RMSE comparison bar chart across variants.

    No Jupyter installation is required to generate the notebook.
    Open it with ``jupyter notebook`` or ``jupyter lab`` to execute it.

    Args:
        source: benchmark context, :class:`~pathlib.Path`, or path string
            pointing to the benchmark base directory.
        filename: name of the evo result zip file to read from each iteration.
            Defaults to ``"output.ape.zip"``.
        output_dir: directory where ``report.ipynb`` will be written.
            Defaults to the benchmark base directory derived from ``source``.

    Returns:
        :class:`~pathlib.Path` to the generated notebook.
    """
    results_dir, out = _resolve_paths(source, output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Collect variant labels from real data to populate the notebook
    stats = evo_data.stats(results_dir, filename)
    if not stats:
        logger.warning(
            "No completed iterations found under %s, report will be empty",
            results_dir,
        )

    by_variant: dict[str, list] = defaultdict(list)
    for entry in stats:
        by_variant[_variant_label(entry.params)].append(entry)

    cells = [
        _markdown_cell(
            "# LAMBKIN Benchmark Report\n\n"
            "APE results across all variants and iterations."
        ),
        _code_cell(
            "from pathlib import Path\n"
            "from collections import defaultdict\n\n"
            "import matplotlib.pyplot as plt\n"
            "import numpy as np\n\n"
            "import lambkin.data.evo as evo_data\n\n"
            f"RESULTS_DIR = Path({str(results_dir)!r})\n"
            f"APE_FILENAME = {filename!r}"
        ),
        _markdown_cell(
            "## APE timeseries by variant\n\n"
            "Individual iterations in light color, mean per variant in bold."
        ),
        _code_cell(
            "series = evo_data.series(RESULTS_DIR, APE_FILENAME)\n\n"
            "by_variant_series = defaultdict(list)\n"
            "for entry in series:\n"
            "    by_variant_series[entry.variant].append(entry)\n\n"
            "fig, ax = plt.subplots(figsize=(12, 5))\n"
            "colors = plt.rcParams['axes.prop_cycle'].by_key()['color']\n\n"
            "for (variant, entries), color in zip(\n"
            "    sorted(by_variant_series.items()), colors\n"
            "):\n"
            "    label = ', '.join(\n"
            "        f'{k}={v}' for k, v in sorted(vars(entries[0].params).items())\n"
            "    )\n"
            "    for entry in entries:\n"
            "        ax.plot(\n"
            "            entry.time, entry.error,\n"
            "            color=color, alpha=0.3, linewidth=0.8\n"
            "        )\n"
            "    t_min = max(e.time[0] for e in entries)\n"
            "    t_max = min(e.time[-1] for e in entries)\n"
            "    t_grid = np.linspace(t_min, t_max, 300)\n"
            "    mean_error = np.mean(\n"
            "        [np.interp(t_grid, e.time, e.error) for e in entries], axis=0\n"
            "    )\n"
            "    ax.plot(t_grid, mean_error, color=color, linewidth=2, label=label)\n\n"
            "ax.set_xlabel('Time (s)')\n"
            "ax.set_ylabel('APE (m)')\n"
            "ax.set_title('Absolute Pose Error — timeseries by variant')\n"
            "ax.legend(loc='upper left', fontsize=8)\n"
            "fig.tight_layout()\n"
            "plt.savefig(RESULTS_DIR / 'report_ape_series.png', dpi=150)\n"
            "plt.show()"
        ),
        _markdown_cell(
            "## Stats summary\n\n"
            "RMSE, mean, and max APE aggregated across iterations per variant."
        ),
        _code_cell(
            "stats = evo_data.stats(RESULTS_DIR, APE_FILENAME)\n\n"
            "by_variant_stats = defaultdict(list)\n"
            "for entry in stats:\n"
            "    label = ', '.join(\n"
            "        f'{k}={v}' for k, v in sorted(vars(entry.params).items())\n"
            "    )\n"
            "    by_variant_stats[label].append(entry)\n\n"
            "header = (\n"
            "    f\"{'Variant':<40} {'N':>4}\"\n"
            "    f\" {'RMSE mean':>10} {'RMSE std':>10} {'Mean':>10} {'Max':>10}\"\n"
            ")\n"
            "print(header)\n"
            "print('-' * len(header))\n"
            "for label in sorted(by_variant_stats):\n"
            "    entries = by_variant_stats[label]\n"
            "    rmse = [e.rmse for e in entries]\n"
            "    print(\n"
            "        f'{label:<40} {len(entries):>4}'\n"
            "        f' {np.mean(rmse):>10.4f} {np.std(rmse):>10.4f}'\n"
            "        f' {np.mean([e.mean for e in entries]):>10.4f}'\n"
            "        f' {np.mean([e.max for e in entries]):>10.4f}'\n"
            "    )"
        ),
        _markdown_cell("## RMSE comparison across variants"),
        _code_cell(
            "labels = sorted(by_variant_stats.keys())\n"
            "rmse_means = [\n"
            "    np.mean([e.rmse for e in by_variant_stats[l]]) for l in labels\n"
            "]\n"
            "rmse_stds = [\n"
            "    np.std([e.rmse for e in by_variant_stats[l]]) for l in labels\n"
            "]\n\n"
            "fig, ax = plt.subplots(figsize=(max(6, len(labels) * 1.2), 4))\n"
            "x = np.arange(len(labels))\n"
            "ax.bar(x, rmse_means, yerr=rmse_stds, capsize=4)\n"
            "ax.set_xticks(x)\n"
            "ax.set_xticklabels(labels, rotation=25, ha='right', fontsize=8)\n"
            "ax.set_ylabel('RMSE (m)')\n"
            "ax.set_title('APE RMSE by variant (mean ± std across iterations)')\n"
            "fig.tight_layout()\n"
            "plt.savefig(RESULTS_DIR / 'report_rmse_bars.png', dpi=150)\n"
            "plt.show()"
        ),
    ]

    notebook = {
        "nbformat": _NOTEBOOK_VERSION[0],
        "nbformat_minor": _NOTEBOOK_VERSION[1],
        "metadata": {
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3",
            },
            "language_info": {
                "name": "python",
                "version": "3.10.0",
            },
        },
        "cells": cells,
    }

    output_path = out / "report.ipynb"
    with open(output_path, "w") as f:
        json.dump(notebook, f, indent=1)

    logger.info("Report written to %s", output_path)
    return output_path
