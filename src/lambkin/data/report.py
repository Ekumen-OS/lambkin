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
summarising evo results across all variants and iterations of a benchmark run.
The notebook is written to disk with code cells ready to execute; no Jupyter
installation is required to generate it.
"""

import json
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

_NOTEBOOK_VERSION = (4, 5)

SECTIONS = ("timeseries", "stats", "rmse_bars")


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


def _stem(filename: str) -> str:
    """Return a short uppercase label from a filename.

    E.g. ``'output.ape.zip'`` -> ``'APE'``.
    """
    parts = Path(filename).suffixes
    # e.g. ['.ape', '.zip'] -> 'APE'
    for part in parts:
        label = part.lstrip(".").upper()
        if label not in ("ZIP", "JSON"):
            return label
    return Path(filename).stem.upper()


def _timeseries_cells(results_dir: Path, filename: str) -> list[dict]:
    """Return cells for an error timeseries section."""
    label = _stem(filename)
    var = filename.replace(".", "_").replace("-", "_")
    return [
        _markdown_cell(
            f"## {label} timeseries by variant\n\n"
            "Individual iterations in light color, mean per variant in bold."
        ),
        _code_cell(
            f"_series_{var} = evo_data.series(RESULTS_DIR, {filename!r})\n\n"
            f"_by_variant_{var} = defaultdict(list)\n"
            f"for _entry in _series_{var}:\n"
            f"    _by_variant_{var}[_entry.variant].append(_entry)\n\n"
            "fig, ax = plt.subplots(figsize=(12, 5))\n"
            "colors = plt.rcParams['axes.prop_cycle'].by_key()['color']\n\n"
            f"for (_variant, _entries), _color in zip(\n"
            f"    sorted(_by_variant_{var}.items()), colors\n"
            "):\n"
            "    _label = ', '.join(\n"
            "        f'{k}={v}' for k, v"
            " in sorted(vars(_entries[0].params).items())\n"
            "    )\n"
            "    for _entry in _entries:\n"
            "        ax.plot(\n"
            "            _entry.time, _entry.error,\n"
            "            color=_color, alpha=0.3, linewidth=0.8\n"
            "        )\n"
            "    _t_min = max(e.time[0] for e in _entries)\n"
            "    _t_max = min(e.time[-1] for e in _entries)\n"
            "    _t_grid = np.linspace(_t_min, _t_max, 300)\n"
            "    _mean_error = np.mean(\n"
            "        [np.interp(_t_grid, e.time, e.error)"
            " for e in _entries], axis=0\n"
            "    )\n"
            "    ax.plot(\n"
            "        _t_grid, _mean_error,"
            " color=_color, linewidth=2, label=_label\n"
            "    )\n\n"
            f"ax.set_xlabel('Time (s)')\n"
            f"ax.set_ylabel('Error (m)')\n"
            f"ax.set_title('{label} — timeseries by variant')\n"
            "ax.legend(loc='upper left', fontsize=8)\n"
            "fig.tight_layout()\n"
            f"plt.savefig(RESULTS_DIR / 'report_{label.lower()}_series.png',"
            " dpi=150)\n"
            "plt.show()"
        ),
    ]


def _stats_cells(filename: str) -> list[dict]:
    """Return cells for a stats summary section."""
    label = _stem(filename)
    var = filename.replace(".", "_").replace("-", "_")
    return [
        _markdown_cell(
            f"## {label} stats summary\n\n"
            "RMSE, mean, and max aggregated across iterations per variant."
        ),
        _code_cell(
            f"_stats_{var} = evo_data.stats(RESULTS_DIR, {filename!r})\n\n"
            f"_by_variant_stats_{var} = defaultdict(list)\n"
            f"for _entry in _stats_{var}:\n"
            "    _label = ', '.join(\n"
            "        f'{k}={v}' for k, v"
            " in sorted(vars(_entry.params).items())\n"
            "    )\n"
            f"    _by_variant_stats_{var}[_label].append(_entry)\n\n"
            "_header = (\n"
            "    f\"{'Variant':<40} {'N':>4}\"\n"
            "    f\" {'RMSE mean':>10} {'RMSE std':>10}"
            " {'Mean':>10} {'Max':>10}\"\n"
            ")\n"
            "print(_header)\n"
            "print('-' * len(_header))\n"
            f"for _label in sorted(_by_variant_stats_{var}):\n"
            f"    _entries = _by_variant_stats_{var}[_label]\n"
            "    _rmse = [e.rmse for e in _entries]\n"
            "    print(\n"
            "        f'{_label:<40} {len(_entries):>4}'\n"
            "        f' {np.mean(_rmse):>10.4f} {np.std(_rmse):>10.4f}'\n"
            "        f' {np.mean([e.mean for e in _entries]):>10.4f}'\n"
            "        f' {np.mean([e.max for e in _entries]):>10.4f}'\n"
            "    )"
        ),
    ]


def _rmse_bars_cells(filename: str) -> list[dict]:
    """Return cells for an RMSE bar chart section."""
    label = _stem(filename)
    var = filename.replace(".", "_").replace("-", "_")
    return [
        _markdown_cell(f"## {label} RMSE comparison across variants"),
        _code_cell(
            f"_labels_{var} = sorted(_by_variant_stats_{var}.keys())\n"
            f"_rmse_means_{var} = [\n"
            f"    np.mean([e.rmse for e in _by_variant_stats_{var}[l]])\n"
            f"    for l in _labels_{var}\n"
            "]\n"
            f"_rmse_stds_{var} = [\n"
            f"    np.std([e.rmse for e in _by_variant_stats_{var}[l]])\n"
            f"    for l in _labels_{var}\n"
            "]\n\n"
            "fig, ax = plt.subplots(\n"
            f"    figsize=(max(6, len(_labels_{var}) * 1.2), 4)\n"
            ")\n"
            f"_x_{var} = np.arange(len(_labels_{var}))\n"
            f"ax.bar(_x_{var}, _rmse_means_{var},"
            f" yerr=_rmse_stds_{var}, capsize=4)\n"
            f"ax.set_xticks(_x_{var})\n"
            f"ax.set_xticklabels(_labels_{var},"
            " rotation=25, ha='right', fontsize=8)\n"
            "ax.set_ylabel('RMSE (m)')\n"
            f"ax.set_title('{label} RMSE by variant (mean ± std)')\n"
            "fig.tight_layout()\n"
            f"plt.savefig(RESULTS_DIR / 'report_{label.lower()}_rmse_bars.png',"
            " dpi=150)\n"
            "plt.show()"
        ),
    ]


def generate(
    source: Path | str | object,
    filenames: tuple[str, ...] = ("output.ape.zip",),
    sections: tuple[str, ...] = SECTIONS,
    output_dir: Path | str | None = None,
) -> Path:
    """Generate a Jupyter notebook report from evo benchmark results.

    Reads all completed iterations under ``source`` and writes a
    ``report.ipynb`` notebook to ``output_dir`` (or ``source`` if not
    specified).

    Args:
        source: benchmark context, :class:`~pathlib.Path`, or path string
            pointing to the benchmark base directory.
        filenames: evo result zip files to include in the report, one section
            group per file. Defaults to ``["output.ape.zip"]``.
        sections: sections to include in the notebook. Any subset of
            ``("timeseries", "stats", "rmse_bars")``. Defaults to all three.
        output_dir: directory where ``report.ipynb`` will be written.
            Defaults to the benchmark base directory derived from ``source``.

    Returns:
        :class:`~pathlib.Path` to the generated notebook.

    Raises:
        ValueError: if an unknown section name is provided.
    """
    unknown = set(sections) - set(SECTIONS)
    if unknown:
        raise ValueError(f"Unknown sections: {unknown!r}. Valid sections: {SECTIONS!r}")

    results_dir, out = _resolve_paths(source, output_dir)
    out.mkdir(parents=True, exist_ok=True)

    cells: list[dict] = [
        _markdown_cell(
            "# LAMBKIN Benchmark Report\n\nResults across all variants and iterations."
        ),
        _code_cell(
            "from pathlib import Path\n"
            "from collections import defaultdict\n\n"
            "import matplotlib.pyplot as plt\n"
            "import numpy as np\n\n"
            "from lambkin.data import evo as evo_data\n\n"
            f"RESULTS_DIR = Path({str(results_dir)!r})"
        ),
    ]

    for filename in filenames:
        if not any((results_dir / f"var_1/iter_1/{filename}").exists() for _ in [None]):
            logger.warning(
                "No results found for %s under %s, skipping",
                filename,
                results_dir,
            )
        if "timeseries" in sections:
            cells.extend(_timeseries_cells(results_dir, filename))
        if "stats" in sections:
            cells.extend(_stats_cells(filename))
        if "rmse_bars" in sections:
            cells.extend(_rmse_bars_cells(filename))

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
