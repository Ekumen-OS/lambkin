# Copyright 2022 Ekumen, Inc.
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

import pathlib
import json

from typing import Generator
from typing import Iterable
from typing import List
from typing import Set
from typing import Tuple

from scipy.interpolate import interp1d

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

from .aggregator import Aggregator


class TimemAggregator(Aggregator):

    def __init__(self, metrics: Iterable[str], dirpaths: Iterable[pathlib.Path]):
        super().__init__(metrics, dirpaths)
        self.metrics = metrics
        peak_rss: List[float] = []
        cpu_usage: List[float] = []
        series: List[pd.DataFrame] = []
        for i, dirpath in enumerate(dirpaths, start=1):
            path = dirpath / 'timem_output.json'
            with path.open('r') as f:
                output = json.load(f)
            output = output['timemory']
            output = output['timem'][0]
            data = output['peak_rss']
            assert data['unit_repr'] == 'MB'
            peak_rss.append(data['value'])
            data = output['cpu_util']
            assert data['unit_repr'] == '%'
            cpu_usage.append(data['value'])
            data = output['wall_clock']
            assert data['unit_repr'] == 'sec'
            elapsed_time = data['value']
            time = np.arange(0., elapsed_time, 0.2)
            data = np.array([[
                sample['wall_clock']['value'],
                sample['page_rss']['value'],
                sample['virtual_memory']['value'],
            ] for sample in output['history']])
            interpolate_rss = interp1d(
                data[:, 0], data[:, 1],
                bounds_error=False, fill_value=0)
            interpolate_virtual_memory = interp1d(
                data[:, 0], data[:, 2],
                bounds_error=False, fill_value=0)
            series.append(pd.DataFrame({
                'run_id': i, 'time': time, 'rss': interpolate_rss(time),
                'virtual_memory': interpolate_virtual_memory(time)
            }))
        self.df_metrics = pd.DataFrame({
            'peak_rss': peak_rss,
            'cpu_usage': cpu_usage,
        })
        self.df_series = pd.concat(series, ignore_index=True)

    @classmethod
    def get_supported_metrics(cls) -> Set[str]:
        return {'peak_rss', 'cpu_usage', 'virtual_memory'}

    def get_dataframe(self) -> pd.DataFrame:
        included_metrics = self.metrics.copy()
        # TODO(nahuel): generate a metric value for virtual_memory
        included_metrics.remove('virtual_memory')
        return self.df_metrics[included_metrics].rename(columns={
            'peak_rss': 'Peak RSS (MB)',
            'cpu_usage': 'CPU Usage (%)',
        })

    def generate_figures(self) -> Generator[Tuple[str, plt.Figure], None, None]:
        if 'peak_rss' in self.metrics:
            fig = plt.figure()
            ax = fig.gca()
            ax.set_title(f'RSS\nTime series with 95% confidence intervals')
            ax.set_xlabel(f'Time (sec)')
            ax.set_ylabel(f'RSS (MB)')
            sns.lineplot(x='time', y='rss', n_boot=20, data=self.df_series)
            yield 'rss_over_time', fig

            fig = plt.figure()
            ax = fig.gca()
            ax.set_title(f'Peak RSS\nHistogram')
            ax.set_xlabel(f'Peak RSS (MB)')
            ax.set_ylabel(f'Iterations')
            sns.histplot(self.df_metrics['peak_rss'], kde=True, ax=ax)
            yield 'peak_rss', fig

        if 'virtual_memory' in self.metrics:
            fig = plt.figure()
            ax = fig.gca()
            ax.set_title(f'Virtual Memory\nTime series with 95% confidence intervals')
            ax.set_xlabel(f'Time (sec)')
            ax.set_ylabel(f'Virtual Memory (MB)')
            sns.lineplot(x='time', y='virtual_memory', n_boot=20, data=self.df_series)
            yield 'virtual_memory_over_time', fig

        if 'cpu_usage' in self.metrics:
            fig = plt.figure()
            ax = fig.gca()
            ax.set_title(f'CPU Usage\nHistogram')
            ax.set_xlabel(f'CPU Usage (%)')
            ax.set_ylabel(f'Iterations')
            sns.histplot(self.df_metrics['cpu_usage'], kde=True, ax=ax)
            yield 'cpu_usage', fig
