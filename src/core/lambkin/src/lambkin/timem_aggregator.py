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
import pandas as pd

from .aggregator import Aggregator


class TimemAggregator(Aggregator):

    def __init__(self, metrics: Iterable[str], dirpaths: Iterable[pathlib.Path]):
        super().__init__(metrics, dirpaths)
        self.metrics = metrics
        peak_rss: List[float] = []
        cpu_usage: List[float] = []
        series: List[pd.DataFrame] = []
        indexes: List[int] = []
        for dirpath in dirpaths:
            run_id = dirpath.name
            indexes.append(run_id)
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
                'run_id': run_id, 'time': time, 'rss': interpolate_rss(time),
                'virtual_memory': interpolate_virtual_memory(time)
            }))
        self.df_metrics = pd.DataFrame({
            'run_id': indexes,
            'peak_rss': peak_rss,
            'cpu_usage': cpu_usage,
        }).set_index('run_id')
        self.df_series = pd.concat(series, ignore_index=True).set_index('time')

    @classmethod
    def get_supported_metrics(cls) -> Set[str]:
        return {'peak_rss', 'cpu_usage', 'virtual_memory'}

    def get_metrics_by_run(self) -> pd.DataFrame:
        included_metrics = self.metrics.copy()
        # TODO(nahuel): generate a metric value for virtual_memory
        included_metrics.discard('virtual_memory')
        return self.df_metrics[list(included_metrics)]

    def generate_timeseries(self) -> Generator[Tuple[str, pd.DataFrame], None, None]:
        yield 'rss', self.df_series[['run_id', 'rss']]
        yield 'virtual_memory', self.df_series[['run_id', 'virtual_memory']]
