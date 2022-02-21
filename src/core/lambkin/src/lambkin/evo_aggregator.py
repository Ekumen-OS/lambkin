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

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

from evo.core.result import Result
from evo.tools import file_interface, pandas_bridge
from pathlib import Path
from typing import Generator
from typing import Iterable
from typing import List
from typing import Set
from typing import Tuple

from .aggregator import Aggregator


class EvoAggregator(Aggregator):

    metric: str = ''

    def __init__(self, metrics: Iterable[str], dirpaths: Iterable[Path]):
        super().__init__(metrics, dirpaths)

        results: List[Result] = [
            file_interface.load_res_file(directory / f'{self.metric}.zip')
            for directory in dirpaths
        ]

        self.df_results = pd.concat([
            pandas_bridge.result_to_df(result, dirname)
            for result, dirname in zip(results, dirpaths)
        ], axis='columns')

        self.df_metrics = self.df_results.loc['stats', 'rmse'].T.rename(f'{self.metric.upper()} rmse (m)')

    @classmethod
    def get_supported_metrics(cls) -> Set[str]:
        return {cls.metric} if cls.metric else {}

    def get_dataframe(self) -> pd.DataFrame:
        return self.df_metrics

    def generate_figures(self) -> Generator[Tuple[str, plt.Figure], None, None]:
        yield f'{self.metric.upper()}', self._figure()

    def _get_timeseries(self) -> pd.DataFrame:
        error_arrays = self.df_results.loc['np_arrays', 'error_array'].tolist()
        time_arrays = self.df_results.loc['np_arrays', 'seconds_from_start'].tolist()

        df = pd.DataFrame(error_arrays, self.df_results.columns).T
        df['time'] = pd.DataFrame(time_arrays).T.mean(axis='columns')

        return df.set_index('time')

    def _figure(self) -> plt.Figure:
        melted_df = pd.melt(
            frame = self._get_timeseries(),
            var_name = 'name',
            value_name = 'value',
            ignore_index = False
        ).reset_index()

        title = self.df_results.loc['info', 'title'][0]

        fig, axes = plt.subplots(1, 2, figsize=(10,5))
        axes[0].set_title(f'{title}\nTime series with 95% confidence intervals')
        axes[0].set(xlabel='t (s)', ylabel=f'{self.metric.upper()} (m)')

        axes[1].set_title(f'{title}\nHistogram')
        axes[1].set(xlabel=f'{self.metric.upper()} rmse (m)', ylabel='Percentage of iterations (%)')

        sns.lineplot(data=melted_df, x='time', y='value', n_boot=20, ax=axes[0])
        sns.histplot(data=self.df_metrics, stat='percent', kde=True, ax=axes[1])
        return fig


class ApeAggregator(EvoAggregator):

    metric: str = 'ape'


class RpeAggregator(EvoAggregator):

    metric: str = 'rpe'
