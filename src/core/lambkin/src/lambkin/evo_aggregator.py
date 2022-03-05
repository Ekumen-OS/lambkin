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

import pandas as pd

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
            file_interface.load_res_file(dirpath / f'{self.metric}.zip')
            for dirpath in dirpaths
        ]

        self.df_results = pd.concat([
            pandas_bridge.result_to_df(result, dirpath.name)
            for result, dirpath in zip(results, dirpaths)
        ], axis='columns')

        self.df_metric = pd.DataFrame({
            f'{self.metric}_rmse': pd.to_numeric(
                self.df_results.loc['stats', 'rmse'])
        }).rename_axis('run_id')

        # Collect all trajectory files in TUM format
        traj_list: List[pd.DataFrame] = []
        for dirpath in dirpaths:
            for path in dirpath.glob('*.tum'):
                traj = file_interface.read_tum_trajectory_file(path)
                df = pandas_bridge.trajectory_to_df(traj) \
                    .rename_axis('time').reset_index()
                df['time'] = df['time'] - df.loc[0, 'time']
                df['traj'] = path.stem
                df['run_id'] = dirpath.name
                traj_list.append(df)
        df_traj = pd.concat(traj_list).sort_values(by=['time'])

        df_values = pd.melt(
            frame = pd.DataFrame(
                self.df_results.loc['np_arrays', 'error_array'].tolist(),
                self.df_results.columns).T,
            var_name = 'run_id',
            value_name = self.metric
        ).rename_axis('index')

        df_time = pd.melt(
            frame = pd.DataFrame(
                self.df_results.loc['np_arrays', 'seconds_from_start'].tolist(),
                self.df_results.columns).T,
            var_name = 'run_id',
            value_name = 'time'
        ).rename_axis('index')

        df_merged = df_time.merge(
            df_values, on=['index', 'run_id']
        ).sort_values(by=['time']).dropna()

        # Match metric values to nearest trajectory timepoint
        self.df_timeseries = pd.merge_asof(
            df_traj, df_merged, on='time', by='run_id'
        ).set_index('time')

    @classmethod
    def get_supported_metrics(cls) -> Set[str]:
        return {cls.metric} if cls.metric else {}

    def get_metrics_by_run(self) -> pd.DataFrame:
        return self.df_metric

    def generate_timeseries(self) -> Generator[Tuple[str, pd.DataFrame], None, None]:
        yield self.metric, self.df_timeseries


class ApeAggregator(EvoAggregator):

    metric: str = 'ape'


class RpeAggregator(EvoAggregator):

    metric: str = 'rpe'
