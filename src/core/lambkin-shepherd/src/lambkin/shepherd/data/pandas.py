# Copyright 2023 Ekumen, Inc.
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

"""This module provides additional APIs specific to pandas for convenience."""

import collections

from typing import Dict, Iterable, List, Union

import numpy as np
import pandas as pd


def inner_join(
    input_dataframes: Iterable[pd.DataFrame],
    on: Union[str, Iterable[str]]
) -> pd.DataFrame:
    """
    Perform an inner join over all `input_dataframes` using the given column or columns as index.

    Duplicate column labels, if any, will be deduplicated by appending a monotonically increasing
    number to each label after the first ie. label, label_1, label_2.

    :param input_dataframes: dataframes to be joined.
    :param on: column or columns common to all dataframes to be used as indices
      for the inner join operation.
    :returns: the resulting dataframe.
    """
    output_dataframe = pd.concat([
        df.set_index(on) for df in input_dataframes
    ], join='inner', axis=1)
    output_dataframe.reset_index(inplace=True)
    deduplicated_columns: List[str] = []
    column_counts: Dict[str, int] = collections.defaultdict(int)
    for column_name in output_dataframe.columns:
        unique_column_name = column_name
        if column_counts[column_name] > 0:
            unique_column_name += f'_{column_counts[column_name]}'
        deduplicated_columns.append(unique_column_name)
        column_counts[column_name] += 1
    output_dataframe.columns = deduplicated_columns
    return output_dataframe


def rescale(
    input_dataframe: pd.DataFrame,
    scale_factors: Dict[str, float]
) -> pd.DataFrame:
    """
    Scale dataframe on a per column basis.

    :param input_dataframe: dataframe to be scaled.
    :param scale_factors: scale factors per column.
    :returns: scaled dataframe.
    """
    output_dataframe = input_dataframe.copy()
    output_dataframe[list(scale_factors.keys())] *= \
        np.array(list(scale_factors.values()))
    return output_dataframe