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
import functools
import hashlib
import pickle
import warnings

from typing import Any, Callable, Dict, Iterable, List, Union

import numpy as np
import pandas as pd
import tables as tb

from lambkin.shepherd.data import access
from lambkin.shepherd.utilities import fqn


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


def cache_storage_path():
    """Return the path to the current benchmark cache storage."""
    return access.current_path() / '.cache' / 'storage.h5'


def cache(func: Callable[..., Any]) -> Callable[..., Any]:
    """
    Decorate a function to cache `pandas.DataFrame` return values.

    Return values other than data frames are ignored. Caching can be
    disabled for a function call by passing `nocache=True`.

    :param func: a function that may return a data frame.
    """
    @functools.wraps(func)
    def __wrapper(*args: Any, nocache: bool = False, **kwargs: Any) -> Any:
        if nocache:
            return func(*args, **kwargs)
        try:
            uid = hashlib.sha1(pickle.dumps((args, kwargs))).hexdigest()
        except (TypeError, AttributeError):
            warnings.warn('unhashable arguments, cannot cache')
            return func(*args, **kwargs)

        func_key = fqn(func).replace('.', '_')
        cache_key = f'{func_key}@{uid}'

        path = cache_storage_path()
        path.parent.mkdir(exist_ok=True)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', tb.NaturalNameWarning)
            with pd.HDFStore(path) as store:
                if cache_key in store:
                    return store.get(cache_key)
                rvalue = func(*args, **kwargs)
                if isinstance(rvalue, pd.DataFrame):
                    store.put(cache_key, rvalue)
                return rvalue
    return __wrapper
