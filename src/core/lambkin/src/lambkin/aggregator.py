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

from abc import ABC, abstractmethod

from matplotlib.pyplot import Figure
from pandas import DataFrame
from pathlib import Path

from typing import Generator
from typing import Iterable
from typing import Set
from typing import Tuple


class Aggregator(ABC):
    """Represents an object that can compute one or more metrics from
    multiple benchmark runs and return a dataframe and plots."""

    subclasses = []

    def __init__(self, metrics: Iterable[str], dirpaths: Iterable[Path]):
        """Load input files from each directory path in dirpaths."""
        if not metrics:
            raise ValueError(f'At least one metric is required.')

        for metric in metrics:
            if metric not in self.get_supported_metrics():
                raise ValueError(f'Metric not supported: {metric}.')

    def __init_subclass__(cls, **kwargs):
        """Registers aggregator subclasses."""
        super().__init_subclass__(**kwargs)
        cls.subclasses.append(cls)

    @classmethod
    def get_all_supported_metrics(cls) -> Set[str]:
        """Returns the set of metrics that registered subclasses can compute."""
        return {metric for aggregator_type in cls.subclasses
                       for metric in aggregator_type.get_supported_metrics()}

    @classmethod
    @abstractmethod
    def get_supported_metrics(cls) -> Set[str]:
        """Returns the set of metrics that this aggregator can compute."""
        raise NotImplementedError()

    @abstractmethod
    def get_dataframe(self) -> DataFrame:
        """Returns a dataframe using directory names as index and one
        or more metrics as columns."""
        raise NotImplementedError

    @abstractmethod
    def generate_figures(self) -> Generator[Tuple[str, Figure], None, None]:
        """Generates figures containing plots for these metrics."""
        raise NotImplementedError
