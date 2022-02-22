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

import argparse
import pandas as pd
import json

from pathlib import Path
from typing import Dict
from typing import Iterable
from typing import List
from typing import Type

from .aggregator import Aggregator
from .evo_aggregator import ApeAggregator
from .evo_aggregator import RpeAggregator
from .timem_aggregator import TimemAggregator


class AggregatorFactory:
    """Represents a factory that can create collections of aggregators based
    on registered metrics."""

    def __init__(self, metrics: Iterable[str]):
        self.aggregator_metrics: Dict[Type[Aggregator], Iterable[str]] = {}

        metric_set = set(metrics)
        for aggregator_type in Aggregator.subclasses:
            matched_metrics = metric_set.intersection(aggregator_type.get_supported_metrics())
            if matched_metrics:
                self.aggregator_metrics[aggregator_type] = matched_metrics
                metric_set.difference_update(matched_metrics)

        if metric_set:
            raise ValueError(f'Metrics not supported by any aggregator: {metric_set}')

    def create_aggregators(self, dirpaths: Iterable[Path]) -> List[Aggregator]:
        """Creates a collection of aggregators for a set of benchmark runs."""
        aggregators: List[Aggregator] = []
        for aggregator_type, metrics in self.aggregator_metrics.items():
            aggregators.append(aggregator_type(metrics, dirpaths))

        return aggregators


def export_to_file(df: pd.DataFrame, path: Path) -> None:
    df.to_csv(path)


def import_from_file(path: Path) -> pd.DataFrame:
    return pd.read_csv(path, index_col=0)


def aggregate_metrics(args) -> None:
    factory = AggregatorFactory(args.metrics)
    aggregators: List[Aggregator] = factory.create_aggregators(args.input)

    results = []
    for aggregator in aggregators:
        results.append(aggregator.get_metrics_by_run())
        for metric, df_series in aggregator.generate_timeseries():
            export_to_file(df_series, args.output_path / f'timeseries_{metric}.data')
    df_results = pd.concat(results, axis='columns')

    results_path = args.output_path / 'metrics.data'
    # Append results to existent dataframe
    if results_path.exists():
        df = import_from_file(results_path)
        df_results = df.combine_first(df_results)
    export_to_file(df_results, results_path)

    df_stats = df_results.agg(args.functions).T
    df_stats.index.name = 'metric'
    export_to_file(df_stats, args.output_path / 'stats.data')


def aggregate_results(args) -> None:
    # Collect metadata and dataframes from all the directories
    metadatas: List[Dict] = []
    results: Dict[str, List[pd.DataFrame]] = dict()
    for dirpath in args.input:
        for path in dirpath.iterdir():
            if path.is_file():
                if path.suffix == '.data':
                    dataframe = import_from_file(path)
                    dataframe.reset_index(inplace=True)
                    results.setdefault(path.name, []).append(dataframe)
                elif path.name == 'metadata.json':
                    with open(path, 'r') as file:
                        metadatas.append(json.load(file))

    for filename, dataframes in results.items():
        for dataframe, metadata in zip(dataframes, metadatas):
            dataframe['test'] = metadata['name']
            for key, value in metadata['params'].items():
                dataframe[key] = value

        df_results = pd.concat(dataframes, ignore_index=True)
        df_results.index.name = 'index'
        export_to_file(df_results, args.output_path / filename)


def get_parser() -> argparse.ArgumentParser:
    """Returns a command-line argument parser for this program."""
    supported_metrics = Aggregator.get_all_supported_metrics()
    parser = argparse.ArgumentParser(
        description='Result aggregation CLI for LAMBKIN')

    common_parser = argparse.ArgumentParser(add_help=False)
    common_parser.add_argument(
        '-o', '--output-path', default=Path.cwd(), help=(
            'Full path to output directory. '
            'It defaults to the current working directory.'))
    common_parser.add_argument(
        'input', type=Path, nargs='+', help=(
            'Input directory paths containing results for each '
            'iteration to aggregate.'))

    subparsers = parser.add_subparsers()
    metrics_subparser = subparsers.add_parser('metrics', parents=[common_parser])
    metrics_subparser.add_argument(
        '-m', '--metrics', choices=supported_metrics, nargs='+',
        metavar='METRIC',
        default=supported_metrics,
        help='Metrics to aggregate.'
             'It defaults to all available metrics: %(choices)s.')
    metrics_subparser.add_argument(
        '-f', '--functions', type=str, nargs='+',
        default=['median', 'mean', 'std'],
        help='Functions to use for aggregating the data. '
             'It defaults to median, mean and std.')
    metrics_subparser.set_defaults(subcommand=aggregate_metrics)

    results_subparser = subparsers.add_parser('results', parents=[common_parser])
    results_subparser.set_defaults(subcommand=aggregate_results)
    return parser


def main(args) -> None:
    return args.subcommand(args)
