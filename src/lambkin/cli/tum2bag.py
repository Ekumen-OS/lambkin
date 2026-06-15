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

"""CLI entrypoint for the ``lambkin-tum2bag`` command.

Requires ``rosbag2_py`` and ``rclpy`` to be available in the environment
(i.e. a sourced ROS 2 workspace).
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

import click

from lambkin.sdk_options import SDK_OPTIONS
from lambkin.utils.tum2bag import tum2bag


@click.command(
    params=[next(opt for opt in SDK_OPTIONS if "--log-level" in opt.opts)],
)
@click.option(
    "--tum",
    "tum_path",
    type=click.Path(exists=True, path_type=Path),
    required=True,
    help="Path to the TUM ground truth file.",
)
@click.option(
    "--input",
    "input_bag",
    type=click.Path(exists=True, path_type=Path),
    required=True,
    help="Path to the input ROS 2 bag (directory).",
)
@click.option(
    "--output",
    "output_bag",
    type=click.Path(path_type=Path),
    required=True,
    help="Path for the output MCAP bag (directory, must not exist).",
)
@click.option(
    "--topic",
    default="/ground_truth",
    show_default=True,
    help="Topic name for the ground truth poses.",
)
def main(
    tum_path: Path,
    input_bag: Path,
    output_bag: Path,
    topic: str,
    log_level: str,
) -> None:
    r"""Merge a TUM ground truth file into a ROS 2 bag.

    Reads all messages from INPUT, appends the poses from TUM as a new
    topic, and writes everything sorted by timestamp to OUTPUT in MCAP
    format.

    Example::

        lambkin-tum2bag \\
            --tum ground_truth.tum \\
            --input raw_bag/ \\
            --output merged_bag/ \\
            --topic /ground_truth
    """
    logging.basicConfig(level=getattr(logging, log_level.upper()))
    try:
        tum2bag(
            tum_path=tum_path,
            input_bag=input_bag,
            output_bag=output_bag,
            topic=topic,
        )
    except (FileNotFoundError, FileExistsError, ValueError) as exc:
        click.echo(f"Error: {exc}", err=True)
        sys.exit(1)
