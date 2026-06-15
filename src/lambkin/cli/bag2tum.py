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

"""CLI entrypoint for the ``lambkin-bag2tum`` command.

Requires ``rosbag2_py`` and ``rclpy`` to be available in the environment
(i.e. a sourced ROS 2 workspace).
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

import click

from lambkin.sdk_options import SDK_OPTIONS
from lambkin.utils.bag2tum import bag2tum


@click.command(
    params=[next(opt for opt in SDK_OPTIONS if "--log-level" in opt.opts)],
)
@click.option(
    "--input",
    "input_bag",
    type=click.Path(exists=True, path_type=Path),
    required=True,
    help="Path to the input ROS 2 bag (directory).",
)
@click.option(
    "--topic",
    required=True,
    help="Topic name to extract (must be geometry_msgs/msg/PoseStamped).",
)
@click.option(
    "--output",
    "output_tum",
    type=click.Path(path_type=Path),
    required=True,
    help="Path to the output TUM file.",
)
def main(
    input_bag: Path,
    topic: str,
    output_tum: Path,
    log_level: str,
) -> None:
    r"""Extract a pose topic from a ROS 2 bag into a TUM file.

    Reads ``geometry_msgs/msg/PoseStamped`` messages from TOPIC in INPUT
    and writes them to OUTPUT in TUM format::

        timestamp tx ty tz qx qy qz qw

    Example::

        lambkin-bag2tum \\
            --input merged_bag/ \\
            --topic /ground_truth \\
            --output ground_truth.tum
    """
    logging.basicConfig(level=getattr(logging, log_level.upper()))
    try:
        bag2tum(
            input_bag=input_bag,
            topic=topic,
            output_tum=output_tum,
        )
    except (FileNotFoundError, FileExistsError, ValueError) as exc:
        click.echo(f"Error: {exc}", err=True)
        sys.exit(1)
