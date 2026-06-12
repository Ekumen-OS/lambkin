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

"""Extract a pose topic from a ROS 2 bag into a TUM trajectory file.

Requires ``rosbag2_py`` and ``rclpy`` to be available in the environment
(i.e. a sourced ROS 2 workspace).

Typical usage as a library (e.g. inside a ``@nominal.output`` hook)::

    from lambkin.utils.bag2tum import bag2tum

    @nominal.output
    def export(ctx):
        bag2tum(
            input_bag=ctx.workdir / "output.mcap",
            topic="/ground_truth",
            output_tum=ctx.workdir / "ground_truth.tum",
        )
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

import click

from lambkin.sdk_options import SDK_OPTIONS

logger = logging.getLogger(__name__)


def bag2tum(
    input_bag: Path | str,
    topic: str,
    output_tum: Path | str,
) -> None:
    """Extract a pose topic from a ROS 2 bag into a TUM trajectory file.

    Reads ``geometry_msgs/msg/PoseStamped`` messages from ``topic`` in
    ``input_bag`` and writes them to ``output_tum`` in TUM format::

        timestamp tx ty tz qx qy qz qw

    Timestamps are written as seconds with nanosecond precision.

    Args:
        input_bag: Path to the input ROS 2 bag (directory).
        topic: Topic name to extract. Must publish
            ``geometry_msgs/msg/PoseStamped``.
        output_tum: Path to the output TUM file.

    Raises:
        FileNotFoundError: If ``input_bag`` does not exist.
        FileExistsError: If ``output_tum`` already exists.
        ValueError: If ``topic`` is not found in the bag or its type is not
            ``geometry_msgs/msg/PoseStamped``.
    """
    import rclpy.serialization
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped

    input_bag = Path(input_bag)
    output_tum = Path(output_tum)

    if not input_bag.exists():
        raise FileNotFoundError(f"Input bag not found: {input_bag}")
    if output_tum.exists():
        raise FileExistsError(f"Output TUM file already exists: {output_tum}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_bag), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {tm.name: tm.type for tm in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(
            f"Topic '{topic}' not found in bag. Available topics: {sorted(topic_types)}"
        )
    expected_type = "geometry_msgs/msg/PoseStamped"
    if topic_types[topic] != expected_type:
        raise ValueError(
            f"Topic '{topic}' has type '{topic_types[topic]}', "
            f"expected '{expected_type}'"
        )

    filter_ = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_)

    poses = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg: PoseStamped = rclpy.serialization.deserialize_message(data, PoseStamped)
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        poses.append((t_s, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    del reader

    if not poses:
        raise ValueError(f"No messages found on topic '{topic}'")

    logger.debug("Extracted %d poses from topic '%s'", len(poses), topic)

    output_tum.parent.mkdir(parents=True, exist_ok=True)
    with open(output_tum, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for t_s, tx, ty, tz, qx, qy, qz, qw in poses:
            f.write(
                f"{t_s:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )

    logger.info("Written %d poses to %s", len(poses), output_tum)


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
