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

"""Merge a TUM ground truth file into a ROS 2 bag.

Requires ``rosbag2_py`` and ``rclpy`` to be available in the environment
(i.e. a sourced ROS 2 workspace).

Typical usage as a library (e.g. inside a ``@nominal.input`` hook)::

    from lambkin.utils.tum2bag import tum2bag

    @nominal.input
    def dataset(ctx):
        tum2bag(
            tum_path=ctx.source.path.parent / "ground_truth.tum",
            input_bag=ctx.source.path.parent / "raw.mcap",
            output_bag=ctx.workdir / "input.mcap",
        )
        return ctx.workdir / "input.mcap"
"""

from __future__ import annotations

import logging
import sys
from collections.abc import Callable
from pathlib import Path

import click

from lambkin.sdk_options import SDK_OPTIONS

logger = logging.getLogger(__name__)


def _default_message_builder(
    t_ns: int,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[object, str]:
    """Build a ``geometry_msgs/msg/PoseStamped`` from a TUM pose.

    This is the default ``message_builder`` for :func:`tum2bag`. It produces
    a ``PoseStamped`` message with the frame set to ``map`` and the timestamp
    derived from the TUM timestamp converted to nanoseconds.

    Args:
        t_ns: Timestamp in nanoseconds.
        x: Translation x (metres).
        y: Translation y (metres).
        z: Translation z (metres).
        qx: Rotation quaternion x.
        qy: Rotation quaternion y.
        qz: Rotation quaternion z.
        qw: Rotation quaternion w.

    Returns:
        A tuple of (serializable ROS 2 message, message type string).
    """
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = Time(
        sec=t_ns // 1_000_000_000,
        nanosec=t_ns % 1_000_000_000,
    )
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg, "geometry_msgs/msg/PoseStamped"


def _parse_tum(
    tum_path: Path,
) -> list[tuple[int, float, float, float, float, float, float, float]]:
    """Parse a TUM trajectory file.

    Each line has the format::

        timestamp tx ty tz qx qy qz qw

    Lines starting with ``#`` are treated as comments and skipped.

    Args:
        tum_path: Path to the TUM file.

    Returns:
        List of tuples ``(timestamp_ns, tx, ty, tz, qx, qy, qz, qw)``
        sorted by timestamp.

    Raises:
        ValueError: If a line cannot be parsed.
    """
    poses = []
    with open(tum_path) as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                raise ValueError(
                    f"{tum_path}:{lineno}: expected 8 fields, got {len(parts)}"
                )
            t_s, tx, ty, tz, qx, qy, qz, qw = (float(p) for p in parts)
            t_ns = int(round(t_s * 1e9))
            poses.append((t_ns, tx, ty, tz, qx, qy, qz, qw))
    poses.sort(key=lambda p: p[0])
    logger.debug("Parsed %d poses from %s", len(poses), tum_path)
    return poses


def tum2bag(
    tum_path: Path | str,
    input_bag: Path | str,
    output_bag: Path | str,
    topic: str = "/ground_truth",
    message_builder: Callable[
        [int, float, float, float, float, float, float, float],
        tuple[object, str],
    ]
    | None = None,
) -> None:
    """Merge a TUM ground truth file into a ROS 2 bag.

    Reads all messages from ``input_bag``, appends the poses from
    ``tum_path`` as a new topic, and writes everything sorted by timestamp
    to ``output_bag`` in MCAP format.

    The serialized message type is determined by ``message_builder``.
    The default builder produces ``geometry_msgs/msg/PoseStamped`` messages.
    To use a custom message type, supply a callable with the signature::

        def my_builder(
            t_ns: int,
            x: float, y: float, z: float,
            qx: float, qy: float, qz: float, qw: float,
        ) -> tuple[object, str]:
            ...
            return my_msg, "my_package/msg/MyType"

    The output is always written in MCAP format regardless of the format
    of ``input_bag``.

    Args:
        tum_path: Path to the TUM trajectory file.
        input_bag: Path to the input ROS 2 bag (directory).
        output_bag: Path for the output MCAP bag (directory, must not exist).
        topic: Topic name for the ground truth poses.
        message_builder: Callable that produces ``(msg, type_string)`` from
            a TUM pose. Defaults to a ``PoseStamped`` builder.

    Raises:
        FileNotFoundError: If ``tum_path`` or ``input_bag`` do not exist.
        FileExistsError: If ``output_bag`` already exists.
    """
    import rosbag2_py
    from rclpy.serialization import serialize_message

    tum_path = Path(tum_path)
    input_bag = Path(input_bag)
    output_bag = Path(output_bag)

    if not tum_path.exists():
        raise FileNotFoundError(f"TUM file not found: {tum_path}")
    if not input_bag.exists():
        raise FileNotFoundError(f"Input bag not found: {input_bag}")
    if output_bag.exists():
        raise FileExistsError(f"Output bag already exists: {output_bag}")

    if message_builder is None:
        message_builder = _default_message_builder

    poses = _parse_tum(tum_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_bag), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = reader.get_all_topics_and_types()
    messages = []
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        messages.append((timestamp, topic_name, data))
    del reader

    logger.debug(
        "Read %d messages from %s (%d topics)",
        len(messages),
        input_bag,
        len(topic_types),
    )

    gt_msgs = []
    gt_type_str = None
    for t_ns, tx, ty, tz, qx, qy, qz, qw in poses:
        msg, type_str = message_builder(t_ns, tx, ty, tz, qx, qy, qz, qw)
        gt_type_str = type_str
        gt_msgs.append((t_ns, topic, serialize_message(msg)))

    all_messages = messages + gt_msgs
    all_messages.sort(key=lambda m: m[0])

    gt_topic_meta = rosbag2_py.TopicMetadata(
        id=len(topic_types),
        name=topic,
        type=gt_type_str,
        serialization_format="cdr",
    )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    for tm in topic_types:
        writer.create_topic(tm)
    writer.create_topic(gt_topic_meta)

    for timestamp, topic_name, data in all_messages:
        writer.write(topic_name, data, timestamp)
    del writer

    logger.info(
        "Written %d messages (%d ground truth poses) to %s",
        len(all_messages),
        len(gt_msgs),
        output_bag,
    )


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
