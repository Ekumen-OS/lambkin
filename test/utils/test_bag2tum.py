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

"""Tests for lambkin.utils.bag2tum.

rosbag2_py and rclpy are ROS 2 packages unavailable in the test environment.
All tests mock these imports to test the SDK logic independently.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest


def _make_ros_stubs() -> None:
    """Install minimal stubs for rosbag2_py, rclpy, geometry_msgs.

    Registers stub modules in ``sys.modules`` so that imports inside
    ``lambkin.utils.bag2tum`` resolve without a ROS 2 installation.
    """
    rosbag2_py = ModuleType("rosbag2_py")

    class _StorageOptions:
        def __init__(self, uri, storage_id):
            pass

    class _ConverterOptions:
        def __init__(self, input_serialization_format, output_serialization_format):
            pass

    class _StorageFilter:
        def __init__(self, topics):
            self.topics = topics

    rosbag2_py.StorageOptions = _StorageOptions
    rosbag2_py.ConverterOptions = _ConverterOptions
    rosbag2_py.StorageFilter = _StorageFilter
    rosbag2_py.SequentialReader = MagicMock
    sys.modules["rosbag2_py"] = rosbag2_py

    rclpy = ModuleType("rclpy")
    rclpy_ser = ModuleType("rclpy.serialization")
    rclpy_ser.deserialize_message = MagicMock(return_value=None)
    rclpy.serialization = rclpy_ser
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.serialization"] = rclpy_ser

    geo = ModuleType("geometry_msgs")
    geo_msg = ModuleType("geometry_msgs.msg")

    class _Vec3:
        x = y = z = 0.0

    class _Quaternion:
        x = y = z = 0.0
        w = 1.0

    class _Pose:
        def __init__(self):
            self.position = _Vec3()
            self.orientation = _Quaternion()

    class _Time:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    class _Header:
        frame_id = ""

        def __init__(self):
            self.stamp = _Time()

    class _PoseStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    geo_msg.PoseStamped = _PoseStamped
    geo.msg = geo_msg
    sys.modules["geometry_msgs"] = geo
    sys.modules["geometry_msgs.msg"] = geo_msg


_make_ros_stubs()

import rclpy.serialization  # noqa: E402
import rosbag2_py  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402

from lambkin.utils.bag2tum import bag2tum  # noqa: E402


@pytest.fixture(autouse=True)
def _reset_ros_stubs() -> None:
    """Reset mutable stub state before each test.

    ``test_tum2bag.py`` runs first and overwrites ``sys.modules["rosbag2_py"]``
    with its own stub. This fixture ensures ``SequentialReader`` and
    ``deserialize_message`` are reset to safe defaults before every test so
    that cross-module stub contamination does not cause false failures.
    """
    rosbag2_py.SequentialReader = MagicMock
    rclpy.serialization.deserialize_message = MagicMock(return_value=None)


def _make_reader_mock(topic_types, messages):
    """Return a configured SequentialReader mock.

    Args:
        topic_types: List of topic metadata objects returned by
            ``get_all_topics_and_types``.
        messages: List of ``(topic, data, timestamp)`` tuples returned
            sequentially by ``read_next``.

    Returns:
        A ``MagicMock`` configured to behave as a ``SequentialReader``.
    """
    reader = MagicMock()
    reader.get_all_topics_and_types.return_value = topic_types
    reader.has_next.side_effect = [True] * len(messages) + [False]
    reader.read_next.side_effect = messages
    return reader


def _make_pose_msg(t_s: float, x: float, y: float, z: float) -> PoseStamped:
    """Return a stub PoseStamped with the given timestamp and position.

    Args:
        t_s: Timestamp in seconds.
        x: Position x (metres).
        y: Position y (metres).
        z: Position z (metres).

    Returns:
        A stub ``PoseStamped`` instance with header stamp and position set.
    """
    msg = PoseStamped()
    sec = int(t_s)
    nanosec = int((t_s - sec) * 1e9)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    return msg


def test_bag2tum_raises_if_bag_missing(tmp_path: Path) -> None:
    """Raises FileNotFoundError when the input bag does not exist."""
    with pytest.raises(FileNotFoundError, match="Input bag not found"):
        bag2tum(
            input_bag=tmp_path / "missing",
            topic="/gt",
            output_tum=tmp_path / "out.tum",
        )


def test_bag2tum_raises_if_output_exists(tmp_path: Path) -> None:
    """Raises FileExistsError when the output TUM file already exists."""
    input_bag = tmp_path / "bag"
    input_bag.mkdir()
    output_tum = tmp_path / "out.tum"
    output_tum.write_text("")

    existing_topic = MagicMock()
    existing_topic.name = "/gt"
    existing_topic.type = "geometry_msgs/msg/PoseStamped"

    reader = _make_reader_mock([existing_topic], [])
    rosbag2_py.SequentialReader = MagicMock(return_value=reader)

    with pytest.raises(FileExistsError, match="already exists"):
        bag2tum(input_bag=input_bag, topic="/gt", output_tum=output_tum)


def test_bag2tum_raises_if_topic_missing(tmp_path: Path) -> None:
    """Raises ValueError when the requested topic is not in the bag."""
    input_bag = tmp_path / "bag"
    input_bag.mkdir()

    reader = _make_reader_mock([], [])
    rosbag2_py.SequentialReader = MagicMock(return_value=reader)

    with pytest.raises(ValueError, match="not found in bag"):
        bag2tum(input_bag=input_bag, topic="/gt", output_tum=tmp_path / "out.tum")


def test_bag2tum_raises_if_wrong_type(tmp_path: Path) -> None:
    """Raises ValueError when the topic type is not PoseStamped."""
    input_bag = tmp_path / "bag"
    input_bag.mkdir()

    wrong_topic = MagicMock()
    wrong_topic.name = "/gt"
    wrong_topic.type = "nav_msgs/msg/Odometry"

    reader = _make_reader_mock([wrong_topic], [])
    rosbag2_py.SequentialReader = MagicMock(return_value=reader)

    with pytest.raises(ValueError, match="expected 'geometry_msgs/msg/PoseStamped'"):
        bag2tum(input_bag=input_bag, topic="/gt", output_tum=tmp_path / "out.tum")


def test_bag2tum_raises_if_no_messages(tmp_path: Path) -> None:
    """Raises ValueError when the topic exists but has no messages."""
    input_bag = tmp_path / "bag"
    input_bag.mkdir()

    gt_topic = MagicMock()
    gt_topic.name = "/gt"
    gt_topic.type = "geometry_msgs/msg/PoseStamped"

    reader = _make_reader_mock([gt_topic], [])
    rosbag2_py.SequentialReader = MagicMock(return_value=reader)

    with pytest.raises(ValueError, match="No messages found"):
        bag2tum(input_bag=input_bag, topic="/gt", output_tum=tmp_path / "out.tum")


def test_bag2tum_writes_tum_file(tmp_path: Path) -> None:
    """Writes a valid TUM file with correct timestamps and positions."""
    input_bag = tmp_path / "bag"
    input_bag.mkdir()
    output_tum = tmp_path / "out.tum"

    gt_topic = MagicMock()
    gt_topic.name = "/gt"
    gt_topic.type = "geometry_msgs/msg/PoseStamped"

    msg1 = _make_pose_msg(1.0, 0.1, 0.2, 0.3)
    msg2 = _make_pose_msg(2.5, 0.4, 0.5, 0.6)

    reader = _make_reader_mock(
        [gt_topic],
        [("/gt", b"d1", 1_000_000_000), ("/gt", b"d2", 2_500_000_000)],
    )

    rosbag2_py.SequentialReader = MagicMock(return_value=reader)
    rclpy.serialization.deserialize_message = MagicMock(side_effect=[msg1, msg2])

    bag2tum(input_bag=input_bag, topic="/gt", output_tum=output_tum)

    lines = [
        line for line in output_tum.read_text().splitlines() if not line.startswith("#")
    ]
    assert len(lines) == 2
    fields = lines[0].split()
    assert len(fields) == 8
    assert abs(float(fields[0]) - 1.0) < 1e-6
    assert abs(float(fields[1]) - 0.1) < 1e-6
