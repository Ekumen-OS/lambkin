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

"""Tests for lambkin.utils.tum2bag.

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
    """Install minimal stubs."""
    rosbag2_py = ModuleType("rosbag2_py")

    class _TopicMetadata:
        def __init__(self, *, id, name, type, serialization_format, **_):
            self.id = id
            self.name = name
            self.type = type
            self.serialization_format = serialization_format

    class _StorageOptions:
        def __init__(self, uri, storage_id):
            self.uri = uri
            self.storage_id = storage_id

    class _ConverterOptions:
        def __init__(self, input_serialization_format, output_serialization_format):
            pass

    rosbag2_py.TopicMetadata = _TopicMetadata
    rosbag2_py.StorageOptions = _StorageOptions
    rosbag2_py.ConverterOptions = _ConverterOptions
    rosbag2_py.StorageFilter = MagicMock
    rosbag2_py.SequentialReader = MagicMock
    rosbag2_py.SequentialWriter = MagicMock
    sys.modules["rosbag2_py"] = rosbag2_py

    rclpy = ModuleType("rclpy")
    rclpy_ser = ModuleType("rclpy.serialization")
    rclpy_ser.serialize_message = lambda msg: b"serialized"
    rclpy.serialization = rclpy_ser
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.serialization"] = rclpy_ser

    bi = ModuleType("builtin_interfaces")
    bi_msg = ModuleType("builtin_interfaces.msg")

    class _Time:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    bi_msg.Time = _Time
    bi.msg = bi_msg
    sys.modules["builtin_interfaces"] = bi
    sys.modules["builtin_interfaces.msg"] = bi_msg

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

import rosbag2_py  # noqa: E402

from lambkin.utils.tum2bag import _parse_tum, tum2bag  # noqa: E402


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


def test_parse_tum_valid(tmp_path: Path) -> None:
    """Parses a valid TUM file and returns poses with nanosecond timestamps."""
    tum = tmp_path / "gt.tum"
    tum.write_text(
        "# comment\n"
        "1.0 0.1 0.2 0.3 0.0 0.0 0.0 1.0\n"
        "2.5 0.4 0.5 0.6 0.0 0.0 0.707 0.707\n"
    )
    poses = _parse_tum(tum)
    assert len(poses) == 2
    assert poses[0][0] == 1_000_000_000
    assert poses[1][0] == 2_500_000_000


def test_parse_tum_sorted(tmp_path: Path) -> None:
    """Returns poses sorted by timestamp regardless of input order."""
    tum = tmp_path / "gt.tum"
    tum.write_text(
        "3.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n"
        "1.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n"
        "2.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n"
    )
    poses = _parse_tum(tum)
    timestamps = [p[0] for p in poses]
    assert timestamps == sorted(timestamps)


def test_parse_tum_empty_lines_and_comments(tmp_path: Path) -> None:
    """Ignores empty lines and comment lines starting with '#'."""
    tum = tmp_path / "gt.tum"
    tum.write_text("\n# header\n1.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n\n")
    poses = _parse_tum(tum)
    assert len(poses) == 1


def test_parse_tum_wrong_field_count(tmp_path: Path) -> None:
    """Raises ValueError when a data line does not have exactly 8 fields."""
    tum = tmp_path / "bad.tum"
    tum.write_text("1.0 0.0 0.0\n")
    with pytest.raises(ValueError, match="expected 8 fields"):
        _parse_tum(tum)


def test_tum2bag_raises_if_tum_missing(tmp_path: Path) -> None:
    """Raises FileNotFoundError when the TUM file does not exist."""
    with pytest.raises(FileNotFoundError, match="TUM file not found"):
        tum2bag(
            tum_path=tmp_path / "missing.tum",
            input_bag=tmp_path / "bag",
            output_bag=tmp_path / "out",
        )


def test_tum2bag_raises_if_input_bag_missing(tmp_path: Path) -> None:
    """Raises FileNotFoundError when the input bag directory does not exist."""
    tum = tmp_path / "gt.tum"
    tum.write_text("1.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n")
    with pytest.raises(FileNotFoundError, match="Input bag not found"):
        tum2bag(
            tum_path=tum,
            input_bag=tmp_path / "missing_bag",
            output_bag=tmp_path / "out",
        )


def test_tum2bag_raises_if_output_exists(tmp_path: Path) -> None:
    """Raises FileExistsError when the output bag directory already exists."""
    tum = tmp_path / "gt.tum"
    tum.write_text("1.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n")
    input_bag = tmp_path / "bag"
    input_bag.mkdir()
    output_bag = tmp_path / "out"
    output_bag.mkdir()
    with pytest.raises(FileExistsError, match="already exists"):
        tum2bag(
            tum_path=tum,
            input_bag=input_bag,
            output_bag=output_bag,
        )


def test_tum2bag_writes_merged_messages(tmp_path: Path) -> None:
    """Writes all input bag messages and ground truth poses in chronological order."""
    tum = tmp_path / "gt.tum"
    tum.write_text(
        "1.0 0.1 0.2 0.3 0.0 0.0 0.0 1.0\n3.0 0.4 0.5 0.6 0.0 0.0 0.707 0.707\n"
    )
    input_bag = tmp_path / "bag"
    input_bag.mkdir()
    output_bag = tmp_path / "out"

    existing_topic = MagicMock()
    existing_topic.name = "/scan"
    existing_topic.type = "sensor_msgs/msg/LaserScan"

    reader = _make_reader_mock(
        [existing_topic],
        [("/scan", b"scan_data", 2_000_000_000)],
    )
    writer = MagicMock()

    rosbag2_py.SequentialReader = MagicMock(return_value=reader)
    rosbag2_py.SequentialWriter = MagicMock(return_value=writer)

    tum2bag(tum_path=tum, input_bag=input_bag, output_bag=output_bag)

    assert writer.create_topic.call_count == 2
    written_timestamps = [c.args[2] for c in writer.write.call_args_list]
    assert written_timestamps == sorted(written_timestamps)
    assert len(written_timestamps) == 3


def test_tum2bag_custom_builder(tmp_path: Path) -> None:
    """Uses a custom message_builder.

    Verifies the builder's returned type is registered in the bag metadata.
    """
    tum = tmp_path / "gt.tum"
    tum.write_text("1.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n")
    input_bag = tmp_path / "bag"
    input_bag.mkdir()
    output_bag = tmp_path / "out"

    builder_calls = []

    def my_builder(t_ns, x, y, z, qx, qy, qz, qw):
        """Build a custom message from TUM pose fields."""
        builder_calls.append((t_ns, x, y, z, qx, qy, qz, qw))
        return MagicMock(), "my_pkg/msg/MyPose"

    reader = _make_reader_mock([], [])
    writer = MagicMock()

    rosbag2_py.SequentialReader = MagicMock(return_value=reader)
    rosbag2_py.SequentialWriter = MagicMock(return_value=writer)

    tum2bag(
        tum_path=tum,
        input_bag=input_bag,
        output_bag=output_bag,
        message_builder=my_builder,
    )

    assert len(builder_calls) == 1
    assert builder_calls[0][0] == 1_000_000_000
    created_topics = [c.args[0] for c in writer.create_topic.call_args_list]
    assert any(t.type == "my_pkg/msg/MyPose" for t in created_topics)
