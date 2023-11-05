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

import functools

import json
import pytest
import yaml

import numpy as np

from importlib import import_module

from lambkin.shepherd.data import access
from lambkin.shepherd.data.ros2 import messages
from lambkin.shepherd.data.ros2 import occupancy_grids

from utilities import make_directory_tree
from utilities import module_missing


@pytest.fixture
def testing_data_path(tmp_path_factory):
    path = tmp_path_factory.mktemp('data')

    def make_ros_data(path, iteration):
        path.mkdir()

        rosbag2 = import_module('rosbag2_py')
        writer = rosbag2.SequentialWriter()
        storage_options = rosbag2.StorageOptions(str(path / 'output_bag'))
        converter_options = rosbag2.ConverterOptions('', '')
        writer.open(storage_options, converter_options)
        writer.create_topic(rosbag2.TopicMetadata(
            '/scale', 'std_msgs/msg/Int32', ''
        ))
        std_msgs = import_module('std_msgs.msg')
        message = std_msgs.Int32(data=10**iteration)
        serialization = import_module('rclpy.serialization')
        data = serialization.serialize_message(message)
        writer.write('/scale', data, 0)
        del writer

        Image = import_module('PIL.Image')
        map_image = np.full((200, 200), 128, dtype=np.uint8)
        with open(path / 'map.pgm', 'wb') as f:
            Image.fromarray(map_image, mode='L').save(f)

        map_metadata = {
            'image': 'map.pgm',
            'resolution': 0.1,
            'origin': [0., 0., 0.],
            'occupied_thresh': 0.7,
            'free_thresh': 0.3,
            'negate': 0
        }
        with open(path / 'map.yaml', 'w') as f:
            f.write(yaml.safe_dump(map_metadata))

    return make_directory_tree({
        'cases': {
            'foo': {
                'metadata.json': json.dumps({'name': 'Foo'}),
                'variations': [{
                    'metadata.json': json.dumps({
                        'parameters': {'scenario': 'nominal'}
                    }),
                    'iterations': [
                        functools.partial(
                            make_ros_data, iteration=i
                        ) for i in range(3)
                    ]
                }]
            }
        }
    }, path)


@pytest.fixture(autouse=True)
def testing_context(testing_data_path):
    with access.benchmark(testing_data_path):
        yield


@pytest.mark.skipif(
    module_missing('rosbag2_py'),
    reason='some packages are not installed')
def test_messages():
    for metadata, (topic_name, message, timestamp) in messages('/scale'):
        scale = 10**metadata['iteration']['index']
        assert topic_name == '/scale'
        assert message.data == scale
        assert timestamp == 0


@pytest.mark.skipif(
    module_missing('rosbag2_py') or module_missing('PIL'),
    reason='some package are not installed')
def test_occupancy_grids():
    for metadata, grid in occupancy_grids():
        assert grid.info.resolution == 0.1
        assert grid.info.height == 200
        assert grid.info.width == 200
        np.testing.assert_equal(np.array(grid.data), 50)
