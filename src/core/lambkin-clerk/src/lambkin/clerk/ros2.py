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

"""This subpackage provides ROS 2 introspection APIs."""

import dataclasses
import importlib
import os
import re

import requests
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET


REP2000_URL = 'https://www.ros.org/reps/rep-2000.html'


@dataclasses.dataclass
class DistributionInfo:
    """Robot Operating System (ROS) distribution attributes."""

    name: str
    codename: str


def distribution_info() -> DistributionInfo:
    """Fetch ROS 2 distribution information."""
    if 'ROS_DISTRO' not in os.environ:
        raise RuntimeError('no ROS 2 distribution found in the environment')
    codename = os.environ['ROS_DISTRO']
    response = requests.get(REP2000_URL)
    html = BeautifulSoup(response.text, 'html.parser')
    distributions_div = html.body.find(
        'div', attrs={'id': 'platforms-by-distribution'})
    distro_div = distributions_div.find('div', attrs={
        'class': 'section', 'id': re.compile(f'{codename}.*')})
    name = ' '.join(distro_div.h2.text.split()[:2])
    return DistributionInfo(name, codename)


@dataclasses.dataclass
class PackageInfo:
    """Robot Operating System (ROS) package attributes."""

    name: str
    version: str


def package_info(name: str) -> PackageInfo:
    """
    Fetch ROS 2 package information.

    :param name: package name.
    """
    ament_index = importlib.import_module('ament_index_python')
    package_share_path = (
        ament_index.get_package_share_path(name, print_warning=False))
    package_xml_path = package_share_path / 'package.xml'
    if not package_xml_path.exists():
        raise RuntimeError(f'package.xml for {name} package does not exist')
    package_tree = ET.parse(package_xml_path)
    version_tag = package_tree.find('version')
    if version_tag is None or version_tag.text is None:
        raise RuntimeError(f'no version specified for {name} package')
    version = version_tag.text
    return PackageInfo(name, version)
