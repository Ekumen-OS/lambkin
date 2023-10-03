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

"""This subpackage provides operating system introspection APIs."""

import dataclasses
import platform
import importlib


@dataclasses.dataclass
class DistributionInfo:
    """Operating system distribution attributes."""

    name: str
    codename: str
    description: str
    architecture: str
    kernel_version: str


def distribution_info() -> DistributionInfo:
    """Fetch OS distribution information."""
    lsb_release = importlib.import_module('lsb_release')
    info = lsb_release.get_distro_information()
    return DistributionInfo(
        name=info['ID'],
        codename=info['CODENAME'],
        description=info['DESCRIPTION'],
        architecture=platform.machine(),
        kernel_version=platform.release())
