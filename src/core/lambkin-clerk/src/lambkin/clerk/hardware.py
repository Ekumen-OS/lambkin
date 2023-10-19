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

"""This subpackage provides hardware introspection APIs."""

import dataclasses

import platform
import shutil
import subprocess
import json

import pint
import psutil

from typing import List


@dataclasses.dataclass
class CPUInfo:
    """CPU hardware attributes."""

    model: str
    cores: int
    threads: int
    frequency: pint.Quantity
    caches: List[str]

    @property
    def description(self) -> str:
        """Human readable CPU description."""
        description = f"{self.model} @ {self.frequency.to('MHz')} MHz x {self.cores} cores"
        if self.cores != self.threads:
            description += f' ({self.threads} threads)'
        return description


def cpu_info() -> CPUInfo:
    """Fetch CPU information."""
    model = platform.processor() + ' processor'
    if shutil.which('lscpu') is not None:
        process = subprocess.run(
            ['lscpu', '-J'], capture_output=True, text=True)
        data = json.loads(process.stdout)['lscpu']
        model = next((
            entry['data'] for entry in data
            if entry['field'].startswith('Model name')
        ), model)
    cores = psutil.cpu_count(logical=False)
    threads = psutil.cpu_count(logical=True)
    frequency = psutil.cpu_freq().max * pint.Unit('MHz')
    caches: List[str] = []
    if shutil.which('cache-info') is not None:
        process = subprocess.run(
            ['cache-info'], capture_output=True, text=True)
        caches.extend(
            line for line in process.stdout.splitlines()
            if line[:2] in ('L1', 'L2', 'L3'))
    return CPUInfo(model, cores, threads, frequency, caches)


@dataclasses.dataclass
class MemoryInfo:
    """Memory (RAM, swap) attributes."""

    ram_size: pint.Quantity
    swap_size: pint.Quantity


def memory_info() -> MemoryInfo:
    """Fetch system memory information."""
    return MemoryInfo(
        psutil.virtual_memory().total * pint.Unit('bytes'),
        psutil.swap_memory().total * pint.Unit('bytes'))
