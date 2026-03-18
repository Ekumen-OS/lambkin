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

"""Benchmark source information.

This module defines the 'Source' class, which describes the benchmark script
being executed.
"""

from pathlib import Path


class Source:
    """Describes the benchmark source file.

    Attributes:
    ----------
    path:
        Path to the source file of the benchmark.
    """

    def __init__(self, path: Path | str) -> None:
        """Initialize a Source with the path to the benchmark script.

        Parameters
        ----------
        path : Path or str
            Path to the source file of the benchmark.
        """
        self.path = Path(path)

    def __repr__(self) -> str:
        """Return a human-readable summary of the Source state."""
        return f"Source(path={self.path})"
