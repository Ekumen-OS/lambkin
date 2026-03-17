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

"""Unit tests for the source function in lambkin.core.ctx."""

from pathlib import Path

from lambkin.core.ctx.source import Source


def test_source_path_is_set_correctly():
    """ctx.source.path reflects the path passed at construction."""
    source = Source(path=Path(__file__))
    assert source.path == Path(__file__)


def test_source_path_is_converted_to_path():
    """Source converts a string path to a Path object."""
    source = Source(path="/home/user/benchmarks/test.py")
    assert isinstance(source.path, Path)
    assert source.path == Path("/home/user/benchmarks/test.py")


def test_source_repr():
    """Source.__repr__ returns a human-readable string."""
    source = Source(path=Path("/home/user/benchmarks/test.py"))
    assert repr(source) == "Source(path=/home/user/benchmarks/test.py)"
