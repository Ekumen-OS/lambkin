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

"""Unit tests for the shell class in lambkin.core.shell."""

import pytest

from lambkin.core.shell import ShellProxy


@pytest.fixture
def shell():
    """Return a fresh ShellProxy instance for each test."""
    return ShellProxy(dry_run=True)


def test_simple_command(shell, capsys):
    """Test that a simple single-level command is printed correctly."""
    shell.ros2.launch("beluga.launch.xml")
    assert capsys.readouterr().out == "[CMD]: ros2 launch beluga.launch.xml\n"


def test_chained_command(shell, capsys):
    """Test that chained attribute access builds the command word by word."""
    shell.ros2.bag.play("my_bag")
    assert capsys.readouterr().out == "[CMD]: ros2 bag play my_bag\n"


def test_multiple_args(shell, capsys):
    """Test that multiple positional arguments are appended in order."""
    shell.ros2.bag.play("my_bag", "--clock", "-r", "1.0")
    assert capsys.readouterr().out == "[CMD]: ros2 bag play my_bag --clock -r 1.0\n"


def test_kwarg_becomes_flag(shell, capsys):
    """Test that keyword arguments are converted to --flag value pairs."""
    shell.evo_ape.bag2("output.mcap", save_results="out.zip")
    assert (
        capsys.readouterr().out
        == "[CMD]: evo_ape bag2 output.mcap --save-results out.zip\n"
    )


def test_arbitrary_tool(shell, capsys):
    """Test that any top-level tool name works without hardcoding."""
    shell.evo.traj("output.mcap")
    assert capsys.readouterr().out == "[CMD]: evo traj output.mcap\n"


def test_kwarg_true_is_standalone_flag(shell, capsys):
    """Test that a boolean True kwarg produces a standalone flag."""
    shell.ros2.bag.play("my_bag", clock=True)
    assert capsys.readouterr().out == "[CMD]: ros2 bag play my_bag --clock\n"


def test_kwarg_false_is_ignored(shell, capsys):
    """Test that a boolean False kwarg is ignored."""
    shell.ros2.bag.play("my_bag", clock=False)
    assert capsys.readouterr().out == "[CMD]: ros2 bag play my_bag\n"


def test_path_with_spaces(shell, capsys):
    """Test that positional args with spaces are correctly quoted."""
    shell.ros2.bag.play("/my path/to/bag.mcap")
    assert capsys.readouterr().out == "[CMD]: ros2 bag play '/my path/to/bag.mcap'\n"


def test_kwarg_multiple_underscores_converted(shell, capsys):
    """Test that multiple underscores in kwarg names are converted to dashes."""
    shell.evo_ape.bag2("output.mcap", save_all_results="out.zip")
    assert (
        capsys.readouterr().out
        == "[CMD]: evo_ape bag2 output.mcap --save-all-results out.zip\n"
    )
