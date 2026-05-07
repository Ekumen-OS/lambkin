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

import subprocess

import pytest

from lambkin.core.shell import CommandError, ShellProxy


@pytest.fixture
def shell():
    """Return a ShellProxy in real execution mode."""
    return ShellProxy(dry_run=False)


@pytest.fixture
def dry_shell():
    """Return a ShellProxy in dry-run mode."""
    return ShellProxy(dry_run=True)


def test_simple_command(dry_shell, capsys):
    """A single-level command is printed correctly."""
    dry_shell.echo("hello")
    assert capsys.readouterr().out == "[DRY RUN] echo hello\n"


def test_chained_command(dry_shell, capsys):
    """Chained attribute access builds the command word by word."""
    dry_shell.ros2.bag.play("my_bag")
    assert capsys.readouterr().out == "[DRY RUN] ros2 bag play my_bag\n"


def test_multiple_positional_args(dry_shell, capsys):
    """Multiple positional arguments are appended in order."""
    dry_shell.ros2.bag.play("my_bag", "--clock", "-r", "1.0")
    assert capsys.readouterr().out == "[DRY RUN] ros2 bag play my_bag --clock -r 1.0\n"


def test_kwarg_becomes_flag(dry_shell, capsys):
    """Keyword arguments are converted to --flag value pairs."""
    dry_shell.evo_ape.bag2("output.mcap", save_results="out.zip")
    assert (
        capsys.readouterr().out
        == "[DRY RUN] evo_ape bag2 output.mcap --save-results out.zip\n"
    )


def test_kwarg_true_is_standalone_flag(dry_shell, capsys):
    """A boolean True kwarg produces a standalone flag."""
    dry_shell.ros2.bag.play("my_bag", clock=True)
    assert capsys.readouterr().out == "[DRY RUN] ros2 bag play my_bag --clock\n"


def test_kwarg_false_is_omitted(dry_shell, capsys):
    """A boolean False kwarg is omitted entirely."""
    dry_shell.ros2.bag.play("my_bag", clock=False)
    assert capsys.readouterr().out == "[DRY RUN] ros2 bag play my_bag\n"


def test_kwarg_multiple_underscores_converted(dry_shell, capsys):
    """Multiple underscores in kwarg names are all converted to dashes."""
    dry_shell.evo_ape.bag2("output.mcap", save_all_results="out.zip")
    assert (
        capsys.readouterr().out
        == "[DRY RUN] evo_ape bag2 output.mcap --save-all-results out.zip\n"
    )


def test_path_with_spaces(dry_shell, capsys):
    """Positional args with spaces are quoted correctly."""
    dry_shell.ros2.bag.play("/my path/to/bag.mcap")
    assert capsys.readouterr().out == "[DRY RUN] ros2 bag play '/my path/to/bag.mcap'\n"


def test_arbitrary_tool(dry_shell, capsys):
    """Any top-level tool name works without hardcoding."""
    dry_shell.evo.traj("output.mcap")
    assert capsys.readouterr().out == "[DRY RUN] evo traj output.mcap\n"


def test_successful_command(shell):
    """A successful command returns a CompletedProcess with returncode 0."""
    result = shell.echo("hello")
    assert result is not None
    assert result.returncode == 0


def test_returns_completed_process(shell):
    """__call__ returns a CompletedProcess instance on success."""
    result = shell.true()
    assert isinstance(result, subprocess.CompletedProcess)


def test_not_found_raises_command_error(shell):
    """A command that does not exist raises CommandError with a helpful message."""
    with pytest.raises(CommandError) as exc_info:
        shell.this_command_does_not_exist_at_all()
    assert exc_info.value.returncode is None
    assert "not found" in str(exc_info.value).lower()


def test_permission_error_raises_command_error(shell, tmp_path):
    """A non-executable file raises CommandError with a helpful message."""
    script = tmp_path / "script.sh"
    script.write_text("#!/bin/bash\necho hello\n")
    script.chmod(0o644)
    with pytest.raises(CommandError) as exc_info:
        shell.__getattr__(str(script))()
    assert exc_info.value.returncode is None
    assert "permission" in str(exc_info.value).lower()


def test_command_error_message_contains_command(shell):
    """CommandError message contains the command that failed."""
    with pytest.raises(CommandError) as exc_info:
        shell.false()
    assert "false" in str(exc_info.value)


def test_not_found_message_contains_command_name(shell):
    """CommandError message for not found contains the command name."""
    with pytest.raises(CommandError) as exc_info:
        shell.this_command_does_not_exist_at_all()
    assert "this_command_does_not_exist_at_all" in str(exc_info.value)


def test_command_error_carries_returncode(shell):
    """CommandError carries the non-zero return code of the failed command."""
    with pytest.raises(CommandError) as exc_info:
        shell.false()
    assert exc_info.value.returncode == 1


def test_command_error_carries_argv(shell):
    """CommandError carries the argv list of the failed command."""
    with pytest.raises(CommandError) as exc_info:
        shell.false()
    assert exc_info.value.command == ["false"]


def test_path_with_spaces_no_shell_injection(shell, tmp_path):
    """A path with spaces is passed as a single token, not split by a shell."""
    target = tmp_path / "my file.txt"
    shell.touch(str(target))
    assert target.exists()


def test_kwarg_value_with_spaces_no_injection(shell, tmp_path):
    """A kwarg value with spaces is passed as a single token."""
    target = tmp_path / "out file.txt"
    shell.touch(str(target))
    assert target.exists()


def test_dry_run_returns_none(dry_shell):
    """In dry-run mode, __call__ returns None instead of CompletedProcess."""
    result = dry_shell.echo("hello")
    assert result is None


def test_chained_proxy_independence(dry_shell, capsys):
    """Each chained proxy is independent — reusing a base proxy works correctly."""
    base = dry_shell.ros2.bag
    base.play("bag1")
    base.record("bag2")
    out = capsys.readouterr().out
    assert "[DRY RUN] ros2 bag play bag1" in out
    assert "[DRY RUN] ros2 bag record bag2" in out
