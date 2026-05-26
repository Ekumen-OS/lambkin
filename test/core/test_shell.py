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
def shell(tmp_path):
    """Return a ShellProxy in real execution mode."""
    return ShellProxy(dry_run=False, cwd=tmp_path)


@pytest.fixture
def dry_shell(tmp_path):
    """Return a ShellProxy in dry-run mode."""
    return ShellProxy(dry_run=True, cwd=tmp_path)


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
    assert result.returncode == 0


def test_returns_completed_process(shell):
    """__call__ returns a CompletedProcess instance on success."""
    result = shell.true()
    assert isinstance(result, subprocess.CompletedProcess)


def test_not_found_raises_command_error_and_contains_command_name(shell):
    """A command that does not exist raises CommandError with a helpful message."""
    with pytest.raises(CommandError) as exc_info:
        shell.this_command_does_not_exist_at_all()
    assert exc_info.value.returncode is None
    assert "not found" in str(exc_info.value).lower()
    assert "this_command_does_not_exist_at_all" in str(exc_info.value)


def test_permission_error_raises_command_error(shell, tmp_path):
    """A non-executable file raises CommandError with a helpful message."""
    script = tmp_path / "script.sh"
    script.write_text("#!/bin/bash\necho hello\n")
    script.chmod(0o644)
    with pytest.raises(CommandError) as exc_info:
        shell.__getattr__(str(script))()
    assert exc_info.value.returncode is None
    assert "permission" in str(exc_info.value).lower()


def test_command_error_carries_returncode_and_argv(shell):
    """CommandError carries the non-zero return code of the failed command."""
    with pytest.raises(CommandError) as exc_info:
        shell.false()
    assert exc_info.value.returncode == 1
    assert exc_info.value.command == ["false"]


def test_dry_run_returns_completed_process(dry_shell):
    """In dry-run mode, __call__ returns a dummy CompletedProcess with returncode=0."""
    result = dry_shell.echo("hello")
    assert isinstance(result, subprocess.CompletedProcess)
    assert result.returncode == 0
    assert result.args == ["echo", "hello"]


def test_chained_proxy_independence(dry_shell, capsys):
    """Each chained proxy is independent — reusing a base proxy works correctly."""
    base = dry_shell.ros2.bag
    base.play("bag1")
    base.record("bag2")
    out = capsys.readouterr().out
    assert "[DRY RUN] ros2 bag play bag1" in out
    assert "[DRY RUN] ros2 bag record bag2" in out


def test_path_with_spaces_no_shell_injection(shell, tmp_path):
    """A path with spaces is passed as a single token, not split by the shell."""
    target = tmp_path / "my file.txt"
    shell.touch(str(target))
    assert target.exists()


def test_log_output_default_is_file(shell):
    """Default log_output mode is 'file'."""
    assert shell.echo._resolve_log_output(None) == "file"


def test_log_output_per_call_overrides_default(shell):
    """Per-call log_output overrides the default."""
    assert shell.echo._resolve_log_output("console") == "console"


def test_log_output_cli_overrides_per_call(tmp_path):
    """CLI log_output overrides per-call value."""
    s = ShellProxy(dry_run=False, cwd=tmp_path, log_output="both")
    assert s.echo._resolve_log_output("console") == "both"


def test_log_base_appends_suffix_on_collision(shell):
    """_log_base appends numeric suffix when same command launched twice."""
    proxy = shell.ros2
    assert proxy._log_base(["ros2", "launch"]) == "ros2_launch"
    assert proxy._log_base(["ros2", "launch"]) == "ros2_launch_1"


def test_getattr_returns_ros_launch_command(tmp_path):
    """shell.ros2.launch returns a RosLaunchCommand."""
    from lambkin.core.shell.ros.launch import RosLaunchCommand

    s = ShellProxy(dry_run=False, cwd=tmp_path)
    assert isinstance(s.ros2.launch, RosLaunchCommand)


def test_file_mode_creates_log_files(tmp_path):
    """In file mode, running a command creates stdout and stderr log files."""
    s = ShellProxy(dry_run=False, cwd=tmp_path)
    s.echo("hello")
    assert len(list(tmp_path.glob("*.log"))) == 2
