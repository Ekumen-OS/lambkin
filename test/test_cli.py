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

"""Tests for the lambkin CLI entry point."""

import subprocess
from unittest.mock import MagicMock, call, patch

import pytest
from click.testing import CliRunner

from lambkin.cli import main


@pytest.fixture
def dummy_script(tmp_path):
    """Create a minimal valid benchmark script."""
    script = tmp_path / "bench.py"
    script.write_text("# dummy benchmark")
    return script


def test_missing_script_argument():
    """Exits with code 2 if no script is given."""
    runner = CliRunner()
    result = runner.invoke(main, [])
    assert result.exit_code == 2


def test_script_not_found():
    """Exits with code 2 if script does not exist."""
    runner = CliRunner()
    result = runner.invoke(main, ["/nonexistent/bench.py"])
    assert result.exit_code == 2


def test_absolute_path_is_passed_to_subprocess(dummy_script):
    """Script absolute path is forwarded to the subprocess."""
    runner = CliRunner()
    with patch("subprocess.Popen") as mock_popen:
        mock_popen.return_value = MagicMock(returncode=0)
        runner.invoke(main, [str(dummy_script)])
        cmd = mock_popen.call_args[0][0]
        assert str(dummy_script) in cmd


def test_keyboard_interrupt_exits_130(dummy_script):
    """Ctrl-C kills the cgroup and exits with code 130."""
    runner = CliRunner()
    with patch("subprocess.Popen") as mock_popen:
        mock_proc = MagicMock()
        mock_proc.wait.side_effect = KeyboardInterrupt
        mock_popen.return_value = mock_proc
        with patch("lambkin.cli.destroy_cgroup") as mock_destroy:
            with patch("lambkin.cli.make_cgroup") as mock_make:
                mock_make.return_value = MagicMock()
                with patch("lambkin.cli.find_delegated_cgroup"):
                    result = runner.invoke(main, [str(dummy_script)])
    assert result.exit_code == 130
    mock_destroy.assert_called_once_with(mock_make.return_value, grace_period=0.0)


def test_script_runs_in_script_with_run_cgroup_exported(dummy_script, tmp_path):
    """The script joins run_cgroup/script and gets LAMBKIN_RUN_CGROUP."""
    runner = CliRunner()
    run_cgroup = tmp_path / "lambkin-bench-1234"
    run_cgroup.mkdir()
    with patch("subprocess.Popen") as mock_popen:
        mock_popen.return_value = MagicMock(returncode=0)
        with patch(
            "lambkin.cli.make_cgroup",
            side_effect=lambda parent, name: (
                run_cgroup / name if parent == run_cgroup else run_cgroup
            ),
        ) as mk:
            with patch("lambkin.cli.destroy_cgroup"):
                with patch("lambkin.cli.cgroup_joiner") as mock_joiner:
                    result = runner.invoke(main, [str(dummy_script)])
    assert result.exit_code == 0
    mk.assert_any_call(run_cgroup, "script")
    assert mock_popen.call_args.kwargs["env"]["LAMBKIN_RUN_CGROUP"] == str(run_cgroup)
    mock_joiner.assert_called_once_with(run_cgroup / "script")
    assert mock_popen.call_args.kwargs["preexec_fn"] is mock_joiner.return_value


def test_script_cgroup_failure_removes_run_cgroup(dummy_script, tmp_path):
    """If the script cgroup cannot be made, the run cgroup is removed."""
    run_cgroup = tmp_path / "lambkin-bench-1234"

    def _make(parent, name):
        if parent == run_cgroup:
            raise PermissionError("denied")
        return run_cgroup

    runner = CliRunner()
    with patch("subprocess.Popen") as mock_popen:
        with patch("lambkin.cli.make_cgroup", side_effect=_make):
            with patch("lambkin.cli.destroy_cgroup") as mock_destroy:
                result = runner.invoke(main, [str(dummy_script)])
    assert isinstance(result.exception, PermissionError)
    mock_popen.assert_not_called()
    mock_destroy.assert_called_once_with(run_cgroup, grace_period=0.0)


@pytest.mark.parametrize(
    "error",
    [FileNotFoundError("no python"), subprocess.SubprocessError("preexec failed")],
)
def test_script_start_failure_removes_run_cgroup(dummy_script, tmp_path, error):
    """If the script never starts, the run cgroup is removed."""
    run_cgroup = tmp_path / "lambkin-bench-1234"
    runner = CliRunner()
    with patch("subprocess.Popen", side_effect=error):
        with patch(
            "lambkin.cli.make_cgroup",
            side_effect=lambda parent, name: (
                run_cgroup / name if parent == run_cgroup else run_cgroup
            ),
        ):
            with patch("lambkin.cli.destroy_cgroup") as mock_destroy:
                result = runner.invoke(main, [str(dummy_script)])
    assert result.exception is error
    mock_destroy.assert_called_once_with(run_cgroup, grace_period=0.0)


def test_ctrl_c_while_starting_removes_run_cgroup(dummy_script, tmp_path):
    """Ctrl-C while the script starts exits 130 and removes the run cgroup."""
    run_cgroup = tmp_path / "lambkin-bench-1234"
    runner = CliRunner()
    with patch("subprocess.Popen", side_effect=KeyboardInterrupt):
        with patch(
            "lambkin.cli.make_cgroup",
            side_effect=lambda parent, name: (
                run_cgroup / name if parent == run_cgroup else run_cgroup
            ),
        ):
            with patch("lambkin.cli.destroy_cgroup") as mock_destroy:
                result = runner.invoke(main, [str(dummy_script)])
    assert result.exit_code == 130
    mock_destroy.assert_called_once_with(run_cgroup, grace_period=0.0)


def test_terminal_state_failure_creates_no_cgroup(dummy_script):
    """If the terminal state cannot be saved, no run cgroup is left behind."""
    runner = CliRunner()
    with patch("lambkin.cli._save_terminal_state", side_effect=OSError("tty")):
        with patch("lambkin.cli.make_cgroup") as mock_make:
            result = runner.invoke(main, [str(dummy_script)])
    assert isinstance(result.exception, OSError)
    mock_make.assert_not_called()


def test_help_output_contains_expected_sections():
    """Help output contains the expected sections."""
    runner = CliRunner()
    result = runner.invoke(main, ["--help"])
    assert result.exit_code == 0
    assert "SDK Options (always available)" in result.output
    assert "Custom Options (script-defined)" in result.output
    assert "--dry-run" in result.output


def test_normal_exit_destroys_run_cgroup(dummy_script, tmp_path):
    """Whatever the script left running is killed before the cgroup is removed."""
    run_cgroup = tmp_path / "lambkin-bench-1234"
    calls = MagicMock()
    runner = CliRunner()
    with patch("subprocess.Popen") as mock_popen:
        mock_popen.return_value = MagicMock(returncode=3)
        with patch(
            "lambkin.cli.make_cgroup",
            side_effect=lambda parent, name: (
                run_cgroup / name if parent == run_cgroup else run_cgroup
            ),
        ):
            with patch("lambkin.cli.cgroup_joiner"):
                with patch("lambkin.cli.destroy_cgroup", calls.destroy):
                    result = runner.invoke(main, [str(dummy_script)])
    assert result.exit_code == 3
    assert calls.mock_calls == [call.destroy(run_cgroup, grace_period=0.0)]
