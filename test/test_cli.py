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

from unittest.mock import MagicMock, patch

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


def test_relative_path_is_resolved(tmp_path, monkeypatch, dummy_script):
    """Script path is resolved to absolute before use."""
    monkeypatch.chdir(tmp_path)
    runner = CliRunner()
    with patch("subprocess.run") as mock_run:
        mock_run.return_value = MagicMock(returncode=0)
        runner.invoke(main, ["bench.py"])
        cmd = mock_run.call_args[0][0]
        assert str(dummy_script) in cmd


def test_systemd_not_found(dummy_script):
    """Exits with error if systemd-run is missing."""
    runner = CliRunner()
    with patch("subprocess.run", side_effect=FileNotFoundError):
        result = runner.invoke(main, [str(dummy_script)])
    assert result.exit_code != 0
    assert "systemd" in result.output


def test_keyboard_interrupt_stops_scope(dummy_script):
    """Ctrl-C stops the cgroup scope before exiting with code 130."""
    runner = CliRunner()
    with patch("subprocess.run") as mock_run:
        mock_run.side_effect = [KeyboardInterrupt, MagicMock(returncode=0)]
        result = runner.invoke(main, [str(dummy_script)])
    assert result.exit_code == 130
    calls = mock_run.call_args_list
    assert "stop" in calls[1][0][0]
