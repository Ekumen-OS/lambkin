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

from lambkin.cli import main


@pytest.fixture
def dummy_script(tmp_path):
    """Create a minimal valid benchmark script."""
    script = tmp_path / "bench.py"
    script.write_text("# dummy benchmark")
    return script


def test_missing_script_argument(capsys):
    """Exits with code 2 (argparse default) if no script is given."""
    with patch("sys.argv", ["lambkin"]):
        with pytest.raises(SystemExit) as exc:
            main()
    assert exc.value.code == 2


def test_script_not_found(capsys):
    """Exits with code 1 and prints a clear error if script does not exist."""
    with patch("sys.argv", ["lambkin", "/nonexistent/bench.py"]):
        with pytest.raises(SystemExit) as exc:
            main()
    assert exc.value.code == 1
    captured = capsys.readouterr()
    assert "not found" in captured.err


def test_relative_path_is_resolved(tmp_path, monkeypatch, dummy_script):
    """Script path is resolved to absolute before use."""
    monkeypatch.chdir(tmp_path)
    with patch("sys.argv", ["lambkin", "bench.py"]):
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=0)

            mock_run.side_effect = [
                MagicMock(returncode=1),
                MagicMock(returncode=0),
            ]
            with pytest.raises(SystemExit):
                main()

    systemd_call = mock_run.call_args_list[1]
    cmd = systemd_call[0][0]
    assert str(dummy_script) in cmd


def test_systemd_not_found(capsys, dummy_script):
    """Exits with code 127 and prints a clear error if systemd-run is missing."""
    with patch("sys.argv", ["lambkin", str(dummy_script)]):
        with patch("subprocess.run") as mock_run:
            mock_run.side_effect = [
                MagicMock(returncode=1),
                FileNotFoundError,
            ]
            with pytest.raises(SystemExit) as exc:
                main()
    assert exc.value.code == 127
    captured = capsys.readouterr()
    assert "systemd" in captured.err


def test_scope_already_active_without_overwrite(capsys, dummy_script):
    """Exits with code 1 and clear error if scope is active and --overwrite not set."""
    with patch("sys.argv", ["lambkin", str(dummy_script)]):
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=0)
            with pytest.raises(SystemExit) as exc:
                main()
    assert exc.value.code == 1
    captured = capsys.readouterr()
    assert "overwrite" in captured.err


def test_scope_already_active_with_overwrite(dummy_script):
    """With --overwrite, stops existing scope and re-launches."""
    with patch("sys.argv", ["lambkin", str(dummy_script), "--overwrite"]):
        with patch("subprocess.run") as mock_run:
            mock_run.side_effect = [
                MagicMock(returncode=0),
                MagicMock(returncode=0),
                MagicMock(returncode=0),
            ]
            with pytest.raises(SystemExit):
                main()
    calls = mock_run.call_args_list
    assert "stop" in calls[1][0][0]


def test_keyboard_interrupt_stops_scope(dummy_script):
    """Ctrl-C stops the cgroup scope before exiting with code 130."""
    with patch("sys.argv", ["lambkin", str(dummy_script)]):
        with patch("subprocess.run") as mock_run:
            mock_run.side_effect = [
                MagicMock(returncode=1),
                KeyboardInterrupt,
                MagicMock(returncode=0),
            ]
            with pytest.raises(SystemExit) as exc:
                main()
    assert exc.value.code == 130
    calls = mock_run.call_args_list
    assert "stop" in calls[2][0][0]
