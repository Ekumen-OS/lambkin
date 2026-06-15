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

"""Tests for the lambkin-tum2bag CLI entry point."""

from pathlib import Path
from unittest.mock import patch

import pytest
from click.testing import CliRunner

from lambkin.cli.tum2bag import main


@pytest.fixture
def tum_file(tmp_path: Path) -> Path:
    """Create a minimal valid TUM file."""
    tum = tmp_path / "gt.tum"
    tum.write_text("1.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n")
    return tum


@pytest.fixture
def input_bag(tmp_path: Path) -> Path:
    """Create a minimal valid input bag directory."""
    bag = tmp_path / "bag"
    bag.mkdir()
    return bag


@pytest.fixture
def runner() -> CliRunner:
    """Return a Click test runner."""
    return CliRunner()


def test_missing_required_options(runner: CliRunner) -> None:
    """Exits with error if required options are missing."""
    result = runner.invoke(main, [])
    assert result.exit_code != 0
    assert "Missing option" in result.output


def test_calls_library(
    runner: CliRunner, tum_file: Path, input_bag: Path, tmp_path: Path
) -> None:
    """Forwards arguments to the tum2bag library function."""
    output_bag = tmp_path / "out"
    with patch("lambkin.cli.tum2bag.tum2bag") as mock:
        result = runner.invoke(
            main,
            [
                "--tum",
                str(tum_file),
                "--input",
                str(input_bag),
                "--output",
                str(output_bag),
                "--topic",
                "/gt",
            ],
        )
    assert result.exit_code == 0
    mock.assert_called_once_with(
        tum_path=tum_file,
        input_bag=input_bag,
        output_bag=output_bag,
        topic="/gt",
    )


def test_propagates_file_not_found(
    runner: CliRunner, tum_file: Path, input_bag: Path, tmp_path: Path
) -> None:
    """Exits with code 1 and prints error on FileNotFoundError."""
    with patch(
        "lambkin.cli.tum2bag.tum2bag", side_effect=FileNotFoundError("not found")
    ):
        result = runner.invoke(
            main,
            [
                "--tum",
                str(tum_file),
                "--input",
                str(input_bag),
                "--output",
                str(tmp_path / "out"),
            ],
        )
    assert result.exit_code == 1
    assert "Error:" in result.output


def test_propagates_file_exists(
    runner: CliRunner, tum_file: Path, input_bag: Path, tmp_path: Path
) -> None:
    """Exits with code 1 and prints error on FileExistsError."""
    with patch(
        "lambkin.cli.tum2bag.tum2bag", side_effect=FileExistsError("already exists")
    ):
        result = runner.invoke(
            main,
            [
                "--tum",
                str(tum_file),
                "--input",
                str(input_bag),
                "--output",
                str(tmp_path / "out"),
            ],
        )
    assert result.exit_code == 1
    assert "Error:" in result.output


def test_propagates_value_error(
    runner: CliRunner, tum_file: Path, input_bag: Path, tmp_path: Path
) -> None:
    """Exits with code 1 and prints error on ValueError."""
    with patch("lambkin.cli.tum2bag.tum2bag", side_effect=ValueError("bad tum")):
        result = runner.invoke(
            main,
            [
                "--tum",
                str(tum_file),
                "--input",
                str(input_bag),
                "--output",
                str(tmp_path / "out"),
            ],
        )
    assert result.exit_code == 1
    assert "Error:" in result.output
