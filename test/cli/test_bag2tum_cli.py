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

"""Tests for the lambkin-bag2tum CLI entry point."""

from pathlib import Path
from unittest.mock import patch

import pytest
from click.testing import CliRunner

from lambkin.cli.bag2tum import main


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


def test_calls_library(runner: CliRunner, input_bag: Path, tmp_path: Path) -> None:
    """Forwards arguments to the bag2tum library function."""
    output_tum = tmp_path / "out.tum"
    with patch("lambkin.cli.bag2tum.bag2tum") as mock:
        result = runner.invoke(
            main,
            [
                "--input",
                str(input_bag),
                "--topic",
                "/ground_truth",
                "--output",
                str(output_tum),
            ],
        )
    assert result.exit_code == 0
    mock.assert_called_once_with(
        input_bag=input_bag,
        topic="/ground_truth",
        output_tum=output_tum,
    )


def test_propagates_file_not_found(
    runner: CliRunner, input_bag: Path, tmp_path: Path
) -> None:
    """Exits with code 1 and prints error on FileNotFoundError."""
    with patch(
        "lambkin.cli.bag2tum.bag2tum", side_effect=FileNotFoundError("not found")
    ):
        result = runner.invoke(
            main,
            [
                "--input",
                str(input_bag),
                "--topic",
                "/ground_truth",
                "--output",
                str(tmp_path / "out.tum"),
            ],
        )
    assert result.exit_code == 1
    assert "Error:" in result.output


def test_propagates_file_exists(
    runner: CliRunner, input_bag: Path, tmp_path: Path
) -> None:
    """Exits with code 1 and prints error on FileExistsError."""
    with patch(
        "lambkin.cli.bag2tum.bag2tum", side_effect=FileExistsError("already exists")
    ):
        result = runner.invoke(
            main,
            [
                "--input",
                str(input_bag),
                "--topic",
                "/ground_truth",
                "--output",
                str(tmp_path / "out.tum"),
            ],
        )
    assert result.exit_code == 1
    assert "Error:" in result.output


def test_propagates_value_error(
    runner: CliRunner, input_bag: Path, tmp_path: Path
) -> None:
    """Exits with code 1 and prints error on ValueError."""
    with patch(
        "lambkin.cli.bag2tum.bag2tum", side_effect=ValueError("topic not found")
    ):
        result = runner.invoke(
            main,
            [
                "--input",
                str(input_bag),
                "--topic",
                "/ground_truth",
                "--output",
                str(tmp_path / "out.tum"),
            ],
        )
    assert result.exit_code == 1
    assert "Error:" in result.output
