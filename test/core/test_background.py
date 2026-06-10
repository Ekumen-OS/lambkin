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

"""Unit tests for background process management via cgroups v2."""

import time

import pytest

from lambkin.common import signals
from lambkin.common.exceptions import LambkinProcessDiedUnexpectedlyError
from lambkin.core.process.background import background
from lambkin.core.process.cgroup import find_delegated_cgroup, make_iteration_cgroup
from lambkin.core.shell.proxy import ShellProxy


@pytest.fixture
def fake_cgroup(tmp_path):
    """Return a fake cgroup directory with a cgroup.procs file."""
    cgroup = tmp_path / "fake-iter-cgroup"
    cgroup.mkdir()
    (cgroup / "cgroup.procs").write_text("")
    return cgroup


@pytest.fixture
def dry_shell(tmp_path, fake_cgroup):
    """Return a ShellProxy in dry-run mode with a fake cgroup."""
    return ShellProxy(dry_run=True, cwd=tmp_path, cgroup=fake_cgroup)


def test_background_dry_run_prints_command(dry_shell):
    """In dry-run mode, background does not launch a real process."""
    bp = background(dry_shell.sleep, "10")
    assert bp._argv == ["sleep", "10"]
    assert bp._dry_run is True


def test_background_dry_run_does_not_launch_process(dry_shell):
    """In dry-run mode, no real process is started."""
    with background(dry_shell.sleep, "10") as bp:
        assert bp._proc is None


def test_background_dry_run_no_cgroup_created(dry_shell, fake_cgroup):
    """In dry-run mode, no child cgroup is created."""
    children_before = list(fake_cgroup.iterdir())
    with background(dry_shell.sleep, "10"):
        pass
    children_after = list(fake_cgroup.iterdir())
    assert children_before == children_after


def test_process_died_unexpectedly_error_message():
    """LambkinProcessDiedUnexpectedlyError carries argv and returncode."""
    err = LambkinProcessDiedUnexpectedlyError(["my_tool", "--flag"], 1)
    assert err.argv == ["my_tool", "--flag"]
    assert err.returncode == 1
    assert "my_tool" in str(err)
    assert "1" in str(err)


def test_process_died_unexpectedly_error_is_exception():
    """LambkinProcessDiedUnexpectedlyError is an Exception subclass."""
    assert issubclass(LambkinProcessDiedUnexpectedlyError, Exception)


def test_background_builds_argv_from_proxy(dry_shell):
    """background() builds the argv correctly from the proxy and args."""
    bp = background(dry_shell.ros2.bag.record, "--output", "output.mcap", "-a")
    assert bp._argv == ["ros2", "bag", "record", "--output", "output.mcap", "-a"]


def test_background_passes_cwd_from_proxy(fake_cgroup, tmp_path):
    """background() passes the proxy cwd to BackgroundProcess."""
    iter_dir = tmp_path / "iter_1"
    iter_dir.mkdir()
    shell = ShellProxy(dry_run=True, cwd=iter_dir, cgroup=fake_cgroup)
    bp = background(shell.sleep, "10")
    assert bp._cwd == iter_dir


def test_background_passes_cgroup_from_proxy(fake_cgroup, tmp_path):
    """background() passes the proxy cgroup to BackgroundProcess."""
    shell = ShellProxy(dry_run=True, cwd=tmp_path, cgroup=fake_cgroup)
    bp = background(shell.sleep, "10")
    assert bp._iteration_cgroup == fake_cgroup


def test_background_passes_dry_run_from_proxy(fake_cgroup, tmp_path):
    """background() passes the dry_run flag from the proxy."""
    shell = ShellProxy(dry_run=True, cwd=tmp_path, cgroup=fake_cgroup)
    bp = background(shell.sleep, "10")
    assert bp._dry_run is True


def test_background_process_starts_and_stops(tmp_path):
    """BackgroundProcess starts a real process and kills it on exit."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    with background(shell.sleep, "30") as bp:
        assert bp._proc is not None
        assert bp._proc.poll() is None
    assert bp._proc.poll() is not None


def test_background_process_cgroup_removed_on_exit(tmp_path):
    """BackgroundProcess removes its child cgroup on exit."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    with background(shell.sleep, "30") as bp:
        child_cgroup = bp._cgroup
        assert child_cgroup.exists()
    assert not child_cgroup.exists()


def test_background_process_raises_if_dies_unexpectedly(tmp_path):
    """LambkinProcessDiedUnexpectedlyError is raised if process dies before __exit__."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    signals.setup()

    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    with pytest.raises(LambkinProcessDiedUnexpectedlyError):
        with background(shell.sleep, "0"):
            shell.sleep("30")


def test_background_process_nested(tmp_path):
    """Two nested BackgroundProcess instances both start and stop correctly."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    with background(shell.sleep, "30") as bp1:
        with background(shell.sleep, "30") as bp2:
            assert bp1._proc.poll() is None
            assert bp2._proc.poll() is None
        assert bp2._proc.poll() is not None
        assert bp1._proc.poll() is None
    assert bp1._proc.poll() is not None


def test_background_process_cwd_is_used(tmp_path):
    """BackgroundProcess launches the process with the correct cwd."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    target = iteration_dir / "test_file.txt"

    payload = (
        "import time, pathlib; pathlib.Path('test_file.txt').touch(); time.sleep(30)"
    )
    with background(shell.python3, "-c", payload):
        time.sleep(0.5)
        assert target.exists()


def test_background_file_mode_creates_log_files(tmp_path):
    """background() in file mode creates stdout and stderr log files."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with background(shell.sleep, "30"):
        pass
    assert (iteration_dir / "sleep.stdout.log").exists()
    assert (iteration_dir / "sleep.stderr.log").exists()


def test_background_console_mode_creates_no_log_files(tmp_path):
    """background() with log_output='console' does not create log files."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with background(shell.sleep, "30", log_output="console"):
        pass
    assert not list(iteration_dir.glob("*.log"))
