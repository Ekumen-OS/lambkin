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

import signal
import time

import pytest
import yaml

from lambkin.common import defaults, signals
from lambkin.common.exceptions import LambkinProcessDiedUnexpectedlyError
from lambkin.core.process.background import background
from lambkin.core.process.cgroup import find_delegated_cgroup, make_iteration_cgroup
from lambkin.core.process.resources import ResourceSampler
from lambkin.core.shell.proxy import ShellProxy

COOPERATIVE = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "time.sleep(30)"
)

# Allocates a few MiB and burns CPU, so a measurement of it is non-zero.
BURN = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "blob = bytearray(8 * 1024 * 1024); "
    "end = time.monotonic() + 30; "
    "\nwhile time.monotonic() < end: pass"
)


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


@pytest.mark.parametrize("duration", ["0", "0.05"])
def test_background_process_raises_if_dies_unexpectedly(tmp_path, duration):
    """LambkinProcessDiedUnexpectedlyError is raised if process dies before __exit__."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    signals.setup()

    shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
    with pytest.raises(LambkinProcessDiedUnexpectedlyError):
        with background(shell.sleep, duration):
            shell.python3("-c", COOPERATIVE)


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


def test_background_process_sets_sigusr1_pending_on_unexpected_death(tmp_path):
    """sigusr1_pending is set by the monitor thread when a process dies unexpectedly."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    signals.sigusr1_pending.clear()
    pending_was_set = []

    def _capturing_handler(signum, frame):
        pending_was_set.append(signals.sigusr1_pending.is_set())
        raise LambkinProcessDiedUnexpectedlyError([], 1)

    previous = signal.getsignal(signal.SIGUSR1)
    signal.signal(signal.SIGUSR1, _capturing_handler)
    try:
        shell = ShellProxy(dry_run=False, cwd=tmp_path, cgroup=cgroup)
        with pytest.raises(LambkinProcessDiedUnexpectedlyError):
            with background(shell.sleep, "0"):
                shell.python3("-c", COOPERATIVE)
        assert pending_was_set and pending_was_set[0], (
            "sigusr1_pending should be set before the signal handler runs"
        )
    finally:
        signal.signal(signal.SIGUSR1, previous)
        signals.sigusr1_pending.clear()


def test_background_measure_defaults_to_empty(dry_shell):
    """Measure is empty unless asked for."""
    assert background(dry_shell.sleep, "10")._measure == ()


def test_background_measure_accepts_a_string(dry_shell):
    """A single process name is normalized to a one-element tuple."""
    bp = background(dry_shell.sleep, "10", measure="cartographer_node")
    assert bp._measure == ("cartographer_node",)


def test_background_measure_accepts_a_sequence(dry_shell):
    """A sequence of process names is preserved in order."""
    bp = background(dry_shell.sleep, "10", measure=["node_a", "node_b"])
    assert bp._measure == ("node_a", "node_b")


def test_background_measure_rejects_an_empty_name(dry_shell):
    """A blank process name is rejected."""
    with pytest.raises(ValueError):
        background(dry_shell.sleep, "10", measure="  ")


def test_background_measure_rejects_a_non_string(dry_shell):
    """A non-string process name is rejected."""
    with pytest.raises(TypeError):
        background(dry_shell.sleep, "10", measure=[42])


def test_background_measure_is_not_passed_to_the_child_argv(dry_shell):
    """Measure configures the SDK and never reaches the command line."""
    bp = background(dry_shell.sleep, "10", measure="cartographer_node")
    assert bp._argv == ["sleep", "10"]
    assert "--measure" not in bp._argv
    assert "cartographer_node" not in bp._argv


def test_background_measure_interval_is_not_passed_to_the_child_argv(dry_shell):
    """measure_interval configures the SDK and never reaches the command line."""
    bp = background(dry_shell.sleep, "10", measure="node", measure_interval=0.1)
    assert bp._argv == ["sleep", "10"]
    assert "--measure-interval" not in bp._argv


def test_background_measure_interval_defaults_to_the_sdk_default(dry_shell):
    """An unset measure_interval falls back to defaults.MEASURE_INTERVAL."""
    bp = background(dry_shell.sleep, "10", measure="node")
    assert bp._measure_interval == defaults.MEASURE_INTERVAL


def test_background_dry_run_measure_writes_no_artifacts(dry_shell, tmp_path):
    """In dry-run mode, measurement produces no files."""
    with background(dry_shell.sleep, "10", measure="sleep"):
        pass
    assert not list(tmp_path.glob("*.resources.*"))


def test_background_measures_a_real_process(tmp_path):
    """A measured background process gets a populated summary and series."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with background(
        shell.python3, "-c", BURN, measure="python3", measure_interval=0.05
    ):
        time.sleep(0.8)

    summary = yaml.safe_load((iteration_dir / "python3.resources.yaml").read_text())
    assert summary["pid_found"] is True
    assert summary["cpu_total_s"] > 0
    assert summary["peak_rss_mib"] > 0
    rows = (iteration_dir / "python3.resources.csv").read_text().splitlines()
    assert len(rows) > 1


def test_background_measure_missing_process_writes_a_summary(tmp_path):
    """A name that never runs still produces a summary saying so."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with background(shell.sleep, "30", measure="definitely_not_running"):
        time.sleep(0.2)

    path = iteration_dir / "definitely_not_running.resources.yaml"
    assert yaml.safe_load(path.read_text())["pid_found"] is False


def test_background_cgroup_removed_when_sampler_fails(tmp_path, monkeypatch):
    """A failing sampler propagates but still leaves the cgroup cleaned up."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)

    def _boom(self):
        raise RuntimeError("sampler exploded")

    monkeypatch.setattr(ResourceSampler, "stop", _boom)

    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with pytest.raises(RuntimeError, match="sampler exploded"):
        with background(shell.sleep, "30", measure="sleep") as bp:
            child_cgroup = bp._cgroup
    assert not child_cgroup.exists()
