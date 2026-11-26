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

"""Integration test: the cgroup layout the lambkin CLI sets up for a run."""

import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest

from lambkin.core.process.cgroup import find_app_slice, find_delegated_cgroup

LAMBKIN = Path(sys.executable).parent / "lambkin"

# The benchmark script the CLI runs. It reports what it sees from inside one
# iteration to report.json in LAMBKIN_TEST_OUT.
SCRIPT = """
import json, os
from pathlib import Path

from lambkin.core.ctx import IterationContext
from lambkin.core.ctx.source import Source

out = Path(os.environ["LAMBKIN_TEST_OUT"])
run = Path(os.environ["LAMBKIN_RUN_CGROUP"])
report = {
    "run": str(run),
    "self": Path("/proc/self/cgroup").read_text().strip(),
}
with IterationContext.from_params(
    variant={},
    variant_index=0,
    iteration=0,
    options={
        "dry_run": False,
        "no_cache": True,
        "log_output": "file",
        "log_level": "info",
    },
    source=Source(path=Path(__file__)),
    base_dir=out / "results",
) as ctx:
    report["iter"] = str(ctx._iteration_cgroup)
    report["run_procs"] = (run / "cgroup.procs").read_text().split()
    report["children"] = sorted(p.name for p in run.iterdir() if p.is_dir())
    controller = os.environ.get("LAMBKIN_TEST_CONTROLLER")
    if controller:
        control = run / "cgroup.subtree_control"
        try:
            control.write_text("+" + controller)
            report["enabled"] = controller in control.read_text().split()
            control.write_text("-" + controller)
        except OSError as e:
            report["enable_error"] = repr(e)
    if os.environ.get("LAMBKIN_TEST_HANG"):
        (out / "ready.json").write_text(json.dumps(report))
        ctx.shell.sleep(60)
if os.environ.get("LAMBKIN_TEST_LEAVE"):
    # Outside any iteration, as a benchmark-level hook would: stays in script/.
    import subprocess
    left = subprocess.Popen(
        ["sleep", "300"],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    report["leftover"] = left.pid
(out / "report.json").write_text(json.dumps(report))
"""


def _cli_parent_cgroup() -> Path | None:
    """Return the cgroup the CLI would create its run cgroup under, if any."""
    try:
        return find_app_slice() or find_delegated_cgroup()
    except (RuntimeError, OSError):
        return None


pytestmark = pytest.mark.skipif(
    _cli_parent_cgroup() is None, reason="cgroup v2 is not writable here"
)


@pytest.fixture
def script(tmp_path):
    """Write the benchmark script under a unique name."""
    path = tmp_path / f"cgtest_{uuid.uuid4().hex[:8]}.py"
    path.write_text(SCRIPT)
    return path


def _env(tmp_path: Path, **extra: str) -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if k != "LAMBKIN_RUN_CGROUP"}
    env["LAMBKIN_TEST_OUT"] = str(tmp_path)
    env.update(extra)
    return env


def _run_cli(script: Path, env: dict[str, str]) -> dict:
    result = subprocess.run(
        [str(LAMBKIN), str(script)],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
    return json.loads((script.parent / "report.json").read_text())


def _leftover_run_cgroups(script: Path) -> list[Path]:
    parent = _cli_parent_cgroup()
    assert parent is not None
    return list(parent.glob(f"lambkin-{script.stem}-*"))


def test_script_runs_in_script_and_run_cgroup_is_empty(script, tmp_path):
    """The script is in script/, a sibling of the iteration cgroups."""
    report = _run_cli(script, _env(tmp_path))

    run = Path(report["run"])
    assert report["self"].endswith(f"{run.name}/script")
    assert Path(report["iter"]).parent == run
    assert sorted(report["children"]) == sorted([
        "script",
        Path(report["iter"]).name,
    ])
    assert report["run_procs"] == []
    assert _leftover_run_cgroups(script) == []


@pytest.mark.parametrize("controller", ["cpuset", "memory"])
def test_run_cgroup_can_enable_controllers(script, tmp_path, controller):
    """With no member processes, the run cgroup can enable a controller."""
    parent = _cli_parent_cgroup()
    assert parent is not None
    delegated = (parent / "cgroup.subtree_control").read_text().split()
    if controller not in delegated:
        pytest.skip(f"{controller} controller is not delegated to {parent}")

    report = _run_cli(script, _env(tmp_path, LAMBKIN_TEST_CONTROLLER=controller))

    assert "enable_error" not in report, report.get("enable_error")
    assert report["enabled"] is True
    assert _leftover_run_cgroups(script) == []


def test_ctrl_c_mid_iteration_leaves_no_cgroup(script, tmp_path):
    """SIGINT to the CLI during a foreground command removes the run cgroup."""
    log = (tmp_path / "cli.log").open("w")
    proc = subprocess.Popen(
        [str(LAMBKIN), str(script)],
        env=_env(tmp_path, LAMBKIN_TEST_HANG="1"),
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    try:
        ready = tmp_path / "ready.json"
        deadline = time.monotonic() + 30
        while not ready.exists() and time.monotonic() < deadline:
            assert proc.poll() is None, (tmp_path / "cli.log").read_text()
            time.sleep(0.05)
        assert ready.exists(), "script never reached the foreground command"
        report = json.loads(ready.read_text())
        run = Path(report["run"])
        # SIGINT only once the foreground sleep runs in its own cgroup, so the
        # test covers Ctrl-C in the middle of a command.
        iteration = Path(report["iter"])
        deadline = time.monotonic() + 10
        while not any(
            (p / "cgroup.procs").read_text().strip() for p in iteration.glob("sleep-*")
        ):
            assert time.monotonic() < deadline, "foreground sleep never started"
            time.sleep(0.01)

        proc.send_signal(signal.SIGINT)
        assert proc.wait(timeout=30) == 130
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        log.close()

    assert not run.exists()
    assert _leftover_run_cgroups(script) == []


def _running(pid: int) -> bool:
    """Return whether pid is a live process.

    A killed process whose parent is gone is reparented to PID 1, and stays a
    zombie until PID 1 reaps it. In a container whose PID 1 is not an init,
    as in CI's ``podman run ... bash -c "... && uv run pytest"``, that never
    happens. A zombie has exited and left its cgroup, so it counts as gone.
    """
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except FileNotFoundError:
        return False
    return stat.rsplit(") ", 1)[1][0] != "Z"


def test_process_left_by_script_is_killed_on_normal_exit(script, tmp_path):
    """A process the script leaves in script/ is killed when the script exits."""
    report = _run_cli(script, _env(tmp_path, LAMBKIN_TEST_LEAVE="1"))

    pid = report["leftover"]
    deadline = time.monotonic() + 5
    while _running(pid) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not _running(pid), f"process {pid} left by the script is still alive"
    assert not Path(report["run"]).exists()
    assert _leftover_run_cgroups(script) == []
