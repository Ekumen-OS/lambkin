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

"""Integration test: background commands clean up their whole cgroup tree."""

import time

import pytest

from lambkin.common.exceptions import CommandError
from lambkin.core.process.background import background
from lambkin.core.process.cgroup import (
    find_delegated_cgroup,
    kill_cgroup_tree,
    make_iteration_cgroup,
    remove_cgroup_tree,
)
from lambkin.core.shell.proxy import ShellProxy


@pytest.fixture
def live_cgroup(tmp_path):
    """Return a real iteration cgroup and its directory, removed afterwards."""
    iteration_dir = tmp_path / "var_1" / "iter_1"
    iteration_dir.mkdir(parents=True)
    cgroup = make_iteration_cgroup(find_delegated_cgroup(), iteration_dir)
    yield iteration_dir, cgroup
    if cgroup.exists():
        kill_cgroup_tree(cgroup)
        remove_cgroup_tree(cgroup)


def test_background_kills_process_in_nested_cgroup(wait_gone, live_cgroup):
    """A process the command moved into a child cgroup is killed on exit."""
    iteration_dir, cgroup = live_cgroup
    pid_file = iteration_dir / "nested.pid"
    script = iteration_dir / "nest.sh"
    script.write_text(
        "cg=\"/sys/fs/cgroup$(sed -n 's/^0:://p' /proc/self/cgroup)\"\n"
        'mkdir "$cg/sub"\n'
        f'sh -c \'echo $$ > "$0/sub/cgroup.procs"; echo $$ > {pid_file};'
        ' exec sleep 64\' "$cg" &\n'
        "exec sleep 64\n"
    )
    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    with background(shell.sh, str(script)):
        deadline = time.monotonic() + 5
        while not (pid_file.exists() and pid_file.read_text().strip()):
            assert time.monotonic() < deadline, "nested process never started"
            time.sleep(0.01)
    nested = int(pid_file.read_text())
    assert wait_gone(nested), f"nested {nested} still alive"
    assert [p for p in cgroup.iterdir() if p.is_dir()] == []


def test_background_missing_iteration_cgroup_is_reported(live_cgroup):
    """A cgroup that cannot be created raises CommandError and closes the logs."""
    iteration_dir, cgroup = live_cgroup
    cgroup.rmdir()
    shell = ShellProxy(dry_run=False, cwd=iteration_dir, cgroup=cgroup)
    bp = background(shell.sleep, "30")
    with pytest.raises(CommandError, match="Could not create a cgroup for 'sleep'"):
        with bp:
            pass
    assert bp._stdout.closed and bp._stderr.closed
