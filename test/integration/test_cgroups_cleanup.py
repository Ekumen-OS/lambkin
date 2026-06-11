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

"""Integration test: cgroup tree is fully removed after iteration ends."""

from pathlib import Path

from lambkin.core.ctx import IterationContext
from lambkin.core.ctx.source import Source
from lambkin.core.process.background import background

COOPERATIVE = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "time.sleep(30)"
)


def test_cgroups_cleanup(tmp_path):
    """Verify full cgroup tree cleanup after a Context exits.

    After a Context exits, the process cgroup and iteration cgroup must be
    fully removed from the filesystem.
    """
    source = Source(path=Path(__file__))
    child_cgroup = None
    iteration_cgroup = None

    with IterationContext.from_params(
        variant={},
        variant_index=0,
        iteration=0,
        options={"dry_run": False, "no_cache": False, "log_output": "file"},
        source=source,
        base_dir=tmp_path,
    ) as ctx:
        with background(ctx.shell.python3, "-c", COOPERATIVE) as bp:
            child_cgroup = bp._cgroup
        iteration_cgroup = ctx._iteration_cgroup

    assert not child_cgroup.exists(), f"process cgroup not removed: {child_cgroup}"
    assert not iteration_cgroup.exists(), (
        f"iteration cgroup not removed: {iteration_cgroup}"
    )
