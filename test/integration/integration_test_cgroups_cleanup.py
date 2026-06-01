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

from lambkin.core.ctx.context import Context
from lambkin.core.ctx.source import Source
from lambkin.core.process.background import background

COOPERATIVE = (
    "import signal, sys, time; "
    "signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0)); "
    "time.sleep(30)"
)


def main():
    """Run integration test verifying full cgroup tree cleanup after iteration.

    Verifies that after a Context exits, the process cgroup and iteration
    cgroup are fully removed from the filesystem.
    """
    source = Source(path=Path(__file__))

    with Context(
        variant={},
        iteration=0,
        options={},
        source=source,
        variant_index=0,
    ) as ctx:
        with background(ctx.shell.python3, "-c", COOPERATIVE) as bp:
            child_cgroup = bp._cgroup
        iteration_cgroup = ctx._iteration_cgroup

    assert not child_cgroup.exists(), f"process cgroup not removed: {child_cgroup}"
    assert not iteration_cgroup.exists(), (
        f"iteration cgroup not removed: {iteration_cgroup}"
    )
    print("Cgroup cleanup test passed")


if __name__ == "__main__":
    main()
