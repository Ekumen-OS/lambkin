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

"""Regenerate the trajectories behind the committed evo result fixtures.

TUM format, one pose per line: ``timestamp tx ty tz qx qy qz qw``. The
reference is an arc rather than a straight line because a collinear trajectory
has a degenerate covariance, which makes evo's alignment fail; the estimate
adds a small drift and wobble.
"""

import math

RADIUS = 5.0
NUM_POSES = 10
STEP_DEG = 10.0


def pose(index: int, dx: float = 0.0, dy: float = 0.0) -> str:
    """Return one TUM-format pose on the arc, offset by ``dx`` and ``dy``."""
    theta = math.radians(STEP_DEG * index)
    x = RADIUS * math.sin(theta) + dx
    y = RADIUS * (1.0 - math.cos(theta)) + dy
    qz, qw = math.sin(theta / 2.0), math.cos(theta / 2.0)
    return (
        f"{float(index):.1f} {x:.6f} {y:.6f} 0.000000 "
        f"0.000000 0.000000 {qz:.6f} {qw:.6f}"
    )


def main() -> None:
    """Write ref.tum and est.tum into the current directory."""
    with open("ref.tum", "w") as f:
        for i in range(NUM_POSES):
            f.write(pose(i) + "\n")
    with open("est.tum", "w") as f:
        for i in range(NUM_POSES):
            f.write(pose(i, dx=0.01 * i, dy=0.02 * math.sin(i)) + "\n")


if __name__ == "__main__":
    main()
