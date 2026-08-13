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
"""Odometry baseline: raw wheel odometry vs. robot_localization EKF fusion."""

import matplotlib.pyplot as plt

import lambkin


@lambkin.benchmark(
    variants=lambkin.common.named_product(algorithm=["raw_odom", "ekf_fused"]),
    num_iterations=2,
)
@lambkin.option("--clock-rate", default=1.0)
def nominal(ctx):
    """Evaluate raw wheel odometry against an EKF-fused (odom+IMU) estimate.

    Args:
        ctx: Lambkin context with variant, options, source, and shell access.

    Raises:
        ValueError: If ctx.variant.algorithm is not a known algorithm.
    """
    lambkin.logger.info(
        "Running benchmark with variant: %s, iteration: %d", ctx.variant, ctx.iteration
    )

    match ctx.variant.algorithm:
        case "raw_odom":
            ctx.shell.evo_traj.bag2(ctx.inputs.dataset, "/odom", "--save_as_tum")
            odom_tum = "odom.tum"
        case "ekf_fused":
            with lambkin.process.background(
                ctx.shell.ros2.bag.record,
                "--output",
                "output",
                "--topics",
                "/odom_filtered",
            ):
                with lambkin.process.background(
                    ctx.shell.ros2.launch,
                    "ekf_ros2",
                    "ekf.launch.py",
                ):
                    ctx.shell.ros2.bag.play(
                        ctx.inputs.dataset, "-r", ctx.options.clock_rate
                    )
            ctx.shell.evo_traj.bag2("output", "/odom_filtered", "--save_as_tum")
            odom_tum = "odom_filtered.tum"
        case unknown:
            raise ValueError(
                f"Unknown algorithm {unknown!r}: expected 'raw_odom' or 'ekf_fused'."
            )

    ctx.shell.evo_rpe.tum(
        ctx.inputs.ground_truth,
        odom_tum,
        "--t_max_diff",
        "0.5",
        "--delta",
        "1",
        "--delta_unit",
        "m",
        "--save_results",
        "output.rpe.zip",
    )


@nominal.input
def dataset(ctx):
    """Return the path to the MCAP dataset used as input for the benchmark."""
    return "/data/datasets/input"


@nominal.input
def ground_truth(ctx):
    """Return the path to the TUM ground-truth trajectory file."""
    return "/data/ground_truth/groundtruth.tum"


@nominal.output
def plots(ctx):
    """Save RPE timeseries plot for all iterations, one line per variant."""
    for entry in lambkin.data.evo.series(ctx, "output.rpe.zip"):
        plt.plot(
            entry.time, entry.error, label=f"{entry.variant} / iter {entry.iteration}"
        )
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()
    plt.savefig(ctx.base_dir / "plots.png")


@nominal.output
def stats(ctx):
    """Log RMSE, mean, and max RPE for all iterations."""
    for entry in lambkin.data.evo.stats(ctx, "output.rpe.zip"):
        lambkin.logger.info(
            "%s iter %d: rmse=%.4f mean=%.4f max=%.4f",
            entry.variant,
            entry.iteration,
            entry.rmse,
            entry.mean,
            entry.max,
        )


if __name__ == "__main__":
    nominal()
