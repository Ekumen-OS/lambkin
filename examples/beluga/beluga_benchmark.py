"""Nominal benchmark for the Beluga AMCL localization system."""
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

import lambkin


@lambkin.benchmark(
    variants=lambkin.common.named_product(
        sensor_model=["beam", "likelihood_field"], num_particles=[10, 20, 100]
    ),
    num_iterations=2,
)
@lambkin.option("--clock-rate", default=1.0)
@lambkin.option("--sensor-topic", default="/scan")
def nominal(ctx):
    """Run a nominal Beluga AMCL benchmark across sensor models and particle counts.

    Launches the Beluga localization stack, plays back a ROS 2 bag,
    and evaluates trajectory accuracy with evo_ape.

    Args:
        ctx: Lambkin context with variant, options, source, and shell access.
    """
    with lambkin.process.background(
        ctx.shell.ros2.bag.record,
        "--output",
        "output.mcap",
        "-a",
    ):
        with lambkin.process.background(
            ctx.shell.ros2.launch,
            "beluga_ros2",
            "beluga.launch.py",
            f"sensor_model:={ctx.variant.sensor_model}",
            f"num_particles:={ctx.variant.num_particles}",
            f"map_path:={ctx.inputs.map}",
        ):
            ctx.shell.ros2.bag.play(
                ctx.inputs.dataset, "--clock", "-r", ctx.options.clock_rate
            )


@nominal.input
def dataset(ctx):
    """Return the path to the MCAP dataset used as input for the benchmark."""
    return ctx.source.path.parent / "my_bags" / "my_bag.mcap"


@nominal.input
def map(ctx):
    """Return the path to the map file used for localization."""
    return "maps/map.yaml"


if __name__ == "__main__":
    nominal()
