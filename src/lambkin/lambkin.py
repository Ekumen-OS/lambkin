#!/usr/bin/env python3
"""Lambkin library for executing and benchmarking Beluga AMCL."""

import subprocess
import time
from pathlib import Path

reference_bag_path = (
    "/ws/rosbags/magazzino_ros2_localization_only/bagfiles/hallway_localization/"
)
reference_map_path = "/ws/rosbags/maps/map.yaml"
num_iterations = 1
clock_rate = 10
qos_file_path = "/ws/beluga/beluga_example/bags/qos_override.yaml"


LASER_MODELS = ["likelihood", "beam"]
NUM_PARTICLES = [1, 10, 100, 1000, 10000]
CLOCK_RATE_OPTION = ["--clock-rate", str(clock_rate)]
QOS_OPTION = ["--qos-profile-overrides-path", qos_file_path]
CLOCK_OPTION = "--clock"
RECORD_TOPICS_INTERESTED = ["/pose", "/tf", "/tf_static"]
RESULTS_PATH = "/ws/lambkin/benchmarking_results"
DRY_MODE = True


def execute_background_process(
    full_cmd_list: list[str], dry_mode: bool = False, log_file: str = None
) -> subprocess.Popen:
    """Executes a shell command in the background.

    Args:
        full_cmd_list (list[str]): The command and its arguments as a list of strings.
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.
        log_file (str, optional): Path to a output log file.

    Returns:
        subprocess.Popen: The running background process.
    """
    if dry_mode:
        print(f"{full_cmd_list}\n")
        return None

    if log_file:
        file = open(log_file, "w")
        return subprocess.Popen(full_cmd_list, stdout=file, stderr=file)
    return subprocess.Popen(full_cmd_list)


def execute_foreground_process(
    full_cmd_list: list[str], dry_mode: bool = False, log_file: str = None
) -> subprocess.run:
    """Executes a shell command in the foreground.

    Args:
        full_cmd_list (list[str]): The command and its arguments as a list of strings.
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.
        log_file (str, optional): Path to a output log file.

    Returns:
        subprocess.run: The running foreground process.
    """
    if dry_mode:
        print(f"{full_cmd_list}\n")
        return None
    if log_file:
        file = open(log_file, "w")
        return subprocess.run(full_cmd_list, stdout=file, stderr=file, check=True)
    return subprocess.run(full_cmd_list, check=True)


def ros_bag_record(
    output_path: str, options: list[str], dry_mode: bool = False
) -> subprocess.Popen:
    """Starts a ROS 2 process to record a bag file.

    Args:
        output_path (str): The base directory where the bag will be saved.
        options (list[str]): Additional options and topics.
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.

    Returns:
        subprocess.Popen: The running ros2 bag record process.
    """
    bag_dir = Path(output_path) / "bag"
    bag_dir.mkdir(parents=True, exist_ok=True)

    return execute_background_process(
        (["ros2", "bag", "record", "-o", str(output_path)] + options), dry_mode
    )


def ros_bag_play(
    input_path: str, options: list[str], dry_mode: bool = False
) -> subprocess.Popen:
    """Starts a ROS 2 process to play a bag file.

    Args:
        input_path (str): The path to the input bag file.
        options (list[str]): Additional options for playback (e.g., clock rate, QoS).
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.

    Returns:
        subprocess.Popen: The running ros2 bag play process.
    """
    cmd_list = ["ros2", "bag", "play", input_path] + options
    return execute_background_process(cmd_list, dry_mode)


def beluga(
    sensor_model: str, num_particles: int, map_path: str, dry_mode: bool = False
) -> subprocess.Popen:
    """Launches the Beluga AMCL node using a custom launch file.

    Args:
        sensor_model (str): The laser sensor model to use.
        num_particles (int): The maximum number of particles for AMCL.
        map_path (str): The absolute path to the map file.
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.

    Returns:
        subprocess.Popen: The running launch process.
    """
    cmd_list = [
        "ros2",
        "launch",
        "lambkin",
        "lambkin_launch.py",
        f"map_path:={map_path}",
        f"laser_model_type:={sensor_model}",
        f"max_particles:={num_particles}",
    ]
    return execute_background_process(cmd_list, dry_mode, log_file="beluga.log")


def bag2tum(bag_path: str, tum_path: str, topic: str) -> str:
    """Converts a ROS 2 bag file to a TUM trajectory format using evo_traj.

    Args:
        bag_path (str): The path to the ROS 2 bag file.
        tum_path (str): The destination directory for the generated TUM file.
        topic (str): The ROS topic containing the trajectory data.

    Returns:
        str: The absolute path to the generated TUM file.
    """
    tum_dir = Path(tum_path) / "tum"
    tum_dir.mkdir(parents=True, exist_ok=True)

    cmd_list = ["evo_traj", "bag2", str(bag_path), topic, "--save_as_tum"]
    subprocess.run(cmd_list, cwd=tum_dir, check=True)

    tum_name = f"{topic.strip('/')}.tum"
    return str(tum_dir / tum_name)


def evo_ape(
    reference_path: str, record_path: str, topic: str = "/pose", dry_mode: bool = False
):
    """Computes the Absolute Pose Error (APE).

    Args:
        reference_path (str): The path to the reference TUM file.
        record_path (str): The path to the recorded bag file.
        topic (str, optional): The topic to evaluate. Defaults to "/pose".
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.
    """
    record_path_obj = Path(record_path)
    iter_dir = record_path_obj.parent.parent

    record_tum = bag2tum(str(record_path_obj), str(iter_dir), topic)

    ape_dir = iter_dir / "ape"
    ape_dir.mkdir(parents=True, exist_ok=True)

    cmd_list = [
        "evo_ape",
        "tum",
        reference_path,
        record_tum,
        "-va",
        "--align",
        "--save_results",
        str(ape_dir / "ape.zip"),
    ]
    execute_foreground_process(cmd_list, dry_mode)


def wait_for_processes(
    waitlist: list[subprocess.Popen], termination_list: list[subprocess.Popen]
) -> None:
    """Manages process synchronization by waiting for specific processes.

    It waits for processes in the waitlist to complete naturally and
    actively terminates others in the termination_list.

    Args:
        waitlist (list[subprocess.Popen]): Processes that must complete naturally.
        termination_list (list[subprocess.Popen]): Processes that should be terminated.
    """
    for p in waitlist:
        if p is not None:
            p.wait()
    for p in termination_list:
        if p is not None:
            p.terminate()
    for p in termination_list:
        if p is not None:
            try:
                p.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                print("Forcing process termination...")
                p.kill()


def plot_ape_metrics(ape_path: str, dry_mode: bool = False) -> None:
    """Plots the Absolute Pose Error (APE) metrics using evo_res.

    Args:
        ape_path (str): The path to the results zip file generated by evo_ape.
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.
    """
    cmd_list = ["evo_res", ape_path]
    execute_foreground_process(cmd_list, dry_mode)


def make_variations() -> list:
    """Generates all possible combinations for the benchmark.

    It creates configurations of sensor models and particle counts.

    Returns:
        list: A list of dictionaries containing configuration pairs.
    """
    variations = []
    for sensor_model in LASER_MODELS:
        for particles in NUM_PARTICLES:
            variations.append(
                {
                    "sensor_model": sensor_model,
                    "num_particles": particles,
                }
            )
    return variations


def run_iteration(
    variation: dict,
    iteration: int,
    map_reference_path: str,
    reference_bag_path: str,
    dry_mode: bool = False,
) -> None:
    """Executes a single benchmark iteration with a specific configuration.

    Args:
        variation (dict): The configuration dictionary containing the
            sensor model and particle count.
        iteration (int): The current iteration number.
        map_reference_path (str): The absolute path to the map file.
        reference_bag_path: The absolute path to the bag file.
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.
    """
    variation_name = f"{variation['sensor_model']}_p{variation['num_particles']}"
    base_dir = (
        Path(RESULTS_PATH) / "benchmarking" / variation_name / f"iter_{iteration}"
    )

    p_beluga = beluga(
        variation["sensor_model"],
        variation["num_particles"],
        map_reference_path,
        dry_mode=dry_mode,
    )
    time.sleep(3)

    p_play = ros_bag_play(
        reference_bag_path, CLOCK_RATE_OPTION + QOS_OPTION, dry_mode=dry_mode
    )

    p_record = ros_bag_record(
        str(base_dir), RECORD_TOPICS_INTERESTED, dry_mode=dry_mode
    )

    wait_for_processes([p_play], [p_beluga, p_record])


def main():
    """Main loop that orchestrates the benchmarking process."""
    for variation in make_variations():
        for it in range(num_iterations):
            run_iteration(
                variation, it, reference_map_path, reference_bag_path, DRY_MODE
            )


if __name__ == "__main__":
    main()
