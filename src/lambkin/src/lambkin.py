#!/usr/bin/env python3
"""Lambkin library for executing and benchmarking Beluga AMCL."""

import subprocess
import time
from pathlib import Path

input_file = "/ws/lambkin/input/hallway_localization"
output_path = "/ws/lambkin/output/"
num_iterations = 1
clock_rate = 10
qos_file_path = "/ws/beluga/beluga_example/bags/qos_override.yaml"
laser_topic = "/scan_front"

Laser_models = ["likelihood", "beam"]
Num_particles = [1, 10, 100, 1000, 10000]
Clock_rate_options = ["--clock-rate", str(clock_rate)]
Qos_options = ["--qos-profile-overrides-path", qos_file_path]
Clock_option = "--clock"
Record_topics_interested = ["/pose", "/tf", "/tf_static"]


def execute_background_process(
    full_cmd_list: list[str], log_file: str = None
) -> subprocess.Popen:
    """Executes a shell command in the background.

    Args:
        full_cmd_list (list[str]): The command and its arguments as a list of strings.
        log_file (str, optional): Path to a output log file.

    Returns:
        subprocess.Popen: The running background process.
    """
    print(f"{full_cmd_list}\n")
    if log_file:
        file = open(log_file, "w")
        return subprocess.Popen(full_cmd_list, stdout=file, stderr=file)
    return subprocess.Popen(full_cmd_list)


def execute_foreground_process(
    full_cmd_list: list[str], log_file: str = None
) -> subprocess.run:
    """Executes a shell command in the foreground.

    Args:
        full_cmd_list (list[str]): The command and its arguments as a list of strings.
        log_file (str, optional): Path to a output log file.

    Returns:
        subprocess.run: The running foreground process.
    """
    if log_file:
        file = open(log_file, "w")
        return subprocess.run(full_cmd_list, stdout=file, stderr=file, check=True)
    return subprocess.run(full_cmd_list, check=True)


def ros_bag_record(output_path: str, options: list[str]) -> subprocess.Popen:
    """Starts a ROS 2 process to record a bag file.

    Args:
        output_path (str): The base directory where the bag will be saved.
        options (list[str]): Additional options and topics.

    Returns:
        subprocess.Popen: The running ros2 bag record process.
    """
    bag_dir = Path(output_path) / "bag"
    bag_dir.mkdir(parents=True, exist_ok=True)
    return execute_background_process(
        ["ros2", "bag", "record", "-o", str(output_path)] + options
    )


def ros_bag_play(input_path: str, options: list[str]) -> subprocess.Popen:
    """Starts a ROS 2 process to play a bag file.

    Args:
        input_path (str): The path to the input bag file.
        options (list[str]): Additional options for playback (e.g., clock rate, QoS).

    Returns:
        subprocess.Popen: The running ros2 bag play process.
    """
    cmd_list = ["ros2", "bag", "play", input_path] + options
    return execute_background_process(cmd_list)


def beluga(
    sensor_model: str, num_particles: int, map_path: str, laser_topic: str
) -> subprocess.Popen:
    """Launches the Beluga AMCL node using a custom launch file.

    Args:
        sensor_model (str): The laser sensor model to use.
        num_particles (int): The maximum number of particles for AMCL.
        map_path (str): The absolute path to the map file.
        laser_topic (str): The topic for the laser sensor

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
        f"max_particles:={num_particles}laser_topic:={laser_topic}",
    ]
    return execute_background_process(cmd_list, log_file="beluga.log")


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


def evo_ape(reference_path: str, record_path: str, topic: str = "/pose"):
    """Computes the Absolute Pose Error (APE).

    Args:
        reference_path (str): The path to the reference TUM file.
        record_path (str): The path to the recorded bag file.
        topic (str, optional): The topic to evaluate. Defaults to "/pose".
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
    execute_foreground_process(cmd_list)


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
        p.wait()
    for p in termination_list:
        p.terminate()
    for p in termination_list:
        try:
            p.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            print("Forcing process termination...")
            p.kill()


def plot_ape_metrics(ape_path: str) -> None:
    """Plots the Absolute Pose Error (APE) metrics using evo_res.

    Args:
        ape_path (str): The path to the results zip file generated by evo_ape.
    """
    cmd_list = ["evo_res", ape_path]
    print(f"{cmd_list}\n")
    execute_foreground_process(cmd_list)


def make_variations() -> list:
    """Generates all possible combinations for the benchmark.

    It creates configurations of sensor models and particle counts.

    Returns:
        list: A list of dictionaries containing configuration pairs.
    """
    variations = []
    for sensor_model in Laser_models:
        for particles in Num_particles:
            variations.append(
                {
                    "sensor_model": sensor_model,
                    "num_particles": particles,
                }
            )
    return variations


def run_iteration(
    variation: dict, iteration: int, map_reference_path: str, laser_topic: str
) -> None:
    """Executes a single benchmark iteration with a specific configuration.

    Args:
        variation (dict): The configuration dictionary containing the
            sensor model and particle count.
        iteration (int): The current iteration number.
        map_reference_path (str): The absolute path to the map file.
        laser_topic (str): The topic for the laser sensor
    """
    variation_name = f"{variation['sensor_model']}_p{variation['num_particles']}"
    base_dir = Path(output_path) / "benchmarking" / variation_name / f"iter_{iteration}"

    p_beluga = beluga(
        variation["sensor_model"],
        variation["num_particles"],
        map_reference_path,
        laser_topic,
    )
    time.sleep(3)

    p_play = ros_bag_play(input_file, Clock_rate_options + Qos_options)

    p_record = ros_bag_record(str(base_dir), Record_topics_interested)

    wait_for_processes([p_play], [p_beluga, p_record])


def main():
    """Main loop that orchestrates the benchmarking process."""
    for variation in make_variations():
        for it in range(num_iterations):
            run_iteration(variation, it, laser_topic)


if __name__ == "__main__":
    main()
