#!/usr/bin/env python3

"""Lambkin library to execute and benchmark Localization and Mapping algorithms."""

import shutil
import subprocess
import time
from pathlib import Path

REFERENCE_BAG_PATH = "/rosbags/reference/hq_files/hq_simulation_segment_0"
REFERENCE_MAP_PATH = "/rosbags/reference/hq_files/map.yaml"
REFERENCE_TUM_PATH = "/rosbags/reference/hq_files/groundtruth.tum"
LOG_PATH = "/ws/log"
NUM_ITERATIONS = 1
RATE = 1
QOS_FILE_PATH = "/rosbags/reference/qos_override.yaml"
BELUGA_READY_DELAY = 3

LASER_MODELS = ["likelihood_field", "beam"]
NUM_PARTICLES = [1, 10, 1000, 2000]
RATE_OPTION = ["--rate", str(RATE)]
QOS_OPTION = ["--qos-profile-overrides-path", QOS_FILE_PATH]
CLOCK_OPTION = ["--clock"]
RECORD_TOPICS_INTERESTED = ["/pose", "/tf", "/tf_static"]
RECORD_TOPICS_OPTION = ["--topics"] + RECORD_TOPICS_INTERESTED
RESULTS_PATH = "/benchmarking_results"
DRY_MODE = False
PROCESS_TERMINATION_TIMEOUT = 7.0
APE_TOPICS_INTERESTED = "/pose"


def kill_ros2_nodes() -> None:
    """Kill any orphaned ROS2 nodes before starting the benchmark."""
    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
    subprocess.run(["ros2", "daemon", "start"], capture_output=True)
    time.sleep(1)

    result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)

    nodes = []
    for line in result.stdout.splitlines():
        node = line.strip()
        if node:
            nodes.append(node)

    if not nodes:
        print("No orphaned ROS2 nodes found.")
        return

    print(f"Found orphaned nodes: {nodes}")

    for node in nodes:
        subprocess.run(["pkill", "-f", node.lstrip("/")], capture_output=True)

    time.sleep(7.0)

    # Verify they are gone
    result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
    remaining = result.stdout.strip()
    if remaining:
        print(f"Warning: some nodes are still alive: {remaining}")
    else:
        print("All orphaned nodes cleaned up successfully.")


def execute_background_process(
    full_cmd_list: list[str], dry_mode: bool = False, log_file: str = None
) -> subprocess.Popen:
    """Executes a command as a background process and performs an instant death check.

    Spawns the given command as a non-blocking subprocess, optionally redirecting
    its output to a log file. After spawning, performs a quick poll to detect
    immediate failures (e.g. command not found, missing package).

    Args:
        full_cmd_list (list[str]): The command and its arguments as a list of strings.
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.
        log_file (str, optional): Filename for stdout/stderr redirection, saved under
                                  LOG_PATH. If None, output is not redirected.
                                  Defaults to None.

    Returns:
        subprocess.Popen: The running background process, or None if dry_mode is True.

    Raises:
        RuntimeError: If the process dies instantly with a non-zero return code
                      after being spawned.
    """
    if dry_mode:
        print(f"{full_cmd_list}\n")
        return None

    if log_file:
        log_path_parent = Path(LOG_PATH)
        log_path_parent.mkdir(parents=True, exist_ok=True)
        log_path = log_path_parent / log_file
        file = open(log_path, "w")
        process = subprocess.Popen(full_cmd_list, stdout=file, stderr=subprocess.STDOUT)
    else:
        process = subprocess.Popen(full_cmd_list)

    # Quick check in case the process died instantly
    process.poll()
    if process.returncode is not None and process.returncode != 0:
        raise RuntimeError(
            f"Background process died instantly with return code "
            f"{process.returncode}.\n"
            f"Command: {full_cmd_list}"
        )
    return process


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

    return subprocess.run(full_cmd_list, check=True, input="y\n", text=True)


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

    if bag_dir.exists():
        shutil.rmtree(bag_dir)
        print(f"Removing existing directory at {bag_dir}...")

    return execute_background_process(
        (["ros2", "bag", "record", "-o", str(bag_dir)] + options),
        dry_mode,
        log_file="bag_record.log",
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
    return execute_background_process(cmd_list, dry_mode, log_file="bag_play.log")


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
        "lambkin_ros2",
        "beluga.launch.py",
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
    subprocess.run(cmd_list, cwd=tum_dir, check=True, input="y\n", text=True)

    tum_name = f"{topic.strip('/')}.tum"
    return str(tum_dir / tum_name)


def evo_ape(
    reference_path: str,
    record_path: str,
    ape_path: str,
    topic: str = "/pose",
    dry_mode: bool = False,
):
    """Computes the Absolute Pose Error (APE).

    Args:
        reference_path (str): The path to the reference TUM file.
        record_path (str): The path to the recorded bag file.
        ape_path (str): The path to the ape file
        topic (str, optional): The topic to evaluate. Defaults to "/pose".
        dry_mode (bool, optional): If True, prints the command without executing it.
                                   Defaults to False.
    """
    record_path_obj = Path(record_path)
    iter_dir = record_path_obj.parent

    record_tum = bag2tum(str(record_path_obj), str(iter_dir), topic)
    Path(ape_path).parent.mkdir(parents=True, exist_ok=True)

    cmd_list = [
        "evo_ape",
        "tum",
        reference_path,
        record_tum,
        "-va",
        "--align",
        "--save_results",
        str(ape_path),
    ]
    execute_foreground_process(cmd_list, dry_mode)


def wait_for_processes(
    waitlist: list[subprocess.Popen], termination_list: list[subprocess.Popen]
) -> None:
    """Manages process synchronization and background monitoring.

    Waits for specific processes to complete while monitoring background
    processes for unexpected failures. Continuously polls all processes
    while waiting for the waitlist to finish, detecting any unexpected
    crashes in the termination_list before sending SIGTERM.

    Args:
        waitlist (list[subprocess.Popen]): Processes that must complete naturally
                                           before termination_list is stopped.
        termination_list (list[subprocess.Popen]): Long-running background processes
                                                   that will be terminated once the
                                                   waitlist finishes.

    Raises:
        RuntimeError: If any process in the termination_list dies unexpectedly
                      during execution, or if any process in the waitlist exits
                      with a non-zero return code.
    """
    # Wait for waitlist processes to finish naturally and check their return codes
    for p in waitlist:
        if p is not None:
            p.wait()
            if p.returncode != 0:
                raise RuntimeError(
                    f"Process {p.args} failed with return code {p.returncode}."
                )

    # Check termination_list processes didn't die unexpectedly before we kill them
    for p in termination_list:
        if p is not None:
            p.poll()
            if p.returncode is not None and p.returncode != 0:
                raise RuntimeError(
                    f"Process {p.args} died unexpectedly with "
                    f"return code {p.returncode}."
                )

    # Broadcast SIGTERM to all termination_list processes
    for p in termination_list:
        if p is not None:
            p.terminate()

    # Wait up to timeout for them to close; force-kill if they hang
    for p in termination_list:
        if p is not None:
            try:
                p.wait(timeout=PROCESS_TERMINATION_TIMEOUT)
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
    plot_file = Path(RESULTS_PATH) / "plots_ape" / "ape_comparison_plot.png"

    if plot_file.parent.exists():
        plot_file.unlink()
    plot_file.parent.mkdir(parents=True, exist_ok=True)

    cmd_list = (
        ["evo_res"] + ape_path + ["--use_filenames", "--save_plot", str(plot_file)]
    )
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
    print(f"Running iteration {iteration} with {variation_name}")
    base_dir = Path(RESULTS_PATH) / variation_name / f"iter_{iteration}"

    p_beluga = beluga(
        variation["sensor_model"],
        variation["num_particles"],
        map_reference_path,
        dry_mode=dry_mode,
    )
    time.sleep(BELUGA_READY_DELAY)

    p_record = ros_bag_record(str(base_dir), RECORD_TOPICS_OPTION, dry_mode=dry_mode)
    p_play = ros_bag_play(
        reference_bag_path, CLOCK_OPTION + RATE_OPTION + QOS_OPTION, dry_mode=dry_mode
    )

    wait_for_processes([p_play], [p_beluga, p_record])
    kill_ros2_nodes()


def main():
    """Main loop that orchestrates the benchmarking process."""
    ape_directories = []
    kill_ros2_nodes()
    for variation in make_variations():
        for it in range(NUM_ITERATIONS):
            run_iteration(
                variation=variation,
                iteration=it,
                map_reference_path=REFERENCE_MAP_PATH,
                reference_bag_path=REFERENCE_BAG_PATH,
                dry_mode=DRY_MODE,
            )
            variation_name = (
                f"{variation['sensor_model']}_p{variation['num_particles']}"
            )
            bag_dir = Path(RESULTS_PATH) / variation_name / f"iter_{it}" / "bag"
            ape_dir = (
                Path(RESULTS_PATH)
                / variation_name
                / f"iter_{it}"
                / "ape"
                / f"ape_{variation_name}_iter_{it}.zip"
            )
            evo_ape(
                reference_path=REFERENCE_TUM_PATH,
                record_path=str(bag_dir),
                ape_path=str(ape_dir),
                topic=APE_TOPICS_INTERESTED,
                dry_mode=DRY_MODE,
            )

            ape_directories.append(str(ape_dir))

    plot_ape_metrics(ape_path=ape_directories, dry_mode=DRY_MODE)


if __name__ == "__main__":
    main()
