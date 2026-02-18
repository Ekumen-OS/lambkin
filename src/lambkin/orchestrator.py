#!/usr/bin/env python3



# User defined variables
input_file = "/path/to/rosbag/input.mcap"
output_path = "/path/to/rosbag/output.mcap"
num_iterations = 3
clock_rate = 100

# Options for the benchmark
sensor_model = ["likelihood", "beam"]
num_particles = [1, 10, 100, 1000, 10000]
clock_rate_options = f"--clock --rate {clock_rate}"
topics_interested = ["/pose", "/tf", "/tf_tatic"]

def execute_background_process(full_cmd_list: list[str]) :
    # Implement me
    # Mock
    print(f"{full_cmd_list}\n")

def ros_bag_record(output_path:str, options: list[str]):

    return execute_background_process(["ros2", "bag", "record", "-o", output_path] + options)
    
def ros_bag_play(input_path: str, options: list[str]):
    return execute_background_process(["ros2", "bag", "play",input_path] + options)
    
def beluga(sensor_model, num_particles, ):
    
    cmd_list = [ "ros2", "launch", "beluga_example","localization.launch.py" , f"laser_model_type:={sensor_model}", f"max_particles:={num_particles}" ]
    
    return execute_background_process(cmd_list)

def evo_tum(record_name):
    cmd_list = [ "evo_traj","bag2" , f"bags/{record_name}", "--save_as_tum" , "--save_to", f"results/ficheros_tum/{record_name}" ]
    # implement me 
def evo_ape():
    cmd_list = [ "ros2", "launch", "beluga_example","localization.launch.py" , f"laser_model_type:={sensor_model}", f"max_particles:={num_particles}" ]
    # implement me 

# def wait_for_processes(waitlist, termination_list):
#     # implement me
    
# def plot_ape_metrics():
#     # implement me

# def make_variations() -> list:
#     # implement me
#     # make all combinations
#     for loop sensor_model in models:
#         for
#             for
#                 append("sensor_model": sensor_model,
#                 ...

def run_iteration():
#     # Orchestrator one run_iteration
#     # Run rosbag record/play, beluga
    beluga("likelihood_field", 1000)
    ros_bag_play("/ws/beluga/beluga_example/bags/hallway_localization", ["--qos-profile-overrides-path /ws/beluga/beluga_example/bags/qos_override.yaml"])
    ros_bag_record("/ws/lamkin/results/output", ["/tf", "/pose", "/tf_static"])
#     # Wait for processes.
#     # Run evo
#     ...

def main():
    for loop in range(num_iterations):
        run_iteration()
    # for variation in make_variations():
    #     for loop num_iterations:
    #         run_iteration(variation)
    # plot_ape_metrics(variation)

if __name__ == "__main__":
    main()