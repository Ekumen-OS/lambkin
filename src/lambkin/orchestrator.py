#!/usr/bin/env python3


import sys
import unittest
import subprocess
import time
from pathlib import Path
# User defined variables
input_file = "/path/to/rosbag/input.mcap"
output_path = "/path/to/rosbag/output.mcap"
num_iterations = 2


# Options for the benchmark
laser_models = ["likelihood", "beam"]
num_particles = [1, 10, 100, 1000, 10000]
clock_rate_options = [10, 50, 100]
topics_interested = ["/pose", "/tf", "/tf_tatic"]

def execute_background_process(full_cmd_list: list[str]) :
    print(f"{full_cmd_list}\n")
    return subprocess.Popen(full_cmd_list)
    

def ros_bag_record(output_path:str, options: list[str]) -> subprocess.Popen:
    return execute_background_process(["ros2", "bag", "record", "-o", output_path] + options)
    
def ros_bag_play(input_path: str, options: list[str]) -> subprocess.Popen:
    return execute_background_process(["ros2", "bag", "play",input_path] + options)
    
def beluga(sensor_model, num_particles, ) -> subprocess.Popen:
    cmd_list = [ "ros2", "launch", "beluga_example","localization.launch.py" , f"laser_model_type:={sensor_model}", f"max_particles:={num_particles}" ]
    return execute_background_process(cmd_list)

def bag2tum(record_path: str, output_tum_path: str, topics) -> str:
    Path(output_tum_path).parent.mkdir(parents=True, exist_ok=True)
    cmd_list = [
        "evo_traj", "bag2", record_path, topics, 
        "--save_as_tum"
    ]
    # subprocess.run(cmd_list, check=True)   
    return output_tum_path


def evo_ape(reference_path: str, compare_bag_path: str, output_zip_path: str):
    tum_path = compare_bag_path.replace(".bag", ".tum")  
    compare_path = bag2tum(compare_bag_path, tum_path)
    Path(output_zip_path).parent.mkdir(parents=True, exist_ok=True)
    cmd_list = [
        "evo_ape", "tum", 
        reference_path, compare_path, 
        "-va", "--align", 
        "--save_results", output_zip_path
    ]
    # subprocess.run(cmd_list, check=True)


def wait_for_processes(waitlist: list[subprocess.Popen], termination_list: list[subprocess.Popen]):

    for p in waitlist:
        p.wait()   
    for p in termination_list:
        p.terminate()    
    for p in termination_list:
        try:
            p.wait(timeout=5.0) 
        except subprocess.TimeoutExpired:
            print("Forzando cierre de un proceso ...")
            p.kill()
    print("Procesos cerrados de forma segura.\n")


    
def plot_ape_metrics(path_ape):
    cmd_list = ["evo_res", path_ape]
    #subprocess.run(cmd_list)
    print(f"{cmd_list}\n")


def make_variations() -> list:
    variations = []
    for sensor_model in laser_models:
        for particles in num_particles:
            for rate in clock_rate_options:
                variations.append({
                    "sensor_model": sensor_model,
                    "num_particles" : particles, 
                    "clock_rate" : rate
                })
    return variations
                

def run_iteration(variation: dict, iteration):
    nombre_variacion = f"{variation["sensor_model"]}_p{variation["num_particles"]}"
    base_dir = Path("benchmarking") / nombre_variacion / f"iter_{iteration}"
    bag_dir = base_dir / "bag"
    bag_dir.mkdir(parents=True, exist_ok=True)
    beluga(variation["sensor_model"], variation["num_particles"])
    ros_bag_play("/ws/beluga/beluga_example/bags/hallway_localization", ["--qos-profile-overrides-path /ws/beluga/beluga_example/bags/qos_override.yaml"])
    ros_bag_record("/ws/lamkin/results/output", ["/tf", "/pose", "/tf_static", "--clock_rate", str(variation["clock_rate"])])



class TestProcessManagement(unittest.TestCase):
    def test_execute_background_process(self):
        
        proceso = execute_background_process(["sleep", "3"])

        self.assertIsNone(proceso.poll(), "El proceso debería estar ejecutándose en segundo plano.")
        
        proceso.terminate()
        proceso.wait()
        pass

    def test_wait_for_processes(self):
        p_play = subprocess.Popen(["sleep", "2"]) 
        p_record = subprocess.Popen(["sleep", "10"]) 
        
        inicio = time.time()
        wait_for_processes(waitlist=[p_play], termination_list=[p_record])
        
        fin = time.time()
        tiempo_total = fin - inicio
        self.assertGreaterEqual(tiempo_total, 2.0, "La función no esperó a que terminara el waitlist.")
        self.assertLess(tiempo_total, 5.0, "La función se quedó bloqueada con el proceso infinito.")

        self.assertIsNotNone(p_record.poll(), "El proceso de termination_list no fue cerrado correctamente.")
        pass

def main():
    print("ejecucion")
    # for variation in make_variations():
    #     for it in range(num_iterations):
    #         print(f"{variation}\n")
    #         run_iteration(variation)
    #evo_ape(3)
    #plot_ape_metrics(3)
    #plot_ape_metrics(variation)


if __name__ == "__main__":

    if len(sys.argv) > 1 and sys.argv[1] == "test":
       
        sys.argv.pop() 
        unittest.main()
        
    else:
        main()
