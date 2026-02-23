#!/usr/bin/env python3


import sys
import unittest
import subprocess
import time
from pathlib import Path
# User defined variables
input_file = "/ws/lambkin/input/hallway_localization"
output_path = "/ws/lambkin/output/"
num_iterations = 0
clock_rate = 10
qos_file_path = "/ws/beluga/beluga_example/bags/qos_override.yaml"
# Options for the benchmark
laser_models = ["likelihood", "beam"]
num_particles = [1, 10, 100, 1000, 10000]
clock_rate_options = ["--clock-rate", str(clock_rate)]
qos_options= ["--qos-profile-overrides-path", qos_file_path]
clock_option = "--clock"
topics_interested = ["/pose", "/tf", "/tf_static"]

def execute_background_process(full_cmd_list: list[str], log_file: str = None) -> subprocess.Popen :
    print(f"{full_cmd_list}\n")
    if log_file:
        file = open(log_file, "w")
        return subprocess.Popen(full_cmd_list,stdout=file,stderr=file) 
    return subprocess.Popen(full_cmd_list)
    

def ros_bag_record(output_path:str, options: list[str]) -> subprocess.Popen:
    bag_dir = output_path / "bag"
    bag_dir.mkdir(parents=True, exist_ok=True)
    return execute_background_process(["ros2", "bag", "record", "-o", output_path ]+ options)
    
def ros_bag_play(input_path: str, options: list[str]) -> subprocess.Popen:
    return execute_background_process(["ros2", "bag", "play",input_path ]+ options)
    
def beluga(sensor_model, num_particles,map_path: str ) -> subprocess.Popen:
    cmd_list = [ "ros2", "launch", "beluga_example","localization_launch.py" , 
                f"localizacion_map:={map_path}" , 
                f"laser_model_type:={sensor_model}", 
                f"max_particles:={num_particles}" ]
    return execute_background_process(cmd_list, log_file="beluga.log")

def bag2tum(bag_path: str, tum_path: str, topic: str) -> str:
    tum_dir = Path(tum_path) / "tum"
    tum_dir.mkdir(parents=True, exist_ok=True)

    cmd_list = [
        "evo_traj", "bag2", str(bag_path), topic, 
        "--save_as_tum"
    ]
    subprocess.run(cmd_list, cwd=tum_dir, check=True)   
    
    tum_name = f"{topic.strip('/')}.tum"
    return str(tum_dir / tum_name)


def evo_ape(reference_path: str, record_path: str, topic: str = "/pose"):   
    
    iter_dir = record_path.parent.parent
    
    record_tum = bag2tum(str(record_path), str(iter_dir), topic)
    
    ape_dir = iter_dir / "ape"
    ape_dir.mkdir(parents=True, exist_ok=True)

    cmd_list = [
        "evo_ape", "tum", 
        reference_path, record_tum, 
        "-va", "--align", 
        "--save_results", str(ape_dir / "ape.zip") 
    ]
    
    subprocess.run(cmd_list, check=True)


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


    
def plot_ape_metrics(path_ape):
    cmd_list = ["evo_res", path_ape]
    print(f"{cmd_list}\n")
    subprocess.run(cmd_list)
   


def make_variations() -> list:
    variations = []
    for sensor_model in laser_models:
        for particles in num_particles:
            variations.append({
                "sensor_model": sensor_model,
                "num_particles" : particles, 
            })
    return variations
                

def run_iteration(variation: dict, iteration):
    nombre_variacion = f"{variation['sensor_model']}_p{variation['num_particles']}"
    base_dir = Path(output_path) / "benchmarking" / nombre_variacion / f"iter_{iteration}"

    
    p_beluga = beluga(variation["sensor_model"], variation["num_particles"], "/ws/lambkin/input/map.yaml")
    time.sleep(3)

    p_play = ros_bag_play(input_file, clock_rate_options + qos_options)
    
    p_record = ros_bag_record(str(base_dir), topics_interested)
    
    wait_for_processes([p_play], [p_beluga, p_record])



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

    for variation in make_variations():
        for it in range(num_iterations):
            run_iteration(variation, it)
            time.sleep(2) 

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        sys.argv.pop() 
        unittest.main()
    else:
        main()