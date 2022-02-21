# Getting Started

## Running an existing benchmark

This is an example on how to run an existing benchmark using the provided `slam_toolbox_benchmark` package.

1. Build `lambkin` and `slam_toolbox_benchmark` docker images.

    ```bash
    make -C <PATH_TO_LAMBKIN_INSTALLATION>/src/core/lambkin/docker/
    make -C <PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/
    ```

2. Change to a directory containing the required datasets in ROS 1 bag format. For this example, you'll need the Magazino robot
[hallway_return.bag](https://storage.googleapis.com/cartographer-public-data/bags/toru/hallway_return.bag) dataset from
[Cartographer's Public Data repository](https://google-cartographer-ros.readthedocs.io/en/latest/data.html#magazino).

    ```bash
    cd <PATH_TO_FOLDER_WITH_DATASET>
    ```

    The current directory will be bound to `/root/work` inside the container.

3. Execute the benchmark pipeline with this command:

    ```bash
    <PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot
    ```

4. After the run, you should see the following folder structure in your current directory:

    ```bash
    ├── hallway_return/         # LAMBKIN output files and images
    │   └── runs/
    │       ├── 1/
    │       ├── 2/
    │       ├── 3/
    │       └── ...
    ├── report.pdf              # LAMBKIN benchmark report
    ├── output.xml              # RobotFramework test execution information
    ├── report.html             # RobotFramework test report
    ├── log.html                # RobotFramework logs
    └── hallway_return.bag      # Input dataset
    ```

## Changing an existing benchmark

If you want to change the benchmark definition (amount of iterations, duration of each run, SLAM system parameters, etc.)
or the way benchmarking tools are implemented, in principle, you will need to rebuild the corresponding docker images.

In order to make the development process easier and skip the image rebuilding step, we provide an option to bind mount any modified package using [docker volumes](https://docs.docker.com/storage/volumes/).
For instance, if you are changing the `lambkin` package itself, you could use the following command to run the previous benchmark with your changes:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot --rebuild --bind lambkin:<PATH_TO_LAMBKIN_INSTALLATION>/lambkin
```

The `--rebuild` option will rebuild the `catkin` workspace for the changes to take effect before execution.

There is a shortcut to bind mount the same benchmark package you are running:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot --rebuild --rebind
```

And you can mount more than one package at your convenience:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot ---rebuild --rebind --bind lambkin:<PATH_TO_LAMBKIN_INSTALLATION>/lambkin
```

## Using interactive mode

When developing, we found useful to test commands before adding them to the automation pipeline. Instead of running a `RobotFramework` script,
you can use the `--interactive` flag, which gives you access to a command-line interface attached to a container already configured to run GUI apps. Running the following command on different terminal windows will give you access to the same container:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run --interactive
```

You might want to combine this option with [package binding](#changing-an-existing-benchmark) to use your preferred IDE in the host while testing your changes in the running container:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run --interactive --rebuild --rebind --bind lambkin:<PATH_TO_LAMBKIN_INSTALLATION>/lambkin
```

Once inside the container, you can run a benchmark using the `rosrun` command:

```
root@lambkin_slam_toolbox:~/work# rosrun slam_toolbox_benchmark magazino_benchmark.robot
```

Use this command to rebuild the `catkin` workspace for your changes to take effect:

```
root@lambkin_slam_toolbox:~/work# catkin_make_isolated --install -C /root/ws
```
