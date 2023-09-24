# Cartographer tooling and useful recipes

Tools and recipes to make the most out of the [`cartographer`](https://github.com/ros2/cartographer) and [`cartographer_ros`](https://github.com/ros2/cartographer_ros) ROS 2 packages.

## Recover map and groundtruth from ROS 2 bags

1. **Perform mapping offline using Cartographer ROS**. For this you will need a suitable configuration for Cartographer. Both a [configuration reference](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#algorithm-walkthrough-for-tuning) and a [tuning guide](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html) are available online. Alternatively, there are example configurations both upstream, in the [`cartographer_ros`](https://github.com/ros2/cartographer_ros/tree/ros2/cartographer_ros/configuration_files) repository, and in [this package](./configuration_files). Use at your own risk.

```sh
ros2 run cartographer_ros cartographer_offline_node -configuration_directory path/to/cartographer_tools/configuration_files -configuration_basenames narrow_spaces_2d.lua -bag_filenames path/to/bagfile_0 [path/to/bagfile_1 ...] -save_state_filename posegraph.pbstream
```

will output a single serialized pose graph, containing one trajectory per bagfile. This is useful to leverage multiple, spatially overlapping recordings.

Note you may have to remap topic names if those in your bagfile do not match [Cartographer ROS conventions](https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html#create-launch-files-for-your-slam-scenarios), as in:

```sh
ros2 run cartographer_ros cartographer_offline_node -configuration_directory path/to/cartographer_tools/configuration_files -configuration_basenames narrow_spaces_2d.lua -bag_filenames path/to/bagfile [path/to/bagfile_1 ...] -save_state_filename posegraph.pbstream --ros-args -r /my/laser/scan:=/scan
```

2. **Recover occupancy grid map from pose graph**:

```sh
ros2 run cartographer_ros cartographer_pbstream_to_ros_map -map_filestem map -pbstream_filename posegraph.pbstream
```

will output both `map.pgm` and `map.yaml` files, suitable for [`nav2_map_server`](https://github.com/ros-planning/navigation2/tree/main/nav2_map_server) configuration.

You may also want to post-process the map image to remove artifacts and blurred areas. [`ImageMagick` tooling](https://legacy.imagemagick.org/Usage/masking/) can be handy in such cases:

```sh
mv map.pgm map.pgm.orig
convert map.pgm.orig \( +clone -threshold 65% \) -compose plus -composite map.pgm
```

3. **Recover ground truth trajectory from pose graph**:

```sh
ros2 run cartographer_tools pbstream_to_tum_trajectory -pbstream_filename posegraph.pbstream -trajectory_filestem groundtruth
```

will output TUM trajectory files, one per trajectory in the pose graph.
