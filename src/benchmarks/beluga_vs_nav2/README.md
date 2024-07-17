# beluga_vs_nav2 package

## Instructions

- Cd to `lambkin` root directory.
```sh
cd {LAMBKIN_ROOT_DIR}
```
- Build the beluga_vs_nav2 `devcontainer` image
```sh
./tools/setup.sh
./tools/earthly ./src/benchmarks/beluga_vs_nav2+local-devel
```
- Clone `beluga-datasets`. **This repository uses LFS**, so make sure you have it installed.
```sh
sudo apt install git-lfs
cd src/benchmarks/beluga_vs_nav2
mkdir -p playground && cd playground
git clone git@github.com:Ekumen-OS/beluga-datasets.git
```
- Open beluga_vs_nav2 `devcontainer` using either its CLI or `vscode`.
- Build 
```sh
BUILD_DOCUMENTATION=false BUILD_TESTING=false colcon build --packages-up-to beluga_vs_nav2 --symlink-install
source install/setup.bash
```
- Cd to `playgrounds` dir, where you've downloaded `beluga-datasets`

```sh
cd src/lambkin/benchmarks/beluga_vs_nav2/playground
```

- Run the benchmark itself.
```sh
ros2 run beluga_vs_nav2 nominal.robot
ros2 run beluga_vs_nav2 ndt_swept.robot
ros2 run beluga_vs_nav2 swept.robot
```
- Inspect the results using the same name of the launch file. For example, if you ran `nominal.robot`, you can inspect the results by running:
```sh
ls nominal
```
- The report can be found in the `report` directory. For example, if you ran `nominal.robot`, you can inspect the `pdf` report by running:
```sh
ls nominal/report/build/latex/report.pdf
```
