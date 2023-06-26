# beluga_vs_nav2 package

## Instructions

- Cd to `lambkin` root directory.
```sh
cd {LAMBKIN_ROOT_DIR}
```
- Clone `beluga-datasets`
```sh
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
cd src/beluga_vs_nav2/playground
```

- Run the benchmark itself.
```sh
ros2 run beluga_vs_nav2 beluga_vs_nav2.robot
```
- Inspect the results
```sh
ls beluga_vs_nav2
```
- Find the report
```sh
ls beluga_vs_nav2/report/latex/report.pdf 
```
