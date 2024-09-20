# beluga_vs_nav2_multi_dataset package

## Instructions

- Cd to `lambkin` root directory.
```sh
cd {LAMBKIN_ROOT_DIR}
```
- Build the beluga_vs_nav2_multi_dataset `devcontainer` image
```sh
./tools/setup.sh
./tools/earthly ./src/benchmarks/beluga_vs_nav2_multi_dataset+local-devel
```
- Clone `beluga-datasets`. **This repository uses LFS**, so make sure you have it installed.
```bash
cd src/benchmarks/beluga_vs_nav2_multi_dataset \
    && mkdir -p playground && cd playground
```
- Copy in that folder the datasets from the `/srv/datasets/beluga_evaluation_datasets` directory in the beefy machine.
```sh
- Open beluga_vs_nav2_multi_dataset `devcontainer` using either its CLI or `vscode`
```
docker compose -f .devcontainer/docker-compose.yml run devcontainer
```
- Build
```sh
BUILD_DOCUMENTATION=false BUILD_TESTING=false colcon build --packages-up-to beluga_vs_nav2_multi_dataset --symlink-install
source install/setup.bash
```
- Go to the `playgrounds` directory, where you've downloaded the datasets
```sh
cd src/lambkin/benchmarks/beluga_vs_nav2_multi_dataset/playground
```
- Run the benchmark itself.
```sh
rm -rf nominal/.cache && ros2 run beluga_vs_nav2_multi_dataset nominal.robot
```
- Inspect the results using the same name of the launch file.
```sh
ls nominal
```
- The report can be found in the `report` directory.
```sh
ls nominal/report/build/latex/report.pdf
```
