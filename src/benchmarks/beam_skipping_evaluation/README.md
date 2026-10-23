# beam_skipping_evaluation package

## Instructions

- Cd to `lambkin` root directory.
```sh
cd {LAMBKIN_ROOT_DIR}
```
- Build the beam_skipping_evaluation `devcontainer` image
```sh
./tools/setup.sh
./tools/earthly ./src/benchmarks/beam_skipping_evaluation+local-devel
```
- Clone `beluga-datasets`. **This repository uses LFS**, so make sure you have it installed.
```bash
cd src/benchmarks/beam_skipping_evaluation
mkdir -p playground && cd playground
```
- Copy in that folder the datasets from the `/srv/datasets/beluga_evaluation_datasets` directory in the beefy machine.
```sh
- Open beam_skipping_evaluation `devcontainer` using either its CLI or `vscode`
```
docker compose -f .devcontainer/docker-compose.yml run devcontainer
```
- Build
```sh
BUILD_DOCUMENTATION=false BUILD_TESTING=false colcon build --packages-up-to beam_skipping_evaluation --symlink-install
source install/setup.bash
```
- Go to the `playgrounds` directory, where you've downloaded the datasets
```sh
cd src/lambkin/benchmarks/beam_skipping_evaluation/playground
```
- Run the benchmark itself.
```sh
ros2 run beam_skipping_evaluation nominal.robot
```
- Inspect the results using the same name of the launch file.
```sh
ls nominal
```
- The report can be found in the `report` directory.
```sh
ls nominal/report/build/latex/report.pdf
```
