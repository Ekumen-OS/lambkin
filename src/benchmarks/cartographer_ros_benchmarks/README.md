# `cartographer_ros_benchmarks`

This package bundles sample benchmarks of [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/) against common public domain datasets.

## How to run

1. **Clone repository**. You will need `git`.

```
git clone https://github.com/ekumenlabs/lambkin.git
```

2. **Build SLAM Toolbox benchmarks suite**. You will need [`earthly`](https://docs.earthly.dev/). It will take ~30 minutes.

```
(cd lambkin && earthly ./src/benchmarks/cartographer_ros_benchmarks+release)
```

3. **Run SLAM Toolbox benchmarks on Magazino datasets**. You will need [`rocker`](https://github.com/osrf/rocker) (and [`rocknroll`](../.rocker/rocknroll)).

```sh
mkdir playground && cd playground
wget https://storage.googleapis.com/cartographer-public-data/bags/toru/hallway_return.bag
rocker --local --user ekumenlabs/cartographer-ros-benchmarks:latest magazino_benchmark.robot
```

After this run, you should see:

```
magazino_benchmark
├── cases
│   └── hallway_return
│       ├── metadata.json
│       └── variations
│           └── 1
│               ├── iterations
│               │   ├── 1
│               │   │   ├── metadata.json
│               │   │   ├── logs
│               │   │   ├── map.pgm
│               │   │   ├── map.yaml
│               │   │   ├── output.bag
│               │   │   └── ...
│               │   └── ...
│               └── ...
├── log.html
├── logs
│   ├── setup
│   └── teardown
├── output.xml
├── report
│   ├── latex
│   │   ├── report.pdf
│   │   └── ...
│   └── ...
└── report.html
```
