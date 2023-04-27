# Getting started

## Crash course

1. **Clone repository**. You will need `git`.

```
git clone https://github.com/ekumenlabs/lambkin.git
```

2. **Build SLAM Toolbox benchmarks suite**. You will need [`docker`](https://docs.docker.com/engine/install/) (with [BuildKit](https://docs.docker.com/build/buildkit/)). It will take ~30 minutes.

```
(cd lambkin && docker buildx bake slam-toolbox-benchmarks)
```

3. **Run SLAM Toolbox benchmarks on Magazino datasets**. You will need [`rocker`](https://github.com/osrf/rocker) (and [`rocknroll`](../.rocker/rocknroll)).

```sh
mkdir playground && cd playground
wget https://storage.googleapis.com/cartographer-public-data/bags/toru/hallway_return.bag
rocker --local --user ekumenlabs/slam-toolbox-benchmarks:latest magazino_benchmark.robot
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

## Developing LAMBKIN

### Development Containers

Most packages in LAMBKIN support [development containers](https://containers.dev/). There is even a full repository development container available. You may use these through [`vscode` IDE](https://code.visualstudio.com/docs/devcontainers/containers) or [`devcontainers` CLI](https://github.com/devcontainers/cli).

### Build Tooling

As an heterogeneous collection of software packages, LAMBKIN does not usually afford single full builds. That said, [`colcon`](https://colcon.readthedocs.io/en/released/) (plus extensions such as `colcon-cmake`, `colcon-ros`, and `colcon-poetry-ros`) will take you a long way.
