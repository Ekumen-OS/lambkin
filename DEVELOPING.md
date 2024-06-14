# Developing LAMBKIN

## Table of Contents

- [Environment](#environment)
- [Workflow](#workflow)

## Environment

Most packages in LAMBKIN support [development containers](https://containers.dev/) and `docker compose`-managed containers. This is also true for the entire repository. You may use the former through the [`vscode` IDE](https://code.visualstudio.com/docs/devcontainers/containers) or [`devcontainers` CLI](https://github.com/devcontainers/cli), or the latter with `docker compose run`. Either way, development container images must be built first. That is where [`earthly`](https://docs.earthly.dev/) enters the scene.

For instance, to bring up the root development environment:

1. **Clone the repository**. You will need `git`.

   ```bash
   git clone --recursive git@github.com:ekumenlabs/lambkin.git
   ```

2. **Build the root development container**. You will need [`earthly 0.8+`](https://earthly.dev/get-earthly).

   ```bash
   cd lambkin
   earthly +local-ubuntu-devel
   ```

3. **Run the root development container**. You may use `devcontainers` tooling or plain `docker-compose`:
    
   ```bash
   devcontainer --workspace-folder . up
   ```
   
   ```bash
   docker compose run -f .devcontainer/docker-compose.yml
   ```

## Workflow

As an heterogeneous collection of software packages, LAMBKIN does not usually afford single full builds. Every package will likely require impose a workflow of its own, specially third-party ones. That said, [`colcon`](https://colcon.readthedocs.io/en/released/) (plus extensions such as `colcon-cmake`, `colcon-ros`, and `colcon-poetry-ros`) will take you a long way.

## Next Steps

- If you want to contribute to this project, please read the [contribuing guidelines](CONTRIBUTING.md) first.
