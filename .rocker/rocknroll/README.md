## `rocknroll`

A package with extensions for [`rocker`](https://github.com/osrf/rocker).

### Installation

As usual for a Python package:

```sh
cd path/to/rocknroll
pip install -U .
```

### Extensions

#### Local

This extension eases working directory bind mounts. Best used along with `--user`.

```sh
mkdir -p /tmp/ws
cd /tmp/ws
rocker --local --user ubuntu:20.04
```

This snippet will land you in a `/tmp/ws` bind mount within the container.
