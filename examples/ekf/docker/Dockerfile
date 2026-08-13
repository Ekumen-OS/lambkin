# Image supplying the reference data at /data/. Override to point at your own
# recording, e.g.:
#   docker compose build --build-arg DATASET_IMAGE=my-datasets:latest
# See ../README.md#dataset-requirements for the topics the bag must carry.
ARG DATASET_IMAGE=docker.io/ekumenlabs/lambkin-beluga-datasets:jazzy

FROM ${DATASET_IMAGE} AS datasets

# ── Stage 0: base ─────────────────────────────────────────
FROM docker.io/ros:jazzy-ros-base AS base
RUN apt-get update && apt-get install -y \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR="/usr/bin" sh

RUN rosdep init || true \
    && rosdep update

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# ── Stage 1: development ───────────────────────────────────
FROM base AS development

COPY --from=datasets /data/ /data/

CMD ["bash"]

# ── Stage 2: production ────────────────────────────────────
FROM base AS production

COPY --from=datasets /data/ /data/

COPY pyproject.toml README.md LICENSE /tmp/lambkin/
COPY src/ /tmp/lambkin/src/
RUN uv pip install /tmp/lambkin --system --break-system-packages \
    && rm -rf /tmp/lambkin

COPY examples/ekf /ws/examples/ekf

RUN apt-get update \
    && rosdep install --from-paths examples/ekf --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths examples/ekf

CMD ["bash"]
