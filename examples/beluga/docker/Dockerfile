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

COPY --from=ekumenlabs/lambkin-beluga-datasets:jazzy /data/ /data/

CMD ["bash"]

# ── Stage 2: production ────────────────────────────────────
FROM base AS production

COPY --from=ekumenlabs/lambkin-beluga-datasets:jazzy /data/ /data/

COPY pyproject.toml README.md LICENSE /tmp/lambkin/
COPY src/ /tmp/lambkin/src/
RUN uv pip install /tmp/lambkin --system --break-system-packages \
    && rm -rf /tmp/lambkin

COPY examples/beluga /ws/examples/beluga

RUN apt-get update \
    && rosdep install --from-paths examples/beluga --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths examples/beluga

CMD ["bash"]
