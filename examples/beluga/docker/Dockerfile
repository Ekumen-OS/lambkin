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

COPY --from=docker.io/ekumenlabs/lambkin-beluga-datasets:jazzy /data/ /data/

CMD ["bash"]

# ── Stage 2: production ────────────────────────────────────
FROM base AS production

COPY --from=docker.io/ekumenlabs/lambkin-beluga-datasets:jazzy /data/ /data/

COPY pyproject.toml README.md LICENSE /tmp/lambkin/
COPY src/ /tmp/lambkin/src/
RUN uv pip install /tmp/lambkin --system --break-system-packages \
    && rm -rf /tmp/lambkin

# Needed by the example, not by LAMBKIN itself. evo goes in as an isolated tool
# so its pins can't collide with the ROS system packages.
RUN uv pip install matplotlib --system --break-system-packages
RUN UV_TOOL_BIN_DIR=/usr/local/bin uv tool install "evo>=1.34.3" \
    && evo_ape --help > /dev/null

COPY examples/beluga /ws/examples/beluga

RUN apt-get update \
    && rosdep install --from-paths examples/beluga --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths examples/beluga

CMD ["bash"]
