# ── Stage 0: base ─────────────────────────────────────────
FROM ros:jazzy-ros-base AS base

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
# Code is mounted via volume in docker-compose, nothing is copied here
CMD ["bash"]
