FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR="/usr/bin" sh

RUN rosdep init || true \
    && rosdep update

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

COPY . .

RUN pip install

CMD ["bash"]
