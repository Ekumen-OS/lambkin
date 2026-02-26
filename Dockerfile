
FROM ros:jazzy-ros-base


RUN apt-get update && apt-get install -y \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws


COPY lambkin_ros2/ /ws/lambkin_ros_ws/src/lambkin_ros2/
COPY /src/lambkin/ /ws/src/lambkin/
COPY pyproject.toml uv.lock ./

RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR="/usr/bin" sh

RUN rosdep init || true \
    && rosdep update

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN apt-get update

CMD ["bash"]
