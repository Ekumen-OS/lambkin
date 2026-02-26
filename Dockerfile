
FROM ros:jazzy-ros-base


RUN apt-get update && apt-get install -y \
    python3-tk \
    python3-numpy \
    python3-pandas \
    ros-jazzy-beluga \
    ros-jazzy-beluga-amcl \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /ws


COPY lambkin_ros2/ /ws/src/lambkin_ros2/
COPY /src/lambkin/ /ws/src/lambkin/
COPY pyproject.toml uv.lock ./

RUN uv pip install --system -e .

RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*


RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
