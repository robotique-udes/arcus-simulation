FROM ros:humble
SHELL ["/bin/bash", "-c"]

# -----------------------------
# Setup ROS environment in bashrc
# -----------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /sim_ws/install/setup.bash ]; then source /sim_ws/install/setup.bash; fi" >> ~/.bashrc

# -----------------------------
# Install dependencies
# -----------------------------
RUN apt-get -o Acquire::ForceIPv4=true update --fix-missing

RUN apt-get update && apt-get install -y \
    libasio-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update --fix-missing && \
    apt-get install -y git \
    nano \
    vim \
    python3-pip \
    libeigen3-dev \
    qt6-base-dev \
    qt6-base-dev-tools \
    qt6-tools-dev \
    qt6-tools-dev-tools \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-humble-rviz2 \
    ros-humble-ackermann-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-amcl \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-udp-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy-teleop \
    x11-apps \
    x11-apps xterm \
    clang-format

RUN echo "export RCUTILS_COLORIZED_OUTPUT=1 " >> ~/.bashrc

RUN echo -e "b() { \n    source /opt/ros/humble/setup.bash\n    cd /sim_ws || exit \n   colcon build "$@" --symlink-install\n    source install/local_setup.bash\n   \n}" >> ~/.bashrc

# Fix for 'no module name 'numba' with docker compose 2.40
# Seems to be a conflict between f110_gym and openAi gym
ENV PYTHONPATH="/usr/local/lib/python3.10/dist-packages:${PYTHONPATH}"
RUN pip3 install transforms3d gym

# -----------------------------
# Setup entrypoint
# -----------------------------
ENTRYPOINT ["/sim_ws/src/scripts/entrypoint.sh"]
CMD ["bash"]
