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
    tmux \
    python3-pip \
    python3-tk \
    libeigen3-dev \
    qt6-base-dev \
    qt6-base-dev-tools \
    qt6-tools-dev \
    qt6-tools-dev-tools \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    libglu1-mesa \
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
    ros-humble-joy \
    ros-humble-rmw-cyclonedds-cpp \
    x11-apps \
    x11-apps xterm \
    clang-format \
    openssh-client \
    iputils-ping \
    libnss-mdns

RUN echo "export RCUTILS_COLORIZED_OUTPUT=1 " >> ~/.bashrc

RUN echo -e "b() { \n    source /opt/ros/humble/setup.bash\n    cd /sim_ws || exit \n   colcon build "$@" --packages-ignore range_libc particle_filter --symlink-install\n    source install/local_setup.bash\n   \n}" >> ~/.bashrc

# Fix for 'no module name 'numba' with docker compose 2.40
RUN pip3 install transforms3d gymnasium numba scipy pyglet pillow "matplotlib<=3.8"


# -----------------------------
# Setup entrypoint
# -----------------------------
ENTRYPOINT ["/sim_ws/src/scripts/entrypoint.sh"]
CMD ["bash"]
