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
                       tmux \
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
                       ros-humble-xacro \
                       ros-humble-udp-msgs \
                       x11-apps

RUN pip3 install transforms3d gym

# -----------------------------
# Setup entrypoint
# -----------------------------
ENTRYPOINT ["/sim_ws/src/scripts/entrypoint.sh"]
CMD ["bash"]
