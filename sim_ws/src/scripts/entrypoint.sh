#!/bin/bash
# entrypoint.sh

# Install f1tenth_gym if needed
if ! python3 -c "import f110_gym" &> /dev/null; then
    echo "Installing f1tenth_gym in editable mode..."
    pip3 install -e /sim_ws/src/f1tenth_gym
fi

# Source ROS setup
source /opt/ros/humble/setup.bash
source /sim_ws/install/setup.bash

# Keep container alive with bash
exec "$@"

