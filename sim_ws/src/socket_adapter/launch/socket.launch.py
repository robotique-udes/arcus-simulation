# MIT License
# (copyright header...)

import os
import yaml
import datetime
import glob

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    socket_node = Node(
        package='socket_adapter',
        executable='socket_adapter',
        name='socket_adapter',
        parameters=[os.path.join(get_package_share_directory('socket_adapter'), 'config', 'foxglove_adapter.yaml')]
    )
    # === Finalize ===
    ld.add_action(socket_node)
    return ld
