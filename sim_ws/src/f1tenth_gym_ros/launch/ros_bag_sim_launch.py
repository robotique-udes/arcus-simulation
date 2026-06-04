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


def get_latest_map_yaml(map_dir: str):
    """Return the path to the most recently modified .yaml file in map_dir, or None if none exist."""
    yaml_files = glob.glob(os.path.join(map_dir, "*.yaml"))
    if not yaml_files:
        return None
    latest = max(yaml_files, key=os.path.getmtime)
    return latest

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )
    
    config_dict = yaml.safe_load(open(config, 'r'))
    map_path = config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'

    # === Nodes ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz'), '--ros-args', '--log-level', 'warn']
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    # === Finalize ===
    ld.add_action(rviz_node)
    ld.add_action(ego_robot_publisher)
   
    return ld
# Note: If both simulated_localization and run_slam are true, only SLAM will run.