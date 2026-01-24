import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    visualization_node = Node(package="visualization",
                    namespace="simulation",
                    executable="waypoints_publisher",
                    name="waypoints_publisher")

    ld.add_action(visualization_node)

    return LaunchDescription([visualization_node])