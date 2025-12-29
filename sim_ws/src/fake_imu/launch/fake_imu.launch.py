import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    fake_imu_node = Node(package="fake_imu",
                    namespace="simulation",
                    executable="fake_imu_publisher",
                    name="fake_imu_publisher")

    ld.add_action(fake_imu_node)

    return LaunchDescription([fake_imu_node])