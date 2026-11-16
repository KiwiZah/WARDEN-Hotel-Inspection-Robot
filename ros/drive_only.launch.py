#!/usr/bin/env python3
"""TurtleBot3 Burger motors only; no unused LiDAR driver or serial collision."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(get_package_share_directory("turtlebot3_bringup"),
                          "param", "humble", "burger.yaml")
    return LaunchDescription([
        Node(package="turtlebot3_node", executable="turtlebot3_ros",
             arguments=["-i", os.environ.get("WARDEN_OPENCR_PORT", "/dev/ttyACM0")],
             parameters=[params], output="screen")
    ])
