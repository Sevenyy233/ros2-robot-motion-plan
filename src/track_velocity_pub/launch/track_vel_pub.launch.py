import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("track_velocity_pub")
    config_path = os.path.join(pkg_share, "config", "track_vel_pub.yaml")

    track_vel_pub_node = Node(
        package = "track_velocity_pub",
        executable = "track_velocity_pub",
        parameters = [config_path],
    )

    return LaunchDescription([
        track_vel_pub_node,
    ])