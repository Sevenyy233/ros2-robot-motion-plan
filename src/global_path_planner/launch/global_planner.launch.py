import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("global_path_planner")
    config_path = os.path.join(pkg_share, "config", "global_path_planner.yaml")

    planner_node = Node(
        package="global_path_planner",
        executable="global_planner_node",
        name="global_path_planner",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([planner_node])
