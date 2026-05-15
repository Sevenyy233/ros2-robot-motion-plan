import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("local_path_planner")
    config_path = os.path.join(pkg_share, "config", "local_planner.yaml")

    local_planner_node = Node(
        package="local_path_planner",
        executable="local_planner_node",
        name="local_path_planner",
        output="screen",
        parameters=[config_path],
    )

    # 示例：新增规划器节点，加载同一个 YAML，ROS2 自动按节点名匹配参数块
    # 只需在 YAML 中新增一个与节点名同名的 key（如 mpc_planner）
    #
    # mpc_planner_node = Node(
    #     package="local_path_planner",
    #     executable="mpc_planner_node",
    #     name="mpc_planner",
    #     output="screen",
    #     parameters=[config_path],
    # )

    return LaunchDescription([local_planner_node])
