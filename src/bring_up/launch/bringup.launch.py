import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1、启动 convert_pcd2_available_map 的robot_gridmap.launch.py启动文件
    convert_pcd2_gridmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('convert_pcd2_available_map'),
                'launch',
                'robot_gridmap.launch.py'
            )
        )
    )

    # 2、启动全局路径规划
    global_path_plan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('global_path_planner'),
                'launch',
                'global_planner.launch.py'
            )
        )
    )

    # 3、启动局部规划
    local_path_plan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('local_path_planner'),
                'launch',
                'local_planner.launch.py'
            )
        )
    )

    # 4、启动左右履带速度发布
    track_vel_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('track_velocity_pub'),
                'launch',
                'track_vel_pub.launch.py'
            )
        )
    )

    # 5、启动轨迹规划节点
    trajectory_planner_node = Node(
        package='trajectory_planner',
        executable='trajectory_planner'
    )

    return LaunchDescription([
        convert_pcd2_gridmap_launch,
        global_path_plan,
        local_path_plan,
        track_vel_pub,
        trajectory_planner_node
    ])