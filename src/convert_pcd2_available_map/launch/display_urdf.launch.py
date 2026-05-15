import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 显示urdf文件
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description_pkg = get_package_share_directory("robot_description")
    robot_xacro_file = os.path.join(robot_description_pkg, "urdf", "rover.urdf.xacro")

    robot_description = ParameterValue(Command(["xacro ", robot_xacro_file]), value_type=str)
    
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )
    joint_state_publisher = Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher,
        joint_state_publisher,
    ])