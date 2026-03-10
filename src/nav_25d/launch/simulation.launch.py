import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = LaunchConfiguration('world_file_name', default='uneven_terrain')

    # Include the robot_description gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world_file_name': world_file_name,
            'use_sim_time': use_sim_time
        }.items()
    )

    odom_tf_broadcaster = Node(
        package='nav_25d',
        executable='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file_name',
            default_value='uneven_terrain',
            description='World file name without .world extension (outdoor, cabin, uneven_terrain, etc.)'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo_launch,
        odom_tf_broadcaster
    ])
