import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    nav_25d_dir = get_package_share_directory('nav_25d')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', 
        default=os.path.join(nav_25d_dir, 'params', 'nav2_params.yaml'))
    
    # RTAB-Map Parameters
    rtabmap_args = DeclareLaunchArgument(
        'rtabmap_args', default_value='--delete_db_on_start',
        description='RTAB-Map arguments')
    
    # Nodes
    
    # 1. RTAB-Map SLAM
    # Use arguments matching the robot's sensors (based on src/nav_25d/launch/rtabmap.launch.py)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'args': LaunchConfiguration('rtabmap_args'),
            'rgb_topic': '/camera/image_raw',             # Corrected
            'depth_topic': '/camera/depth/image_raw',     # Corrected
            'camera_info_topic': '/camera/camera_info',   # Corrected
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'wait_imu_to_init': 'false',                  # Disabled as no IMU topic in local launch
            # 'imu_topic': '/imu',                        # Removed
            'subscribe_scan_cloud': 'true',               # Enabled for Velodyne
            'scan_cloud_topic': '/velodyne_points',       # Added
            'visual_odometry': 'false',                   # Use robot odom
            'odom_topic': '/odom',             # Corrected
            'qos': '2', 
            'rviz': 'false', 
        }.items()
    )

    # 2. Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(nav_25d_dir, 'params', 'nav2_params.yaml')),
        rtabmap_args,
        rtabmap_launch,
        nav2_launch,
    ])
