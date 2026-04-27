import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    convert_pcd_gridmap_pkg = get_package_share_directory('convert_pcd_gridmap')

    # 找到convert_pcd_gridmap下的pcd文件并作为参数传入Node
    pcd_file = os.path.join(convert_pcd_gridmap_pkg, 'pcd', 'merged_ground.pcd')

    pcd_to_PointCloud_node = Node(
        package="pcl_ros",
        executable='pcd_to_pointcloud',
        output='screen',
        parameters=[
            {"use_sim_time":False},
            {"tf_frame":"map"},
            {"file_name":pcd_file},
        ],
        remappings=[
            ("cloud_pcd", "my_points")
        ]
    )

    PointCloud_to_GridMap_node = Node(
        package="convert_pcd_gridmap",
        executable='pointcloud_to_gridmap',
        output='screen',
        parameters=[
            {"use_sim_time":False},
            {"resolution":0.1},
            {"hole_filling_radius": 2},
        ],
    )

    return LaunchDescription([
        pcd_to_PointCloud_node,
        PointCloud_to_GridMap_node,
    ])