from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 转换pcd文件为gridmap
def generate_launch_description():
    pcd_launch_arg = DeclareLaunchArgument(
        'pcd',
        default_value='merged_ground',
        description='PCD文件名(不需要.pcd后缀,存放在convert_pcd2_available_map包的pcd目录下)'
    )

    pcd_file_name = LaunchConfiguration('pcd')

    pcd_file = PathJoinSubstitution([
        FindPackageShare('convert_pcd2_available_map'),
        'pcd',
        [pcd_file_name, '.pcd'],
    ])

    # .pcd文件 -> PointCloud2消息
    pcd_to_PointCloud_node = Node(
        package="pcl_ros",
        executable='pcd_to_pointcloud',
        output='screen',
        parameters=[
            {"use_sim_time": False},
            {"tf_frame": "map"},
            {"file_name": pcd_file},
        ],
        remappings=[
            ("cloud_pcd", "map_points")
        ]
    )

    # PointCloud2消息 -> GridMap消息
    PointCloud_to_GridMap_node = Node(
        package="convert_pcd2_available_map",
        executable='pointcloud_to_gridmap',
        output='screen',
        parameters=[
            {"use_sim_time": False},
            {"resolution": 0.1},
            {"hole_filling_radius": 2},
        ],
    )

    return LaunchDescription([
        pcd_launch_arg,
        pcd_to_PointCloud_node,
        PointCloud_to_GridMap_node,
    ])