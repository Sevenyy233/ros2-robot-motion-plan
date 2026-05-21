from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 转换pcd文件为gridmap
def generate_launch_description():
    pcd_launch_arg = DeclareLaunchArgument(
        'pcd',
        default_value='map',
        description='PCD文件名(不需要.pcd后缀,存放在convert_pcd2_available_map包的pcd目录下)'
    )

    pcd_file_name = LaunchConfiguration('pcd')

    pcd_file = PathJoinSubstitution([
        FindPackageShare('convert_pcd2_available_map'),
        'pcd',
        [pcd_file_name, '.pcd'],
    ])

    # .pcd文件 -> PointCloud2消息 (作为静态底图)
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
            ("cloud_pcd", "/map_points_static")
        ]
    )

    # 融合静态底图和动态雷达点云
    combine_pointcloud_lidar_node = Node(
        package="convert_pcd2_available_map",
        executable='combine_pointcloud_lidar',
        output='screen',
        parameters=[
            {"static_map_topic": "/map_points_static"},
            {"lidar_topic": "/lidar3_points"},
            {"output_topic": "/map_points"},
            {"map_frame": "map"},
            {"lidar_timeout": 1.0},
            {"publish_rate": 5.0},
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
            {"max_height":7.0},
            {"min_height": -2.0},
            {"hole_filling_radius": 2},
            {"max_slope_angle": 45.0},
        ],
        remappings=[
            ("map_points", "/map_points")
        ]
    )

    return LaunchDescription([
        pcd_launch_arg,
        pcd_to_PointCloud_node,
        combine_pointcloud_lidar_node,
        PointCloud_to_GridMap_node,
    ])