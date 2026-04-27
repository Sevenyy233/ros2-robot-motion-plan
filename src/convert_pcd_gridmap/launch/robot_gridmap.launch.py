import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    convert_pcd_gridmap_pkg = get_package_share_directory("convert_pcd_gridmap")

    # rviz配置
    rviz_config_path = PathJoinSubstitution([
        convert_pcd_gridmap_pkg,
        "rviz",
        "show.rviz"
    ])

    # 1、启动PCD转GridMap
    convert_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(convert_pcd_gridmap_pkg, 
            "launch", "convert_pcd_gridmap.launch.py"))
    )

    # 2、启动机器人模型加载
    display_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                convert_pcd_gridmap_pkg,
                "launch",
                "display_urdf.launch.py"
            )
        )
    )

    # 3. 发布静态 TF 变换：将地图原点与机器人里程计/基座标系连接
    # 参数顺序: x y z yaw pitch roll parent_frame child_frame
    # 这里将机器人初始位置放在地图的 (0, 0, 0) 处
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    static_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_link",
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    dummy_robot_node = Node(
        package="convert_pcd_gridmap",
        executable="dummy_robot",
        name="dummy_robot",
        output="screen",
    )

    traversability_analysis_node = Node(
        package="traversability_analysis",
        executable="traversability_node",
        name="traversability_node",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", rviz_config_path,
        ]
    )

    return LaunchDescription([
        convert_launch,
        display_robot_launch,
        static_tf_map_to_odom,
        dummy_robot_node,
        traversability_analysis_node,
        rviz_node
    ])