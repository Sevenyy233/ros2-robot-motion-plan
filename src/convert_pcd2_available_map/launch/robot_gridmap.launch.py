import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    convert_pcd2_available_map_pkg = get_package_share_directory("convert_pcd2_available_map")

    # rviz配置
    rviz_config_path = PathJoinSubstitution([
        convert_pcd2_available_map_pkg,
        "rviz",
        "show.rviz"
    ])

    # 1、启动PCD转GridMap
    convert_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                convert_pcd2_available_map_pkg, 
                "launch", 
                "convert_pcd2_gridmap.launch.py"))
    )

    # 2、启动机器人模型加载
    display_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                convert_pcd2_available_map_pkg,
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
        # 前三个坐标值为地图的原点坐标，改变可以平移，后三个为绕xyz轴的偏转角度
        arguments=['9.0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 4. 启动虚拟机器人节点：用于在 rviz 中显示机器人位置
    # 以及发布机器人 odom->base_footprint 变换和odom话题
    dummy_robot_node = Node(
        package="convert_pcd2_available_map",
        executable="dummy_robot",
        name="dummy_robot",
        output="screen",
        parameters=[
            {"initial_x": -3.0},
            {"initial_y": 0.0},
            {"initial_theta": 0.0}
        ]
    )

    # 3和4两个节点在部署到实机上的时候需要删除

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
        rviz_node
    ])