import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 局部高程图截取与代价计算节点
    local_gridmap_node = Node(
        package='local_path_planner',
        executable='local_gridmap_node',
        name='local_gridmap_node',
        output='screen',
        parameters=[{
            'global_map_topic': '/grid_map',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'local_map_size_x': 10.0,
            'local_map_size_y': 10.0,
            'publish_rate': 10.0,
            'max_slope_angle': 25.0,
            'obstacle_height_thresh': 0.2,
            'lethal_cost': 255.0
        }]
    )

    # 纯空间解耦版局部路径规划器
    new_local_planner_node = Node(
        package='local_path_planner',
        executable='new_local_planner',
        name='new_local_planner',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'map_frame': 'map',
            'lookahead_dist': 4.0,
            'max_curvature': 1.5,
            'num_paths': 21,
            'points_per_path': 20,
            'max_obstacle_height': 0.2,
            'max_slope_angle': 25.0,
            'goal_weight': 1.0,
            'curvature_weight': 0.5
        }]
    )

    return LaunchDescription([
        local_gridmap_node,
        new_local_planner_node
    ])
