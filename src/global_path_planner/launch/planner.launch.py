import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    layer_name_arg = DeclareLaunchArgument(
        'layer_name', default_value='elevation',
        description='Layer name in GridMap to use for obstacle checking'
    )
    
    max_elevation_arg = DeclareLaunchArgument(
        'max_elevation', default_value='1.0',
        description='Threshold above which absolute elevation is considered an obstacle'
    )
    
    max_slope_angle_arg = DeclareLaunchArgument(
        'max_slope_angle', default_value='45.0',
        description='Maximum traversable slope angle in degrees'
    )
    
    treat_nan_as_obstacle_arg = DeclareLaunchArgument(
        'treat_nan_as_obstacle', default_value='True',
        description='Whether to treat NaN cells as obstacles'
    )
    
    use_tf_for_start_arg = DeclareLaunchArgument(
        'use_tf_for_start', default_value='True',
        description='Use TF map->base_link instead of /initialpose for start pose'
    )

    planner_node = Node(
        package='global_path_planner',
        executable='planner_node',
        name='global_path_planner',
        output='screen',
        parameters=[{
            'layer_name': LaunchConfiguration('layer_name'),
            'max_elevation': LaunchConfiguration('max_elevation'),
            'max_slope_angle': LaunchConfiguration('max_slope_angle'),
            'treat_nan_as_obstacle': LaunchConfiguration('treat_nan_as_obstacle'),
            'use_tf_for_start': LaunchConfiguration('use_tf_for_start'),
            'base_frame': 'base_link',
            'map_frame': 'map'
        }]
    )

    return LaunchDescription([
        layer_name_arg,
        max_elevation_arg,
        max_slope_angle_arg,
        treat_nan_as_obstacle_arg,
        use_tf_for_start_arg,
        planner_node
    ])
