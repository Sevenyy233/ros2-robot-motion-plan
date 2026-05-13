from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    layer_name_arg = DeclareLaunchArgument(
        "layer_name", default_value="elevation",
        description="GridMap layer name for terrain elevation"
    )
    max_slope_angle_arg = DeclareLaunchArgument(
        "max_slope_angle", default_value="45.0",
        description="Maximum traversable slope angle (degrees)"
    )
    max_elevation_arg = DeclareLaunchArgument(
        "max_elevation", default_value="1.0",
        description="Maximum absolute elevation treated as obstacle"
    )
    lookahead_min_arg = DeclareLaunchArgument(
        "lookahead_min", default_value="0.3",
        description="Minimum lookahead distance for Pure Pursuit"
    )
    lookahead_max_arg = DeclareLaunchArgument(
        "lookahead_max", default_value="2.0",
        description="Maximum lookahead distance for Pure Pursuit"
    )
    max_linear_speed_arg = DeclareLaunchArgument(
        "max_linear_speed", default_value="0.5",
        description="Maximum linear velocity (m/s)"
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        "max_angular_speed", default_value="1.0",
        description="Maximum angular velocity (rad/s)"
    )
    goal_tolerance_arg = DeclareLaunchArgument(
        "goal_tolerance", default_value="0.15",
        description="Distance tolerance to consider goal reached (m)"
    )
    control_rate_arg = DeclareLaunchArgument(
        "control_rate", default_value="20.0",
        description="Control loop frequency (Hz)"
    )

    local_planner_node = Node(
        package="local_path_planner",
        executable="local_planner_node",
        name="local_path_planner",
        output="screen",
        parameters=[{
            "layer_name": LaunchConfiguration("layer_name"),
            "max_slope_angle": LaunchConfiguration("max_slope_angle"),
            "max_elevation": LaunchConfiguration("max_elevation"),
            "lookahead_min": LaunchConfiguration("lookahead_min"),
            "lookahead_max": LaunchConfiguration("lookahead_max"),
            "max_linear_speed": LaunchConfiguration("max_linear_speed"),
            "max_angular_speed": LaunchConfiguration("max_angular_speed"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),
            "control_rate": LaunchConfiguration("control_rate"),
            "base_frame": "base_link",
            "map_frame": "map",
            "wheel_base": 0.3,
            "lookahead_scale": 0.3,
        }],
    )

    return LaunchDescription([
        layer_name_arg,
        max_slope_angle_arg,
        max_elevation_arg,
        lookahead_min_arg,
        lookahead_max_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        goal_tolerance_arg,
        control_rate_arg,
        local_planner_node,
    ])
