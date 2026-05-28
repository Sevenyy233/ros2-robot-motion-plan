from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    track_vel_can_pub_node = Node(
        package="track_vel_can_pub",
        executable="track_vel_can_send",
        parameters=[
            {'can_interface': 'can3'},
            {'max_speed': 1.2},
            {'wheel_separation': 2.586},
            {'publish_rate': 10.0}
        ]
    )

    return LaunchDescription([
        track_vel_can_pub_node
    ])