import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped

# 订阅local_planner的速度消息并分别转化为履带机器人的两个履带的速度
class TrackVelocityPublisher(Node):
    def __init__(self):
        super().__init__("track_velocity_publisher")
        
        # 参数
        self.declare_parameter("track_length", 3.0)
        self.declare_parameter("track_width", 0.35)
        self.declare_parameter("track_wheel_radius", 0.1)
        self.declare_parameter("track_wheel_separation_distance", 1.8)

        # 初始化变量
        self.track_length = self.get_parameter("track_length").value
        self.track_width = self.get_parameter("track_width").value
        self.track_wheel_radius = self.get_parameter("track_whee_radius").value
        self.track_wheel_separation_distance = self.get_parameter("track_wheel_separation_distance").value
        
        self.left_track_vel = None
        self.right_track_vel = None

        # 订阅local_planner的速度消息
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # 左履带发布速度者
        self.left_track_vel_pub = self.create_publisher(
            TwistStamped,
            "/left_track_vel",
            10
        )
        # 右履带发布速度者
        self.right_track_vel_pub = self.create_publisher(
            TwistStamped,
            "/right_track_vel",
            10
        )

    def cmd_vel_callback(self, msg:TwistStamped):
        self.left_track_vel = TwistStamped()
        self.right_track_vel = TwistStamped()

def main():
    rclpy.init()
    node = TrackVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

        
        