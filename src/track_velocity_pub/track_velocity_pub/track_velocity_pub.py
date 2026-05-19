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
        self.declare_parameter("track_wheel_separation_distance", 1.8)
        self.declare_parameter("left_track_frame", "left_track_link")
        self.declare_parameter("right_track_frame", "right_track_link")

        # 初始化变量
        self.track_length = self.get_parameter("track_length").value
        self.track_width = self.get_parameter("track_width").value
        self.track_wheel_separation_distance = self.get_parameter("track_wheel_separation_distance").value
        self.left_track_frame = self.get_parameter("left_track_frame").value
        self.right_track_frame = self.get_parameter("right_track_frame").value


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

        self.get_logger().info("左右履带速度发布者启动,初始化成功")

    def cmd_vel_callback(self, msg:TwistStamped):
        # 提取目标线速度和角速度
        v = msg.twist.linear.x
        omega = msg.twist.angular.z
        
        # 履带中心间距
        L = self.track_wheel_separation_distance
        
        # 运动学逆解：计算左右履带的线速度 (m/s)
        v_left = v - (omega * L) / 2.0
        v_right = v + (omega * L) / 2.0
        
        # 如果你的底层驱动需要的是驱动轮的角速度 (rad/s)，请取消以下注释并使用 w_left/w_right
        # R = self.track_wheel_radius
        # w_left = v_left / R
        # w_right = v_right / R

        # 构造并发布左履带速度消息
        left_msg = TwistStamped()
        left_msg.header.stamp = msg.header.stamp
        left_msg.header.frame_id = self.left_track_frame
        left_msg.twist.linear.x = v_left
        self.left_track_vel_pub.publish(left_msg)
    
        # 构造并发布右履带速度消息
        right_msg = TwistStamped()
        right_msg.header.stamp = msg.header.stamp
        right_msg.header.frame_id = self.right_track_frame
        right_msg.twist.linear.x = v_right
        self.right_track_vel_pub.publish(right_msg)

         # 打印左右履带速度
        self.get_logger().info(f"左履带速度: {v_left:.4f} m/s, 右履带速度: {v_right:.4f} m/s")

def main():
    rclpy.init()
    node = TrackVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        