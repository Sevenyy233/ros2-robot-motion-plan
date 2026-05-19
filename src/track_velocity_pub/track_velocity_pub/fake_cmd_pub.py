import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math

# 模拟发布目标速度消息,循环发布不同速度消息，匀速曲线运动
class FakeCmdPub(Node):
    def __init__(self):
        super().__init__("fake_cmd_pub")

        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            10
        )

        # 创建定时器，10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 获取初始时间，直接保存为Time对象方便后续做差计算时间差
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("FakeCmdPub node started. Publishing continuous curve motion commands.")

    def timer_callback(self):
        # 计算当前时间和运行经过的时间（秒）
        current_time = self.get_clock().now()
        elapsed_time = (current_time.nanoseconds - self.start_time.nanoseconds) / 1e9

        # 创建TwistStamped消息
        msg = TwistStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "base_link"  # 通常 cmd_vel 的参考系可以设为 odom 或 base_link

        # 匀速曲线运动：
        # 1. 线速度保持恒定 (匀速)
        # 2. 角速度使用正弦函数随时间连续变化 (曲线)
        msg.twist.linear.x = 0.5  # 恒定线速度 0.5 m/s
        msg.twist.angular.z = 0.5 * math.sin(elapsed_time)  # 角速度在 -0.5 到 0.5 rad/s 之间平滑循环变化

        # 发布消息
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCmdPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
