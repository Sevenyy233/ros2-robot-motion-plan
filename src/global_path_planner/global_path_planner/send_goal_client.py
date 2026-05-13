import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_motion_plan_msgs.action import SendGoal
from custom_motion_plan_msgs.msg import RobotInfo
from geometry_msgs.msg import PoseStamped

class SendGoalClient(Node):
    def __init__(self):
        super().__init__("send_goal_client")
        self.send_goal_client_ = ActionClient(
            self,
            SendGoal,
            "/goal_check"
        )

        # 等待服务上线，并循环输出日志
        while not self.send_goal_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("等待 /goal_check Action 服务上线...")

        self.get_logger().info("发送目标客户端初始化完成！")

    def send_goal(self, x, y, z=0.0):
        """
        发送目标点进行全局规划
        """
        goal_msg = SendGoal.Goal()
        goal_msg.request_time = self.get_clock().now().to_msg()
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        # 默认无旋转
        pose.pose.orientation.w = 1.0
        
        goal_msg.goal_pose = pose
        
        self.get_logger().info(f"发送目标点: x={x}, y={y}, z={z}")
        
        # 异步发送请求，并绑定反馈回调函数
        self.send_goal_future = self.send_goal_client_.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        处理服务端对目标请求的响应
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("服务端拒绝了该目标请求！")
            return

        self.get_logger().info("服务端已接收目标请求，正在等待执行结果...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        处理规划/运动过程中的实时反馈
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f"收到反馈 - 阶段ID: {feedback.current_stage}, "
                               f"进度: {feedback.completion_ratio:.2f}, "
                               f"剩余距离: {feedback.distance_remaining:.2f}m")

    def get_result_callback(self, future):
        """
        处理最终结果
        """
        result = future.result().result
        if result.success:
            self.get_logger().info(f"全局路径规划成功！耗时及结果信息: {result.message}")
        else:
            self.get_logger().warn(f"全局路径规划失败！错误码: {result.error_code}, 错误信息: {result.message}")
            
        # 收到结果后，通知退出
        rclpy.shutdown()

def main(args=None):
    # 输入x,y坐标进行全局规划
    x = float(input("请输入目标点x坐标: "))
    y = float(input("请输入目标点y坐标: "))
    rclpy.init(args=args)
    send_goal_client = SendGoalClient()
    
    # 作为测试，我们在节点启动后发送一个示例目标点 (如 x=5.0, y=5.0)
    # 你可以根据实际逻辑通过其他方式（比如监听按键、定时器、或其他话题）来触发发送
    send_goal_client.send_goal(x, y)
    
    # spin会阻塞，直到收到shutdown信号
    try:
        rclpy.spin(send_goal_client)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保在退出前正确销毁节点
        if rclpy.ok():
            send_goal_client.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
