import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_motion_plan_msgs.action import SendGoal
from geometry_msgs.msg import PoseStamped
import threading
import math

import time

class SendGoalClient(Node):
    def __init__(self):
        super().__init__("send_goal_client")
        self.send_goal_client_ = ActionClient(
            self,
            SendGoal,
            "/goal_check"
        )

        # 等待服务上线
        self.get_logger().info("等待 /goal_check Action 服务上线...")
        self.send_goal_client_.wait_for_server()
        self.get_logger().info("Action 服务已上线，可以开始发送目标点！")

        # 状态阶段映射表
        self.stage_map = {
            0: "空闲 (IDLE)",
            1: "全局规划中 (GLOBAL_PLANNING)",
            2: "局部规划中 (LOCAL_PLANNING)",
            3: "轨迹规划中 (TRAJECTORY_PLANNING)",
            4: "移动中 (MOVING)"
        }

    def send_goal(self, x, y, yaw=0.0):
        """
        发送目标点进行规划
        """
        goal_msg = SendGoal.Goal()
        
        # 组装 PoseStamped 消息
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # 将偏航角(yaw)转换为四元数(Quaternion)
        # Roll = 0, Pitch = 0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        goal_msg.goal_pose = pose
        
        self.get_logger().info(f"正在发送目标点: x={x}, y={y}, yaw={yaw:.2f} rad")
        
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

        self.get_logger().info("服务端已接收目标请求，正在执行...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        处理规划/运动过程中的实时反馈
        """
        feedback = feedback_msg.feedback
        stage_str = self.stage_map.get(feedback.current_stage, f"未知阶段({feedback.current_stage})")
        
        self.get_logger().info(
            f"[实时反馈] 阶段: {stage_str}, "
            f"总进度: {feedback.completion_ratio * 100:.1f}%, "
            f"剩余距离: {feedback.distance_remaining:.2f}m"
        )

    def get_result_callback(self, future):
        """
        处理最终结果
        """
        result = future.result().result
        if result.success:
            self.get_logger().info(f"✅ 成功到达目标点！耗时及结果信息: {result.message}")
        else:
            self.get_logger().warn(f"❌ 规划失败！错误码: {result.error_code}, 错误信息: {result.message}")


def main(args=None):
    rclpy.init(args=args)
    client_node = SendGoalClient()
    
    # 在后台线程中运行 ROS 2 事件循环 (spin)，以便主线程可以接收持续的用户输入
    spin_thread = threading.Thread(target=rclpy.spin, args=(client_node,), daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok():
            # 稍微等待一下，让后台线程的打印日志能够先输出，防止和input提示文字混叠
            time.sleep(0.1)
            print("\n" + "="*50)
            user_input = input("请输入目标点坐标和朝向(格式: x y [yaw]) 或输入 'q' 退出: ")
            
            if user_input.strip().lower() == 'q':
                break
            
            try:
                parts = user_input.split()
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                    client_node.send_goal(x, y, yaw)
                else:
                    print("⚠️ 输入格式错误，请按照格式输入，例如: 5.0 2.5 或 5.0 2.5 1.57")
            except ValueError:
                print("⚠️ 无效的数字，请重新输入！")
                
    except KeyboardInterrupt:
        pass
    finally:
        print("正在关闭客户端...")
        if rclpy.ok():
            client_node.destroy_node()
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
