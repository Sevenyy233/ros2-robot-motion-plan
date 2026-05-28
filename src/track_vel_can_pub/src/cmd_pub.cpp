#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// 发送一个TwistStamped消息序列
class CmdPub : public rclcpp::Node
{
public:
    CmdPub() : Node("cmd_pub")
    {
        // 创建发布者，话题为 /cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        
        // 记录节点启动时间
        start_time_ = this->now();
        is_stopped_ = false;

        // 声明并获取参数
        this->declare_parameter<double>("target_speed", 0.1);
        this->declare_parameter<double>("cruise_duration", 3.0);
        this->declare_parameter<double>("decel_duration", 2.0);
        this->declare_parameter<double>("wait_duration", 2.0);

        target_speed_ = this->get_parameter("target_speed").as_double();
        cruise_duration_ = this->get_parameter("cruise_duration").as_double();
        decel_duration_ = this->get_parameter("decel_duration").as_double();
        wait_duration_ = this->get_parameter("wait_duration").as_double();

        // 以 10Hz (100ms) 的频率发布。
        timer_ = this->create_wall_timer(
            100ms, std::bind(&CmdPub::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "开始测试：加速至 %.2fm/s 并保持 %.1fs...", target_speed_, cruise_duration_);
    }

private:
    void timer_callback()
    {
        if (is_stopped_) return;

        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        double elapsed_time = (this->now() - start_time_).seconds();

        // 阶段 1：匀速巡航阶段
        if (elapsed_time <= cruise_duration_) {
            msg.twist.linear.x = target_speed_;
            msg.twist.angular.z = 0.0;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[巡航] 发布速度: v=%.3f | 已持续: %.1fs", target_speed_, elapsed_time);
        } 
        // 阶段 2：匀减速阶段
        else if (elapsed_time <= cruise_duration_ + decel_duration_) {
            // 计算减速比例 [1.0 -> 0.0]
            double decel_ratio = 1.0 - (elapsed_time - cruise_duration_) / decel_duration_;
            double current_speed = target_speed_ * decel_ratio;
            
            // 限制极小值直接归零 (使用绝对值兼容后退)
            if (std::abs(current_speed) < 0.01) current_speed = 0.0;
            
            msg.twist.linear.x = current_speed;
            msg.twist.angular.z = 0.0;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[减速] 发布速度: v=%.3f | 已持续: %.1fs", current_speed, elapsed_time);
        }
        // 阶段 3：停车等待阶段（保持发送0速度，让比例阀保持在0开度状态一段时间）
        else if (elapsed_time <= cruise_duration_ + decel_duration_ + wait_duration_) {
            msg.twist.linear.x = 0.0;
            msg.twist.angular.z = 0.0;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[停车等待] 发布速度: v=0.0 | 等待关闭比例阀: %.1fs", 
                cruise_duration_ + decel_duration_ + wait_duration_ - elapsed_time);
        }
        // 阶段 4：测试完成，退出节点（此时会触发 can_send 节点的超时或退出时的比例阀关闭）
        else {
            RCLCPP_INFO(this->get_logger(), "测试完成，停止发布并退出程序。");
            is_stopped_ = true;
            timer_->cancel();   // 停止定时器
            rclcpp::shutdown(); // 退出节点，结束测试
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    bool is_stopped_;
    
    double target_speed_;
    double cruise_duration_;
    double decel_duration_;
    double wait_duration_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}