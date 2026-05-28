#include <memory>
#include <cmath>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "track_can_control/CanController.hpp"

class RemoteToCmdVel : public rclcpp::Node
{
public:
    RemoteToCmdVel()
        : Node("remote_to_cmd_vel")
    {
        // 参数声明
        this->declare_parameter<std::string>("can_interface", "can3");
        this->declare_parameter<double>("max_speed", 1.2);      // 最大线速度 (m/s)
        this->declare_parameter<double>("wheel_base", 2.586);    // 履带中心距 (m)
        this->declare_parameter<double>("publish_rate", 10.0);  // 发布频率 (Hz)

        std::string can_iface = this->get_parameter("can_interface").as_string();
        max_speed_ = this->get_parameter("max_speed").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        double rate = this->get_parameter("publish_rate").as_double();

        // 初始化 CAN 控制器
        can_ = std::make_unique<CanController>(can_iface);
        if (!can_->init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init CAN on %s", can_iface.c_str());
            rclcpp::shutdown();
            return;
        }
	 else {
    		RCLCPP_INFO(this->get_logger(), "CAN initialized successfully on %s", can_iface.c_str());
	}

        // 注册 0x102 帧回调（标准帧）
        can_->registerRxHandler(0x102,
            std::bind(&RemoteToCmdVel::can102Callback, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // 创建 cmd_vel 发布器
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        // 定时器，周期性发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&RemoteToCmdVel::publishCmdVel, this));

        RCLCPP_INFO(this->get_logger(), "Remote to cmd_vel node started (listening CAN 0x102)");
    }

private:
    void can102Callback(uint32_t id, const uint8_t* data, uint8_t len)
    {
//	    RCLCPP_INFO(this->get_logger(), "Received CAN 0x102 frame, len=%d", len);
//    	    for (int i=0; i<len && i<4; i++) {
//	        printf("%02x ", data[i]);
//	    }
// 	   printf("\n");

        if (len < 4) {
            RCLCPP_WARN(this->get_logger(), "Received 0x102 frame with less than 4 bytes");
            return;
        }
        uint8_t left_raw = data[0];   // 左履带摇杆值
        uint8_t right_raw = data[2];  // 右履带摇杆值

        // 转换为带符号速度
        left_speed_ = rawToSpeed(left_raw);
        right_speed_ = rawToSpeed(right_raw);
        data_valid_ = true;

        // 可选调试打印
        // RCLCPP_DEBUG(this->get_logger(), "Left raw=%d speed=%.2f, Right raw=%d speed=%.2f",
        //              left_raw, left_speed_, right_raw, right_speed_);
    }

    double rawToSpeed(uint8_t raw)
    {
        const uint8_t CENTER = 128;
        if (raw == CENTER) return 0.0;
        double ratio = 0.0;
        if (raw < CENTER) {
            ratio = (CENTER - raw) / 128.0;   // 前进
            return ratio * max_speed_;
        } else {
            ratio = (raw - CENTER) / 128.0;   // 后退
            return -ratio * max_speed_;
        }
    }

    void publishCmdVel()
    {
        if (!data_valid_) {
            // 从未收到有效数据，不发布（或发布零速）
            return;
        }

        // 根据左右履带速度计算车体线速度和角速度
        double v = (left_speed_ + right_speed_) / 2.0;
        double omega = (right_speed_ - left_speed_) / wheel_base_;

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link"; // 或者根据你的需求填写
        twist_msg.twist.linear.x = v;
        twist_msg.twist.angular.z = omega;
        cmd_pub_->publish(twist_msg);

        // 可选打印
         RCLCPP_INFO(this->get_logger(), "Published cmd_vel: v=%.2f, w=%.2f", v, omega);
    }

    std::unique_ptr<CanController> can_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double max_speed_;
    double wheel_base_;
    double left_speed_ = 0.0, right_speed_ = 0.0;
    bool data_valid_ = false;
};

int main(int argc, char ** argv)
{
    signal(SIGPIPE, SIG_IGN);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteToCmdVel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
