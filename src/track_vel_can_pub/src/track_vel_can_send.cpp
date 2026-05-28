#include <chrono>
#include <cmath>
#include <memory>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "track_can_control/CanController.hpp"

class TrackVelCanSend : public rclcpp::Node
{
public:
    TrackVelCanSend() : Node("track_vel_can_send")
    {
        // 声明参数
        this->declare_parameter<std::string>("can_interface", "can3");
        this->declare_parameter<double>("wheel_separation", 2.586);
        this->declare_parameter<double>("max_speed", 0.5);  //最大线速度 m/s
        this->declare_parameter<double>("control_rate", 10.0); //发送频率 10hz
        this->declare_parameter<double>("cmd_time_out", 1.0);   //超时时间 s

        //获取参数
        can_interface = this->get_parameter("can_interface").as_string();
        wheel_separation = this->get_parameter("wheel_separation").as_double();
        max_speed = this->get_parameter("max_speed").as_double();
        control_rate = this->get_parameter("control_rate").as_double();
        cmd_time_out = this->get_parameter("cmd_time_out").as_double();

        //初始化 CAN 控制器
        can_controller_ = std::make_unique<CanController>(can_interface);
        if (!can_controller_->init()) {
            RCLCPP_ERROR(this->get_logger(), "CAN %s 初始化失败", can_interface.c_str());
            throw std::runtime_error("CAN " + can_interface + " 初始化失败");
        }

        // 发送启动行走比例阀指令 (ID: 0x201)
        uint8_t enable_data[8] = {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
        if (can_controller_->sendCanFrame(0x201, enable_data, 8)) {
            RCLCPP_INFO(this->get_logger(), "已自动发送行走比例阀启动指令 (ID: 0x201)");
        } else {
            RCLCPP_WARN(this->get_logger(), "行走比例阀启动指令发送失败！");
        }

        // 创建订阅者
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10,
            std::bind(&TrackVelCanSend::twist_callback, this, std::placeholders::_1)
        );

        // 创建定时器，周期性发布 CAN 命令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate)),
            std::bind(&TrackVelCanSend::send_can_commands, this)
        );

        last_cmd_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "左右履带 CAN 通信节点初始化成功!");
    }

    ~TrackVelCanSend()
    {
        // 节点退出时，自动发送关闭行走比例阀指令 (ID: 0x201) 和关闭液压指令 (ID: 0x301)
        if (can_controller_) {
            uint8_t disable_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            if (can_controller_->sendCanFrame(0x201, disable_data, 8)) {
                RCLCPP_INFO(this->get_logger(), "已自动发送行走比例阀关闭指令 (ID: 0x201)");
            } else {
                RCLCPP_WARN(this->get_logger(), "行走比例阀关闭指令发送失败！");
            }

            uint8_t disable_hydraulic[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            if (can_controller_->sendCanFrame(0x301, disable_hydraulic, 8)) {
                RCLCPP_INFO(this->get_logger(), "已自动发送液压关闭指令 (ID: 0x301)");
            } else {
                RCLCPP_WARN(this->get_logger(), "液压关闭指令发送失败！");
            }
        }
    }

private:
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到的线速度 v = %.2f, 角速度 w = %.2f",
            msg->twist.linear.x, msg->twist.angular.z);
        last_cmd_time_ = this->now();
        
        double linear_v = msg->twist.linear.x;
        double angular_w = msg->twist.angular.z;

        // 限幅
        linear_v = std::max(-max_speed, std::min(max_speed, linear_v));
        double max_w = max_speed / wheel_separation * 2.0; // 简单限幅
        angular_w = std::max(-max_w, std::min(max_w, angular_w));

        // 计算左右履带速度
        double v_left = linear_v - angular_w * wheel_separation / 2.0;
        double v_right = linear_v + angular_w * wheel_separation / 2.0;

        // 转换为油门和方向
        left_throttle_ = speedToThrottle(v_left);
        left_dir_ = speedToDirection(v_left);
        right_throttle_ = speedToThrottle(v_right);
        right_dir_ = speedToDirection(v_right);
    }

    void send_can_commands()
    {
        // 超时检测
        if ((this->now() - last_cmd_time_).seconds() > cmd_time_out) {
            left_throttle_ = 0; left_dir_ = 0;
            right_throttle_ = 0; right_dir_ = 0;
        }

        // 周期性发送使能和液压控制指令，确保底层不超时
        uint8_t enable_data[8] = {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
        can_controller_->sendCanFrame(0x201, enable_data, 8);

        uint8_t cmd301[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        can_controller_->sendCanFrame(0x301, cmd301, 8);

        uint8_t cmd302[8] = {0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        can_controller_->sendCanFrame(0x302, cmd302, 8);

        // 构建左履带 CAN 帧 (ID: 0x0CFE3206)
        uint8_t left_data[8] = {0};
        left_data[0] = left_throttle_;
        left_data[2] = left_dir_;
        
        // 右履带 (ID: 0x0CFE3306)
        uint8_t right_data[8] = {0};
        right_data[0] = right_throttle_;
        right_data[2] = right_dir_;

        // 发送控制指令
        if (!can_controller_->sendCanFrame(0x0CFE3206, left_data, 8)) {
            RCLCPP_WARN(this->get_logger(), "左侧履带控制指令发送失败");
        }
        if (!can_controller_->sendCanFrame(0x0CFE3306, right_data, 8)) {
            RCLCPP_WARN(this->get_logger(), "右侧履带控制指令发送失败");
        }
    }

    uint8_t speedToThrottle(double speed)
    {
        if (std::abs(speed) < 1e-6) return 0;
        double ratio = std::abs(speed) / max_speed;
        int thr = static_cast<int>(ratio * 250.0);
        if (thr < 1) thr = 1;
        if (thr > 250) thr = 250;
        return static_cast<uint8_t>(thr);
    }

    uint8_t speedToDirection(double speed)
    {
        if (speed > 0) return 1;
        if (speed < 0) return 2;
        return 0;
    }

    std::unique_ptr<CanController> can_controller_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_cmd_time_;

    double wheel_separation;
    double max_speed;
    double cmd_time_out;
    std::string can_interface;
    double control_rate;

    uint8_t left_throttle_ = 0, left_dir_ = 0;
    uint8_t right_throttle_ = 0, right_dir_ = 0;
};

int main(int argc, char ** argv)
{
    signal(SIGPIPE, SIG_IGN);
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<TrackVelCanSend>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("track_vel_can_send"), "节点启动失败: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}