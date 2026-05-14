#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "custom_motion_plan_msgs/msg/robot_trajectory.hpp"

// 订阅局部路/local_path并根据速度指令计算轨迹

namespace trajectory_planner
{
class TrajectoryPlanner : public rclcpp::Node{
public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path",
            10,
            std::bind(&TrajectoryPlanner::local_path_callback, this, std::placeholders::_1)
        );

        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel",
            10,
            std::bind(&TrajectoryPlanner::twist_callback, this, std::placeholders::_1)
        );

        trajectory_pub_ = this->create_publisher<custom_motion_plan_msgs::msg::RobotTrajectory>(
            "/robot_trajectory",
            10
        );

        
    
        RCLCPP_INFO(this->get_logger(), "轨迹规划器初始化成功！");
    }

private:
    // 局部路径回调函数
    // 订阅局部路径并将事件信息赋予每个局部路径点
    void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到局部路径，包含点数量：%d", msg->poses.size());


    }

    // 订阅速度，计算线加速度和角加速度
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {

    }

    void publish_trajectory(const custom_motion_plan_msgs::msg::RobotTrajectory::SharedPtr msg)
    {

    }

private:
    // 订阅局部路径和速度指令
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    // 发布轨迹
    rclcpp::Publisher<custom_motion_plan_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
    // 
};
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trajectory_planner::TrajectoryPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}