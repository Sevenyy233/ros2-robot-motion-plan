#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "custom_motion_plan_msgs/msg/robot_trajectory.hpp"
#include "trajectory_planner/cubic_spline.hpp"

namespace trajectory_planner
{
class TrajectoryPlanner : public rclcpp::Node {
public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        // 声明参数
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("target_speed", 0.5); // 期望的恒定运行速度 m/s
        
        // 获取参数
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        target_speed_ = this->get_parameter("target_speed").as_double();

        // 初始化 TF2 缓冲和监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅局部路径
        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path",
            10,
            std::bind(&TrajectoryPlanner::local_path_callback, this, std::placeholders::_1)
        );
        
        // 订阅真实里程计信息，获取当前速度
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&TrajectoryPlanner::odom_callback, this, std::placeholders::_1)
        );

        // 发布轨迹话题
        trajectory_pub_ = this->create_publisher<custom_motion_plan_msgs::msg::RobotTrajectory>(
            "/robot_trajectory",
            10
        );

        // 创建定时器
        trajectory_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryPlanner::trajectory_timer_callback, this)
        );
    
        RCLCPP_INFO(this->get_logger(), "轨迹规划器初始化成功！");
    }

private:
    // 局部路径回调函数
    void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到局部路径，包含点数量：%ld", msg->poses.size());
        current_local_path_ = msg;
    }

    // 里程计回调函数，获取当前实际速度
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到里程计信息, 当前位置:%f,%f,%f",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        RCLCPP_INFO(this->get_logger(), "当前位置实际速度:%f,%f,%f",
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        current_odom_ = msg;
    }

    void publish_trajectory(const custom_motion_plan_msgs::msg::RobotTrajectory::SharedPtr msg)
    {
        trajectory_pub_->publish(*msg);
    }

    // 定时器回调函数
    void trajectory_timer_callback()
    {
        if (!current_local_path_ || current_local_path_->poses.size() < 3) {
            return; // 没有路径或点太少，无法生成样条曲线
        }

        // 提取 x 和 y 坐标用于插值
        std::vector<double> px, py;
        for (const auto& pose_stamped : current_local_path_->poses) {
            px.push_back(pose_stamped.pose.position.x);
            py.push_back(pose_stamped.pose.position.y);
        }

        // 初始化 2D 样条曲线
        Spline2D spline;
        if (!spline.init(px, py)) {
            RCLCPP_WARN(this->get_logger(), "样条曲线初始化失败，路径点可能存在问题");
            return;
        }

        // 创建轨迹消息
        auto trajectory_msg = std::make_shared<custom_motion_plan_msgs::msg::RobotTrajectory>();
        trajectory_msg->header.stamp = this->now();
        trajectory_msg->header.frame_id = current_local_path_->header.frame_id;

        // 根据路径总长度和期望速度，重采样并分配时间
        double total_length = spline.get_total_length();
        double ds = 0.1; // 重采样空间间隔 0.1m
        
        for (double s = 0; s <= total_length; s += ds) {
            custom_motion_plan_msgs::msg::TrajectoryPoint point;
            
            // 1. 计算平滑后的位置
            double x, y;
            spline.calc_position(s, x, y);
            point.pose.position.x = x;
            point.pose.position.y = y;
            
            // 2. 计算航向角，并转换为四元数
            double yaw = spline.calc_yaw(s);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            point.pose.orientation = tf2::toMsg(q);

            // 3. 计算速度 (目前简化为恒定线速度，不考虑起点加速)
            // 如果需要梯形加速，可以在这里根据 s 进行分段判断
            point.velocity.linear.x = target_speed_ * std::cos(yaw);
            point.velocity.linear.y = target_speed_ * std::sin(yaw);

            // 4. 计算相对时间 ( t = s / v )
            double t = s / target_speed_;
            point.time_from_start.sec = static_cast<int32_t>(t);
            point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

            trajectory_msg->trajectory_points.push_back(point);
        }

        // 发布轨迹
        publish_trajectory(trajectory_msg);
        
        // 发布后清空当前路径，等待局部规划器的下一次更新
        current_local_path_ = nullptr;
    }

private:
    std::string base_frame_;
    std::string odom_frame_;
    double target_speed_;

    // 订阅局部路径和里程计的智能指针
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // 发布轨迹的智能指针
    rclcpp::Publisher<custom_motion_plan_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
    
    // 定时器智能指针
    rclcpp::TimerBase::SharedPtr trajectory_publish_timer_;

    // TF 相关智能指针
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 缓存数据
    nav_msgs::msg::Path::SharedPtr current_local_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
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
