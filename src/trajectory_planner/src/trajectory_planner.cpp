#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
        this->declare_parameter("map_frame", "map");
        
        // Pure Pursuit parameters
        this->declare_parameter("lookahead_min", 0.3);
        this->declare_parameter("lookahead_max", 2.0);
        this->declare_parameter("lookahead_scale", 0.3);
        this->declare_parameter("max_linear_speed", 0.5);
        this->declare_parameter("max_angular_speed", 1.0);
        this->declare_parameter("goal_tolerance", 0.15);
        this->declare_parameter("yaw_tolerance", 0.087);
        this->declare_parameter("max_slope_angle", 45.0);
        
        // 获取参数
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        
        lookahead_min_ = this->get_parameter("lookahead_min").as_double();
        lookahead_max_ = this->get_parameter("lookahead_max").as_double();
        lookahead_scale_ = this->get_parameter("lookahead_scale").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
        max_slope_angle_ = this->get_parameter("max_slope_angle").as_double();
        
        goal_reached_ = false;
        aligning_yaw_ = false;

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

        // 发布底层控制速度
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel",
            10
        );

        // 创建定时器：负责生成轨迹并发布 (10Hz)
        trajectory_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryPlanner::trajectory_timer_callback, this)
        );

        // 创建控制定时器：负责速度计算与发布 (20Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TrajectoryPlanner::control_timer_callback, this)
        );
    
        RCLCPP_INFO(this->get_logger(), "轨迹规划与跟踪控制器初始化成功！(支持履带原地转向)");
    }

private:
    void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (current_local_path_ && !msg->poses.empty() && !current_local_path_->poses.empty()) {
            auto old_goal = current_local_path_->poses.back().pose.position;
            auto new_goal = msg->poses.back().pose.position;
            double dist = std::hypot(new_goal.x - old_goal.x, new_goal.y - old_goal.y);
            if (dist > 0.1) {
                goal_reached_ = false;
                aligning_yaw_ = false;
            }
        } else {
            goal_reached_ = false;
            aligning_yaw_ = false;
        }
        current_local_path_ = msg;
        last_path_time_ = this->now();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = msg;
    }

    void publish_trajectory(const custom_motion_plan_msgs::msg::RobotTrajectory::SharedPtr msg)
    {
        trajectory_pub_->publish(*msg);
    }

    void publish_zero_velocity()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = base_frame_;
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.linear.z = 0.0;
        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    // 轨迹规划定时器
    void trajectory_timer_callback()
    {
        // 确保局部路径是最新的
        if (!current_local_path_ || current_local_path_->poses.size() < 3) {
            return; 
        }
        
        // 路径超时不进行轨迹生成
        if ((this->now() - last_path_time_).seconds() > 0.5) {
            return;
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
            point.pose.orientation.x = q.x();
            point.pose.orientation.y = q.y();
            point.pose.orientation.z = q.z();
            point.pose.orientation.w = q.w();
            point.velocity.linear.x = max_linear_speed_ * std::cos(yaw);
            point.velocity.linear.y = max_linear_speed_ * std::sin(yaw);

            // 3. 计算相对时间 ( t = s / v )
            double t = s / max_linear_speed_;
            point.time_from_start.sec = static_cast<int32_t>(t);
            point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

            trajectory_msg->trajectory_points.push_back(point);
        }

        // 发布轨迹
        publish_trajectory(trajectory_msg);
    }

    // 控制定时器：负责底层速度下发与履带运动学约束处理
    void control_timer_callback()
    {
        // 如果没有收到局部路径，或者路径数据太老（例如局部规划器停止发布），则停车
        if (!current_local_path_ || (this->now() - last_path_time_).seconds() > 0.5) {
            publish_zero_velocity();
            return;
        }

        if (current_local_path_->poses.empty()) {
            publish_zero_velocity();
            return;
        }

        // 1. 获取机器人在 map 下的当前位姿
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, rclcpp::Time(0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF 查找失败: %s", ex.what());
            publish_zero_velocity();
            return;
        }

        double cx = transform.transform.translation.x;
        double cy = transform.transform.translation.y;
        
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, cyaw;
        m.getRPY(roll, pitch, cyaw);

        // 2. 在局部路径上寻找控制前视点 (Lookahead Point)
        double current_speed = 0.0;
        if (current_odom_) {
            current_speed = std::hypot(current_odom_->twist.twist.linear.x, current_odom_->twist.twist.linear.y);
        }
        
        double lookahead = lookahead_min_ + lookahead_scale_ * current_speed;
        lookahead = std::max(lookahead_min_, std::min(lookahead_max_, lookahead));
        
        double lx = cx, ly = cy;
        bool found = false;
        int goal_idx = -1;
        for (size_t i = 0; i < current_local_path_->poses.size(); ++i) {
            double px = current_local_path_->poses[i].pose.position.x;
            double py = current_local_path_->poses[i].pose.position.y;
            double d = std::hypot(px - cx, py - cy);
            if (d >= lookahead) {
                lx = px;
                ly = py;
                found = true;
                goal_idx = i;
                break;
            }
        }

        // 如果路径总长度都比前视距离短，直接取路径终点
        if (!found) {
            lx = current_local_path_->poses.back().pose.position.x;
            ly = current_local_path_->poses.back().pose.position.y;
            goal_idx = current_local_path_->poses.size() - 1;
        }

        double dx = lx - cx;
        double dy = ly - cy;
        double Ld = std::hypot(dx, dy);

        // 如果距离目标非常近，执行终点对齐或停止
        auto final_pose = current_local_path_->poses.back().pose;
        auto final_pt = final_pose.position;
        if (std::hypot(cx - final_pt.x, cy - final_pt.y) < goal_tolerance_) {
            // 已到达 XY 位置，开始原地对齐朝向
            tf2::Quaternion fq(
                final_pose.orientation.x,
                final_pose.orientation.y,
                final_pose.orientation.z,
                final_pose.orientation.w
            );
            tf2::Matrix3x3 fm(fq);
            double froll, fpitch, final_yaw;
            fm.getRPY(froll, fpitch, final_yaw);

            double yaw_diff = final_yaw - cyaw;
            while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

            if (std::abs(yaw_diff) < yaw_tolerance_) {
                if (!goal_reached_) {
                    RCLCPP_INFO(this->get_logger(), "已到达目标点，且朝向已对齐！");
                    goal_reached_ = true;
                }
                publish_zero_velocity();
                return;
            } else {
                if (!aligning_yaw_) {
                    RCLCPP_INFO(this->get_logger(), "已到达目标点位置，正在原地旋转对齐朝向...");
                    aligning_yaw_ = true;
                }
                
                geometry_msgs::msg::TwistStamped cmd;
                cmd.header.stamp = this->now();
                cmd.header.frame_id = base_frame_;
                cmd.twist.linear.x = 0.0;
                
                double angular_z_align = yaw_diff * 1.5;
                angular_z_align = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_z_align));
                
                // 保证最小旋转速度防止卡死
                double min_w = 0.2;
                if (angular_z_align > 0 && angular_z_align < min_w) angular_z_align = min_w;
                if (angular_z_align < 0 && angular_z_align > -min_w) angular_z_align = -min_w;
                
                cmd.twist.angular.z = angular_z_align;
                cmd_vel_pub_->publish(cmd);
                return;
            }
        }

        // 3. 计算偏航角误差 (用于初始对齐判断，非 Pure Pursuit 核心)
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = target_yaw - cyaw;
        
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = base_frame_;

        // 4. 履带运动学双态控制与 Pure Pursuit 计算
        if (std::abs(yaw_error) > M_PI / 6.0) { 
            // 状态 A：航向偏差过大 (约30度)，执行原地转向
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 1.5 * yaw_error; // kp_yaw
        } else {
            // 状态 B：执行 Pure Pursuit 跟随
            double local_y = -std::sin(cyaw) * dx + std::cos(cyaw) * dy;
            double curvature = 2.0 * local_y / (Ld * Ld);
            curvature = std::max(-1.0, std::min(1.0, curvature));
            
            cmd.twist.angular.z = curvature * max_linear_speed_;
            
            // 多级减速策略
            double linear_x = max_linear_speed_;
            
            // 1) 转弯减速
            double curve_factor = 1.0 - std::abs(curvature) * 0.8;
            linear_x *= std::max(0.2, curve_factor);
            
            // 2) 终点减速
            int remaining = current_local_path_->poses.size() - goal_idx;
            if (remaining < 5) {
                linear_x *= std::max(0.1, remaining / 5.0);
            }
            
            cmd.twist.linear.x = linear_x;
        }

        // 5. 速度限幅发布
        cmd.twist.linear.x = std::max(-max_linear_speed_, std::min(max_linear_speed_, cmd.twist.linear.x));
        cmd.twist.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, cmd.twist.angular.z));

        cmd_vel_pub_->publish(cmd);
    }

private:
    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    
    double lookahead_min_;
    double lookahead_max_;
    double lookahead_scale_;
    double max_linear_speed_;
    double max_angular_speed_;
    double goal_tolerance_;
    double yaw_tolerance_;
    double max_slope_angle_;
    
    bool goal_reached_;
    bool aligning_yaw_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<custom_motion_plan_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    
    rclcpp::TimerBase::SharedPtr trajectory_publish_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::Path::SharedPtr current_local_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    rclcpp::Time last_path_time_;
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
