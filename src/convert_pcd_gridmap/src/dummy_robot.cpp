#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

class DummyRobot : public rclcpp::Node
{
public:
    DummyRobot() : Node("dummy_robot"), x_(0.0), y_(0.0), z_(0.0), roll_(0.0), pitch_(0.0), theta_(0.0)
    {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", 10, std::bind(&DummyRobot::cmdVelCallback,
                this, std::placeholders::_1));
        
        grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "grid_map", 10, std::bind(&DummyRobot::gridMapCallback,
                this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        last_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DummyRobot::update,this));

        RCLCPP_INFO(this->get_logger(),"Dummy robot 运动学仿真节点已启动！");
    }

private:
    void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    }

    double getElevation(double px, double py, double default_z) {
        if (map_.exists("elevation")) {
            grid_map::Position pos(px, py);
            if (map_.isInside(pos)) {
                double z = map_.atPosition("elevation", pos);
                if (!std::isnan(z)) return z;
            }
        }
        return default_z;
    }

    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_vel_ = msg->twist;
    }

    void update()
    {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double vx = current_vel_.linear.x;
        double vy = current_vel_.linear.y;
        double vth = current_vel_.angular.z;

        // 根据速度计算位移
        double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * dt;
        double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * dt;
        double delta_th = vth * dt;

        // 更新位置
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_th;

        // 根据GridMap更新高度(Z轴)和姿态(Roll, Pitch)
        double d = 0.15; // 采样距离，用于计算坡度
        if (map_.exists("elevation")) {
            z_ = getElevation(x_, y_, z_);

            // 计算前、后、左、右的高度
            double z_f = getElevation(x_ + d * cos(theta_), y_ + d * sin(theta_), z_);
            double z_b = getElevation(x_ - d * cos(theta_), y_ - d * sin(theta_), z_);
            double z_l = getElevation(x_ - d * sin(theta_), y_ + d * cos(theta_), z_);
            double z_r = getElevation(x_ + d * sin(theta_), y_ - d * cos(theta_), z_);

            // 根据高度差计算倾角 (Pitch 和 Roll)
            pitch_ = -atan2(z_f - z_b, 2.0 * d);
            roll_  = atan2(z_l - z_r, 2.0 * d);
        }

        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, theta_);

        // 1、发布TF：odom->base_footprint
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = z_;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);

        // 2、发布Odometry消息
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = z_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub_->publish(odom);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist current_vel_;
    rclcpp::Time last_time_;
    double x_, y_, z_, roll_, pitch_, theta_;
    grid_map::GridMap map_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto dummy_robot = std::make_shared<DummyRobot>();
    rclcpp::spin(dummy_robot);
    rclcpp::shutdown();
    return 0;
}