#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class CombinePointCloudLidar : public rclcpp::Node {
public:
    CombinePointCloudLidar() : Node("combine_pointcloud_lidar"), static_cloud_received_(false) {
        // Parameters
        this->declare_parameter<std::string>("static_map_topic", "/map_points_static");
        this->declare_parameter<std::string>("lidar_topic", "/lidar3_points");
        this->declare_parameter<std::string>("output_topic", "/map_points");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("lidar_timeout", 1.0); // 1秒收不到雷达数据认为雷达掉线
        this->declare_parameter<double>("publish_rate", 5.0);  // 5Hz发布融合后的点云

        std::string static_map_topic = this->get_parameter("static_map_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        lidar_timeout_ = this->get_parameter("lidar_timeout").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        sub_static_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            static_map_topic, 10,
            std::bind(&CombinePointCloudLidar::staticMapCallback, this, std::placeholders::_1));

        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10,
            std::bind(&CombinePointCloudLidar::lidarCallback, this, std::placeholders::_1));

        // Publisher
        pub_combined_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        // Timer for publishing combined cloud
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&CombinePointCloudLidar::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "点云融合节点启动成功，等待静态地图...");
    }

private:
    void staticMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!static_cloud_received_) {
            pcl::fromROSMsg(*msg, static_cloud_);
            static_cloud_received_ = true;
            RCLCPP_INFO(this->get_logger(), "接收到静态地图点云，点数: %lu", static_cloud_.size());
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_lidar_msg_ = msg;
        last_lidar_time_ = this->get_clock()->now();
    }

    void timerCallback() {
        if (!static_cloud_received_) {
            // 如果还没收到静态地图，什么都不做
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>(static_cloud_));

        bool lidar_active = false;
        if (latest_lidar_msg_ != nullptr) {
            double age = (this->get_clock()->now() - last_lidar_time_).seconds();
            if (age < lidar_timeout_) {
                lidar_active = true;
            }
        }

        if (lidar_active) {
            // 雷达在线，将雷达点云转换到 map 坐标系并合并
            try {
                // 等待 TF 树中存在目标变换
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    map_frame_, latest_lidar_msg_->header.frame_id, rclcpp::Time(0));

                // 利用 pcl_ros 的工具直接转换点云
                sensor_msgs::msg::PointCloud2 transformed_lidar_msg;
                pcl_ros::transformPointCloud(map_frame_, transform, *latest_lidar_msg_, transformed_lidar_msg);

                pcl::PointCloud<pcl::PointXYZ> lidar_cloud;
                pcl::fromROSMsg(transformed_lidar_msg, lidar_cloud);

                *combined_cloud += lidar_cloud;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "无法将雷达点云转换到地图坐标系: %s", ex.what());
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "雷达数据超时或未启动，仅发布静态地图");
        }

        // 转换回 ROS 消息并发布
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*combined_cloud, output_msg);
        output_msg.header.frame_id = map_frame_;
        output_msg.header.stamp = this->get_clock()->now();
        
        pub_combined_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_static_map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_combined_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    pcl::PointCloud<pcl::PointXYZ> static_cloud_;
    bool static_cloud_received_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_msg_;
    rclcpp::Time last_lidar_time_;

    std::string map_frame_;
    double lidar_timeout_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CombinePointCloudLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}