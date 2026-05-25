#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

using namespace std::chrono_literals;

class PcdToPointCloudNode : public rclcpp::Node
{
public:
  PcdToPointCloudNode() : Node("pcd_to_pointcloud_node")
  {
    this->declare_parameter<std::string>("file_name", "");
    this->declare_parameter<std::string>("tf_frame", "map");

    std::string file_name;
    this->get_parameter("file_name", file_name);
    this->get_parameter("tf_frame", tf_frame_);

    if (file_name.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'file_name' is empty!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading PCD file: %s", file_name.c_str());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", file_name.c_str());
      return;
    }

    pcl::toROSMsg(*cloud, cloud_msg_);
    cloud_msg_.header.frame_id = tf_frame_;

    // Use Transient Local QoS for static map publishing
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_pcd", qos);

    // Publish once initially
    cloud_msg_.header.stamp = this->now();
    pub_->publish(cloud_msg_);
    RCLCPP_INFO(this->get_logger(), "PCD file published to 'cloud_pcd' topic with frame_id: %s", tf_frame_.c_str());

    // We can also publish it periodically just to be safe, e.g., 1 Hz
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&PcdToPointCloudNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cloud_msg_.header.stamp = this->now();
    pub_->publish(cloud_msg_);
  }

  std::string tf_frame_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcdToPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
