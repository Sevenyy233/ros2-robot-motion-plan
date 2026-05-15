#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudToOctomapNode : public rclcpp::Node {
public:
    PointCloudToOctomapNode() : Node("pointcloud_to_octomap") {
        // Declare parameters
        this->declare_parameter<double>("resolution", 0.1);
        this->declare_parameter<double>("prob_hit", 0.7);
        this->declare_parameter<double>("prob_miss", 0.4);
        this->declare_parameter<double>("thres_min", 0.12);
        this->declare_parameter<double>("thres_max", 0.97);

        // Subscribers and Publishers
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "map_points", 10,
            std::bind(&PointCloudToOctomapNode::pointCloudCallback, this, std::placeholders::_1));
            
        pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud2转换Octomap节点启动成功！");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        double resolution = this->get_parameter("resolution").as_double();
        double prob_hit = this->get_parameter("prob_hit").as_double();
        double prob_miss = this->get_parameter("prob_miss").as_double();
        double thres_min = this->get_parameter("thres_min").as_double();
        double thres_max = this->get_parameter("thres_max").as_double();

        // Convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "未接收到PointCloud2数据!");
            return;
        }

        // Initialize Octomap
        octomap::OcTree tree(resolution);
        tree.setProbHit(prob_hit);
        tree.setProbMiss(prob_miss);
        tree.setClampingThresMin(thres_min);
        tree.setClampingThresMax(thres_max);

        // Iterate through point cloud and update the Octomap
        for (const auto& point : cloud->points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                continue;
            }
            // Add point to the tree
            tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }

        // Update inner occupancy values
        tree.updateInnerOccupancy();

        // Convert Octomap to ROS message and publish
        octomap_msgs::msg::Octomap map_msg;
        map_msg.header.frame_id = msg->header.frame_id;
        map_msg.header.stamp = msg->header.stamp;
        
        bool res = octomap_msgs::fullMapToMsg(tree, map_msg);
        if (res) {
            pub_->publish(map_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Octomap转换为ROS消息失败！");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToOctomapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
