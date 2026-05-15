#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <vector>
#include <cmath>

class PointCloudToOctomapNode : public rclcpp::Node {
public:
    PointCloudToOctomapNode() : Node("pointcloud_to_octomap") {
        // Declare parameters
        this->declare_parameter<double>("resolution", 0.1);
        this->declare_parameter<double>("prob_hit", 0.7);
        this->declare_parameter<double>("prob_miss", 0.4);
        this->declare_parameter<double>("thres_min", 0.12);
        this->declare_parameter<double>("thres_max", 0.97);
        this->declare_parameter<int>("hole_filling_radius", 1);

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

        // Fill holes in the octomap
        int hole_filling_radius = this->get_parameter("hole_filling_radius").as_int();
        fillHoles(tree, hole_filling_radius);

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

    void fillHoles(octomap::OcTree& tree, int radius) {
        if (radius <= 0) return;

        RCLCPP_INFO_ONCE(this->get_logger(), "开始填补Octomap空缺，半径: %d", radius);

        std::vector<octomap::point3d> points_to_add;

        // 获取所有已占用的节点
        octomap::KeySet occupied_keys;
        for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
            if (tree.isNodeOccupied(*it)) {
                occupied_keys.insert(it.getKey());
            }
        }

        // 寻找候选空节点（与占用节点相邻的节点）
        octomap::KeySet empty_candidates;
        for (const auto& key : occupied_keys) {
            for (int dx = -radius; dx <= radius; ++dx) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dz = -radius; dz <= radius; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        octomap::OcTreeKey n_key = key;
                        n_key[0] += dx;
                        n_key[1] += dy;
                        n_key[2] += dz;
                        if (occupied_keys.find(n_key) == occupied_keys.end()) {
                            empty_candidates.insert(n_key);
                        }
                    }
                }
            }
        }

        // 判断候选节点周围的占用节点数量
        // 计算阈值：假设是一个平面，半径内的平面节点数约为 (2r+1)^2 - 1
        // 我们取其 40% 作为阈值，这样既能补齐孔洞，又不会过度膨胀
        int threshold = std::max(4, static_cast<int>(std::pow(2 * radius + 1, 2) * 0.4));

        for (const auto& key : empty_candidates) {
            int occupied_count = 0;
            for (int dx = -radius; dx <= radius; ++dx) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dz = -radius; dz <= radius; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        octomap::OcTreeKey n_key = key;
                        n_key[0] += dx;
                        n_key[1] += dy;
                        n_key[2] += dz;
                        if (occupied_keys.find(n_key) != occupied_keys.end()) {
                            occupied_count++;
                        }
                    }
                }
            }
            
            if (occupied_count >= threshold) {
                points_to_add.push_back(tree.keyToCoord(key));
            }
        }

        // 将补齐的节点加入到 Octomap 中
        for (const auto& pt : points_to_add) {
            tree.updateNode(pt, true);
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "填补完成，共补齐了 %zu 个节点", points_to_add.size());
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
