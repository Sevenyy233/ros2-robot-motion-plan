#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class PointCloudToGridMapNode : public rclcpp::Node {
public:
    PointCloudToGridMapNode() : Node("pointcloud_to_gridmap") {
        // Declare parameters
        this->declare_parameter<double>("resolution", 0.1);
        this->declare_parameter<std::string>("layer_name", "elevation");
        this->declare_parameter<int>("hole_filling_radius", 2); // Radius for filling holes in grid (0 to disable)

        // Subscribers and Publishers
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "map_points", 10,
            std::bind(&PointCloudToGridMapNode::pointCloudCallback, this, std::placeholders::_1));
            
        pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);

        // parameters_callback_handle_ = this->add_on_set_parameters_callback(

        // )

        RCLCPP_INFO(this->get_logger(), "PointCloud2转换GridMap节点启动成功！");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        double resolution = this->get_parameter("resolution").as_double();
        std::string layer_name = this->get_parameter("layer_name").as_string();

        // Convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "未接收到PointCloud2数据!");
            return;
        }

        // Find min and max bounds to calculate GridMap size
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();

        for (const auto& point : cloud->points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                continue;
            }
            if (point.x < min_x) min_x = point.x;
            if (point.x > max_x) max_x = point.x;
            if (point.y < min_y) min_y = point.y;
            if (point.y > max_y) max_y = point.y;
        }

        // Add small margin to bounds
        min_x -= resolution;
        max_x += resolution;
        min_y -= resolution;
        max_y += resolution;

        double length_x = max_x - min_x;
        double length_y = max_y - min_y;
        double center_x = (max_x + min_x) / 2.0;
        double center_y = (max_y + min_y) / 2.0;

        // Initialize GridMap
        grid_map::GridMap map({layer_name});
        map.setFrameId(msg->header.frame_id);
        map.setTimestamp(rclcpp::Time(msg->header.stamp).nanoseconds());
        map.setGeometry(grid_map::Length(length_x, length_y), resolution, grid_map::Position(center_x, center_y));
        map.setBasicLayers({layer_name});

        // Iterate through point cloud and update the GridMap layer
        for (const auto& point : cloud->points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                continue;
            }

            grid_map::Position position(point.x, point.y);
            grid_map::Index index;
            if (map.getIndex(position, index)) {
                // If cell is uninitialized or new point has higher elevation, update it
                if (!map.isValid(index, layer_name) || point.z > map.at(layer_name, index)) {
                    map.at(layer_name, index) = point.z;
                }
            }
        }

        // Fill holes in the grid map to prevent empty spots that break path planning
        int hole_filling_radius = this->get_parameter("hole_filling_radius").as_int();
        if (hole_filling_radius > 0) {
            grid_map::GridMap map_filled = map;
            for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
                if (!map.isValid(*iterator, layer_name)) {
                    double sum = 0.0;
                    int count = 0;
                    grid_map::Index center_index = *iterator;
                    
                    // Search neighboring cells within the specified radius
                    for (int i = -hole_filling_radius; i <= hole_filling_radius; ++i) {
                        for (int j = -hole_filling_radius; j <= hole_filling_radius; ++j) {
                            grid_map::Index neighbor_index(center_index(0) + i, center_index(1) + j);
                            // Check if index is within map bounds and has valid data
                            if (neighbor_index(0) >= 0 && neighbor_index(0) < map.getSize()(0) &&
                                neighbor_index(1) >= 0 && neighbor_index(1) < map.getSize()(1)) {
                                if (map.isValid(neighbor_index, layer_name)) {
                                    sum += map.at(layer_name, neighbor_index);
                                    count++;
                                }
                            }
                        }
                    }
                    // If we found valid neighbors, assign the average value to fill the hole
                    if (count > 0) {
                        map_filled.at(layer_name, *iterator) = sum / count;
                    }
                }
            }
            map = map_filled;
        }

        // Convert GridMap to ROS message and publish
        std::unique_ptr<grid_map_msgs::msg::GridMap> message;
        message = grid_map::GridMapRosConverter::toMessage(map);
        pub_->publish(std::move(message));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_; 
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToGridMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
