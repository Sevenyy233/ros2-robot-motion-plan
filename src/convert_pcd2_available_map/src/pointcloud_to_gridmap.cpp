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
        this->declare_parameter<std::string>("slope_layer_name", "slope");
        this->declare_parameter<std::string>("traversability_layer_name", "traversability");
        this->declare_parameter<int>("hole_filling_radius", 2); // Radius for filling holes in grid (0 to disable)
        this->declare_parameter<double>("max_slope_angle", 45.0); // 最大允许坡度(度)
        this->declare_parameter<double>("min_height", -10.0); // 最小有效高度，过滤掉过低的点
        this->declare_parameter<double>("max_height", 2.0); // 最大有效高度，过滤掉过高的点(如屋顶)

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
        std::string slope_layer_name = this->get_parameter("slope_layer_name").as_string();
        std::string trav_layer_name = this->get_parameter("traversability_layer_name").as_string();
        double min_height = this->get_parameter("min_height").as_double();
        double max_height = this->get_parameter("max_height").as_double();

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
            // 过滤掉不在有效高度范围内的点（例如屋顶）
            if (point.z < min_height || point.z > max_height) {
                continue;
            }
            if (point.x < min_x) min_x = point.x;
            if (point.x > max_x) max_x = point.x;
            if (point.y < min_y) min_y = point.y;
            if (point.y > max_y) max_y = point.y;
        }

        if (min_x > max_x || min_y > max_y) {
            RCLCPP_WARN(this->get_logger(), "过滤后没有有效的点云数据!");
            return;
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
        // 初始化 GridMap 并添加 elevation, slope, traversability 三个层
        grid_map::GridMap map({layer_name, slope_layer_name, trav_layer_name});
        map.setFrameId(msg->header.frame_id);
        map.setTimestamp(rclcpp::Time(msg->header.stamp).nanoseconds());
        map.setGeometry(grid_map::Length(length_x, length_y), resolution, grid_map::Position(center_x, center_y));
        map.setBasicLayers({layer_name, slope_layer_name, trav_layer_name});

        // Iterate through point cloud and update the GridMap layer
        for (const auto& point : cloud->points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                continue;
            }
            
            // 过滤掉不在有效高度范围内的点
            if (point.z < min_height || point.z > max_height) {
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

        // ================= 添加坡度层 (Slope) 和 通行度层 (Traversability) =================
        double max_slope_angle_deg = this->get_parameter("max_slope_angle").as_double();
        double max_slope_rad = max_slope_angle_deg * M_PI / 180.0;

        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
            grid_map::Index index = *iterator;
            if (!map.isValid(index, layer_name)) {
                continue;
            }

            // 使用中心差分法计算X和Y方向的高程梯度
            double dz_dx = 0.0;
            double dz_dy = 0.0;
            bool has_grad = false;

            grid_map::Index idx_x_prev(index(0) - 1, index(1));
            grid_map::Index idx_x_next(index(0) + 1, index(1));
            if (map.isValid(idx_x_prev, layer_name) && map.isValid(idx_x_next, layer_name)) {
                dz_dx = (map.at(layer_name, idx_x_next) - map.at(layer_name, idx_x_prev)) / (2.0 * resolution);
                has_grad = true;
            }

            grid_map::Index idx_y_prev(index(0), index(1) - 1);
            grid_map::Index idx_y_next(index(0), index(1) + 1);
            if (map.isValid(idx_y_prev, layer_name) && map.isValid(idx_y_next, layer_name)) {
                dz_dy = (map.at(layer_name, idx_y_next) - map.at(layer_name, idx_y_prev)) / (2.0 * resolution);
                has_grad = true;
            }

            if (has_grad) {
                // 计算合成坡度（弧度）
                double slope = std::atan(std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy));
                map.at(slope_layer_name, index) = slope;

                // 计算通行度 Traversability (0.0: 不可通行, 1.0: 完全可通行)
                double traversability = 1.0 - (slope / max_slope_rad);
                if (traversability < 0.0) traversability = 0.0; // 超过最大坡度，不可通行
                map.at(trav_layer_name, index) = traversability;
            } else {
                // 边界情况，假设平坦
                map.at(slope_layer_name, index) = 0.0;
                map.at(trav_layer_name, index) = 1.0;
            }
        }
        // =================================================================================

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
