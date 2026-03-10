#include "nav_25d/gradient_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp" // Required for doTransform

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav_25d
{

GradientLayer::GradientLayer()
{
  costmap_ = NULL; 
}

GradientLayer::~GradientLayer()
{
}

void GradientLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic_name", rclcpp::ParameterValue("/velodyne_points"));
  declareParameter("max_slope_limit", rclcpp::ParameterValue(0.5)); // radians ~ 30 degrees
  declareParameter("slope_cost_factor", rclcpp::ParameterValue(100.0));
  declareParameter("min_height_threshold", rclcpp::ParameterValue(-2.0));
  declareParameter("max_height_threshold", rclcpp::ParameterValue(3.0));
  
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "topic_name", topic_name_);
  node->get_parameter(name_ + "." + "max_slope_limit", max_slope_limit_);
  node->get_parameter(name_ + "." + "slope_cost_factor", slope_cost_factor_);
  node->get_parameter(name_ + "." + "min_height_threshold", min_height_threshold_);
  node->get_parameter(name_ + "." + "max_height_threshold", max_height_threshold_);

  rolling_window_ = layered_costmap_->isRolling();

  point_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name_, rclcpp::SensorDataQoS(),
    std::bind(&GradientLayer::pointCloudCallback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
  
  matchSize();
}

void GradientLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
  
  unsigned int size = size_x_ * size_y_;
  min_height_grid_.assign(size, 1000.0f); // Init with high value
  max_height_grid_.assign(size, -1000.0f); // Init with low value
  count_grid_.assign(size, 0);
}

void GradientLayer::reset()
{
  matchSize();
  current_ = false;
}

void GradientLayer::onFootprintChanged()
{
  // Nothing to do for now
}

void GradientLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  last_cloud_ = msg;
  need_recalculation_ = true;
}

void GradientLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  // Check rolling window status
  bool current_rolling = layered_costmap_->isRolling();
  if (current_rolling != rolling_window_) {
    rolling_window_ = current_rolling;
    matchSize();
  }
  
  // If rolling window, we might need to shift or reset. 
  // For simplicity, we reset if the origin moves significantly or just rely on decay.
  // But here we will just process the latest cloud and update the relevant cells.
  // In a real robust implementation, we would scroll the map. 
  // Since we inherit from CostmapLayer, resizeMap handles some of this, but our custom vectors need manual handling.
  // For this prototype, we'll just re-populate from the latest cloud every cycle (reactive).
  // Ideally, we accumulate. But accumulation requires raytracing to clear.
  // Let's stick to "latest cloud" approach for responsiveness to dynamic terrain.
  
  // Reset grids for this cycle (Reactive approach)
  // Optimization: Only reset if we are going to process a new cloud? 
  // No, if we don't process, we keep old data? 
  // Let's clear every time to avoid ghosts if the cloud is dense enough.
  std::fill(min_height_grid_.begin(), min_height_grid_.end(), 1000.0f);
  std::fill(max_height_grid_.begin(), max_height_grid_.end(), -1000.0f);
  std::fill(count_grid_.begin(), count_grid_.end(), 0);

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!last_cloud_) return;
      cloud = last_cloud_;
  }

  // Transform cloud
  sensor_msgs::msg::PointCloud2 cloud_out;
  std::string global_frame = layered_costmap_->getGlobalFrameID();
  
  try {
    // Timeout is critical here
    geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
        global_frame, cloud->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
    tf2::doTransform(*cloud, cloud_out, transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "Transform failed: %s", ex.what());
    return;
  }

  // Iterate points
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud_out, pcl_cloud);

  for (const auto& point : pcl_cloud.points) {
    if (point.z < min_height_threshold_ || point.z > max_height_threshold_) continue;
    
    unsigned int mx, my;
    if (worldToMap(point.x, point.y, mx, my)) {
      unsigned int index = getIndex(mx, my);
      if (point.z < min_height_grid_[index]) min_height_grid_[index] = point.z;
      if (point.z > max_height_grid_[index]) max_height_grid_[index] = point.z;
      count_grid_[index]++;
      
      // Expand bounds
      *min_x = std::min(*min_x, (double)point.x);
      *min_y = std::min(*min_y, (double)point.y);
      *max_x = std::max(*max_x, (double)point.x);
      *max_y = std::max(*max_y, (double)point.y);
    }
  }
}

void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  unsigned int size_x = getSizeInCellsX();
  unsigned int size_y = getSizeInCellsY();
  double resolution = getResolution();

  // Constrain bounds
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min((int)size_x, max_i);
  max_j = std::min((int)size_y, max_j);

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      unsigned int index = getIndex(i, j);
      
      if (count_grid_[index] == 0) continue;

      // Check neighbors for slope calculation
      // We use the max height difference in the neighborhood
      float current_height = (min_height_grid_[index] + max_height_grid_[index]) / 2.0;
      float max_diff = 0.0;
      
      // 4-connectivity
      int dx[] = {0, 0, 1, -1};
      int dy[] = {1, -1, 0, 0};
      
      for (int k = 0; k < 4; k++) {
        int nx = i + dx[k];
        int ny = j + dy[k];
        
        if (nx >= 0 && nx < (int)size_x && ny >= 0 && ny < (int)size_y) {
          unsigned int n_index = getIndex(nx, ny);
          if (count_grid_[n_index] > 0) {
             float n_height = (min_height_grid_[n_index] + max_height_grid_[n_index]) / 2.0;
             float diff = std::abs(current_height - n_height);
             if (diff > max_diff) max_diff = diff;
          }
        }
      }
      
      // Calculate Slope
      // slope = tan(theta) = rise / run
      double slope = atan2(max_diff, resolution);
      
      unsigned char cost = 0;
      if (slope > max_slope_limit_) {
        cost = LETHAL_OBSTACLE;
      } else {
        // Linear mapping of slope to cost
        double normalized_slope = slope / max_slope_limit_;
        cost = static_cast<unsigned char>(normalized_slope * 253.0); 
        // Or user factor
        // cost = static_cast<unsigned char>(std::min(253.0, slope * slope_cost_factor_));
      }

      // Update master grid
      // We only increase cost, never decrease (conservative)
      unsigned char old_cost = master_grid.getCost(i, j);
      if (old_cost == NO_INFORMATION || cost > old_cost) {
        master_grid.setCost(i, j, cost);
      }
    }
  }
}

}  // namespace nav_25d
