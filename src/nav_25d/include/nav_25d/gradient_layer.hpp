#ifndef NAV_25D__GRADIENT_LAYER_HPP_
#define NAV_25D__GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <mutex>
#include <vector>

namespace nav_25d
{

class GradientLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  GradientLayer();
  virtual ~GradientLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  
  virtual void matchSize();
  virtual void reset();
  virtual void onFootprintChanged();
  virtual bool isClearable() {return false;}

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  
  // Grids to store height statistics
  std::vector<float> min_height_grid_;
  std::vector<float> max_height_grid_;
  std::vector<int> count_grid_;

  bool need_recalculation_;
  
  double max_slope_limit_; 
  double slope_cost_factor_;
  double min_height_threshold_; // Filter noise
  double max_height_threshold_; // Filter ceiling
  std::string topic_name_;
  
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  std::mutex cloud_mutex_;
  bool rolling_window_;
};

}  // namespace nav_25d

#endif  // NAV_25D__GRADIENT_LAYER_HPP_
