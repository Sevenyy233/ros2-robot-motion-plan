cmake_minimum_required(VERSION 3.8)
project(convert_pcd2_available_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(pointcloud_to_gridmap src/pointcloud_to_gridmap.cpp)
ament_target_dependencies(pointcloud_to_gridmap
  rclcpp
  grid_map
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_ros
  sensor_msgs
  pcl_conversions
)

add_executable(dummy_robot src/dummy_robot.cpp)
ament_target_dependencies(dummy_robot
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  grid_map_msgs
  grid_map_ros
  grid_map_core
)

add_executable(pointcloud_to_octomap src/pointcloud_to_octomap.cpp)
ament_target_dependencies(pointcloud_to_octomap
  rclcpp
  sensor_msgs
  octomap
  octomap_msgs
  pcl_conversions
  grid_map_core
)

add_executable(combine_pointcloud_lidar src/combine_pointcloud_lidar.cpp)
ament_target_dependencies(combine_pointcloud_lidar
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  geometry_msgs
  grid_map_ros
  pcl_ros
)
target_link_libraries(combine_pointcloud_lidar
  ${PCL_LIBRARIES}
)

install(TARGETS
  pointcloud_to_gridmap
  dummy_robot
  pointcloud_to_octomap
  combine_pointcloud_lidar
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch pcd rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
