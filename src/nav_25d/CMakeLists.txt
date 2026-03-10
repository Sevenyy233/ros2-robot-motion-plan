cmake_minimum_required(VERSION 3.8)
project(nav_25d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_library(nav_25d_layer SHARED
  src/gradient_layer.cpp
)

target_include_directories(nav_25d_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav_25d_layer
  rclcpp
  std_msgs
  sensor_msgs
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_plugins.xml)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
)

install(TARGETS nav_25d_layer odom_tf_broadcaster
  EXPORT export_nav_25d_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config params rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES nav2_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(nav_25d_layer)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
)

ament_package()
