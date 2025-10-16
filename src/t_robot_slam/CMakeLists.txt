cmake_minimum_required(VERSION 3.8)
project(t_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Create executable
add_executable(map_odom_tf_publisher src/map_odom_tf_publisher.cpp)
ament_target_dependencies(map_odom_tf_publisher
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

# Create executable v2
add_executable(map_odom_tf_publisher_v2 src/map_odom_tf_publisher_v2.cpp)
ament_target_dependencies(map_odom_tf_publisher_v2
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

# Point cloud preprocessor
add_executable(pointcloud_preprocessor src/pointcloud_preprocessor.cpp)
ament_target_dependencies(pointcloud_preprocessor
  rclcpp
  sensor_msgs
  tf2_ros
  tf2
  pcl_conversions
  pcl_ros
)

# Install executables
install(TARGETS
  map_odom_tf_publisher
  map_odom_tf_publisher_v2
  pointcloud_preprocessor
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY
    launch
    config
    params
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)

# Make scripts executable
install(
  PROGRAMS
    scripts/slam_monitor.py
    scripts/mapping_recorder.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()