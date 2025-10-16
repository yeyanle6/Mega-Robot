cmake_minimum_required(VERSION 3.8)
project(t_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    config
    scripts
    params
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

# Make scripts executable
install(
  PROGRAMS
    scripts/time_sync_monitor.py
    scripts/imu_relay.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
