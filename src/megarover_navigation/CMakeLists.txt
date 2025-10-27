cmake_minimum_required(VERSION 3.8)
project(megarover_navigation)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorStatus.msg"
  "msg/HealthStatus.msg"
  "srv/SwitchMode.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# 安装Python可执行文件
install(PROGRAMS
  megarover_navigation/sensor_detector.py
  megarover_navigation/health_monitor.py
  megarover_navigation/odometry_fusion.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装config文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 安装脚本
install(PROGRAMS
  scripts/start_modular_slam.sh
  scripts/monitor_rtabmap.sh
  scripts/diagnose_rtabmap.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
