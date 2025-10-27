# 传感器状态消息（简洁版）

std_msgs/Header header

# 传感器标识
string sensor_name          # 传感器名称: "livox_mid360", "realsense_d455", "wheel_odometry"
string sensor_type          # 类型: "lidar", "rgbd", "odometry"

# 状态信息
bool is_active              # 传感器是否激活
float32 update_rate         # 更新频率 (Hz)

# 诊断信息
uint8 status_level          # 0=OK, 1=WARN, 2=ERROR
string status_message       # 状态描述

# 状态级别常量
uint8 OK = 0
uint8 WARN = 1
uint8 ERROR = 2
