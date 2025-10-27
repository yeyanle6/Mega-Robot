# 系统健康状态消息（简洁版）

std_msgs/Header header

# 系统标识
string system_name          # "rtabmap_slam", "nav2_navigation"
string operating_mode       # "fusion", "lidar_only", "rgbd_only", "odom_only"

# 资源使用
float32 cpu_usage_percent   # CPU使用率 (%)
float32 memory_usage_mb     # 内存使用 (MB)

# 运行状态
bool is_operational         # 系统是否正常运行
uint32 uptime_seconds       # 运行时间 (秒)

# 传感器状态
uint8 active_sensors        # 活动传感器数量
string[] sensor_list        # 传感器列表
