# MegaRover Navigation Package

ROS2 navigation package for MegaRover3 robot with modular RTABMAP SLAM and Nav2 autonomous navigation capabilities.

## Features

- **Modular Architecture**: Automatically selects optimal configuration based on available sensors
- **Multi-Sensor Fusion**: Supports Livox MID360 LiDAR + Intel RealSense D455i depth camera
- **Intelligent Failover**: Automatic mode switching on sensor failure
- **Complete Navigation Stack**: SLAM mapping + Nav2 autonomous navigation
- **Health Monitoring**: Real-time diagnostics with automatic recovery
- **Performance Optimization**: Configurable for different hardware capabilities

## 系统架构

```
megarover_navigation/
├── config/               # 配置文件
│   ├── nav2_params.yaml           # Nav2参数
│   ├── rtabmap_fusion.yaml        # 融合模式配置
│   ├── rtabmap_lidar_only.yaml    # 激光雷达模式
│   ├── rtabmap_rgbd_only.yaml     # RGB-D模式
│   └── rtabmap_odom_only.yaml     # 纯里程计模式
├── launch/              # Launch文件
│   ├── navigation.launch.py        # 主入口
│   ├── modular_rtabmap.launch.py  # 模块化SLAM
│   └── megarover_nav2_slam.launch.py  # Nav2集成
├── megarover_navigation/  # Python节点
│   ├── sensor_detector.py  # 传感器检测
│   └── health_monitor.py   # 健康监控
├── rviz/                # 可视化配置
│   └── nav2_rviz_config.rviz
└── scripts/             # 工具脚本
    ├── start_modular_slam.sh
    ├── monitor_rtabmap.sh
    └── diagnose_rtabmap.sh
```

## 安装

### 1. 依赖安装

```bash
# Nav2导航栈
sudo apt install ros-$ROS_DISTRO-navigation2 \
                 ros-$ROS_DISTRO-nav2-bringup

# RTABMAP SLAM
sudo apt install ros-$ROS_DISTRO-rtabmap-ros

# 其他依赖
sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan \
                 python3-psutil
```

### 2. 构建包

```bash
cd ~/Code/Demo6
colcon build --packages-select megarover_navigation
source install/setup.bash
```

## 使用方法

### 快速启动

```bash
# SLAM建图 + 导航（默认）
ros2 launch megarover_navigation navigation.launch.py

# 仅SLAM建图
ros2 launch megarover_navigation navigation.launch.py mode:=slam_only

# 使用已有地图导航
ros2 launch megarover_navigation navigation.launch.py mode:=nav_only map:=/path/to/map.yaml
```

### 运行模式

#### 1. SLAM建图模式 (`slam_only`)
- 启动RTABMAP进行地图构建
- 根据传感器自动选择配置
- 不启动导航功能

```bash
ros2 launch megarover_navigation navigation.launch.py mode:=slam_only
```

#### 2. SLAM + 导航模式 (`slam_nav`)
- 同时进行建图和导航
- 适合探索未知环境
- 默认模式

```bash
ros2 launch megarover_navigation navigation.launch.py mode:=slam_nav
```

#### 3. 纯导航模式 (`nav_only`)
- 使用已有地图进行导航
- 启动AMCL定位
- 需要提供地图文件

```bash
ros2 launch megarover_navigation navigation.launch.py \
  mode:=nav_only \
  map:=/path/to/map.yaml
```

### 传感器模式

系统支持四种传感器配置：

| 模式 | 传感器 | 说明 |
|------|--------|------|
| `auto` | 自动检测 | 根据可用传感器自动选择（默认） |
| `fusion` | MID360 + D455 | 最高精度，推荐 |
| `lidar_only` | 仅MID360 | 大范围3D建图 |
| `rgbd_only` | 仅D455 | 室内环境 |

强制使用特定传感器模式：
```bash
ros2 launch megarover_navigation navigation.launch.py sensor_mode:=lidar_only
```

### Launch参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `slam_nav` | 运行模式 |
| `sensor_mode` | `auto` | 传感器模式 |
| `use_sim_time` | `false` | 使用仿真时间 |
| `rviz` | `true` | 启动RViz |
| `map` | `''` | 地图文件路径 |
| `namespace` | `''` | 机器人命名空间 |

## 节点说明

### sensor_detector
传感器检测节点，监控传感器状态并选择最佳配置。

**发布话题:**
- `/sensor_status` (std_msgs/String) - 传感器状态

**参数:**
- `check_interval` (float) - 检查间隔（秒）

### health_monitor
系统健康监控节点，监控节点状态并处理故障恢复。

**发布话题:**
- `/diagnostics` (diagnostic_msgs/DiagnosticArray) - 诊断信息
- `/rtabmap_health_status` (std_msgs/String) - 健康状态

**参数:**
- `mode` (string) - 当前运行模式
- `restart_on_failure` (bool) - 故障时是否重启

## 工具脚本

### 系统监控
```bash
cd src/megarover_navigation/scripts
./monitor_rtabmap.sh
```

### 系统诊断
```bash
./diagnose_rtabmap.sh
```

### 数据录制
```bash
./record_bag.sh --duration 300
```

## 导航使用

### 设置初始位姿
在RViz中使用"2D Pose Estimate"工具设置机器人初始位置。

### 发送导航目标
1. 在RViz中使用"Nav2 Goal"工具
2. 或通过命令行：
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}}"
```

### 取消导航
```bash
ros2 action send_goal /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

## 地图管理

### 保存地图
```bash
# 从RTABMAP数据库导出
rtabmap-databaseViewer ~/.ros/rtabmap.db

# 或使用map_saver
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 加载地图
```bash
ros2 launch megarover_navigation navigation.launch.py \
  mode:=nav_only \
  map:=/path/to/my_map.yaml
```

## 故障排查

### 传感器未检测到
```bash
# 检查USB权限
sudo usermod -a -G dialout $USER

# 检查设备
lsusb | grep Intel  # RealSense
ls /dev/ttyUSB*     # Livox
```

### TF树不完整
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo map base_link
```

### 导航失败
1. 检查地图质量
2. 调整代价地图参数
3. 检查里程计精度

## 配置优化

### 提高建图精度
编辑 `config/rtabmap_fusion.yaml`:
```yaml
Vis/MaxFeatures: "2000"  # 增加特征点
Grid/CellSize: "0.02"    # 提高分辨率
```

### 提高导航性能
编辑 `config/nav2_params.yaml`:
```yaml
controller_frequency: 30.0  # 提高控制频率
inflation_radius: 0.3       # 减小膨胀半径
```

## 高级功能

### 多机器人支持
```bash
# 机器人1
ros2 launch megarover_navigation navigation.launch.py namespace:=robot1

# 机器人2
ros2 launch megarover_navigation navigation.launch.py namespace:=robot2
```

### 自定义行为树
修改 `config/nav2_params.yaml` 中的 `bt_navigator` 部分。

## 许可证

Apache-2.0

## 维护者

Wang (wang@megarover.com)