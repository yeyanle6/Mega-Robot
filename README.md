# Mega-Robot

**基于 Jetson AGX Orin 的室内自主移动机器人系统**

一个集成 Megarover3 底盘与 Livox MID360 LiDAR 的完整 ROS 2 机器人平台，支持 3D SLAM 建图和自主导航。

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Jetson_AGX_Orin-76B900.svg)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

---

## 📋 项目概述

### 核心特性
- **3D LiDAR SLAM**: 使用 RTAB-Map 进行纯 LiDAR 建图，支持回环检测与地图优化
- **多传感器融合**: EKF 融合轮速计与 IMU 数据，提供鲁棒的状态估计
- **自主导航**: 基于 Nav2 的路径规划与动态避障
- **点云预处理**: 高性能 C++ 节点，实现地面分割、体素下采样和 TF 变换
- **模块化设计**: 分层 Launch 文件，支持灵活的系统配置

### 硬件平台
| 组件 | 型号 | 功能 |
|------|------|------|
| 计算平台 | NVIDIA Jetson AGX Orin | 主控单元 (JetPack 6.2) |
| 移动底盘 | Megarover3 | 差速驱动，提供轮速计 |
| 3D LiDAR | Livox MID360 | 360° 视场，最大 40m 测距 |
| IMU | MID360 内置 IMU | 6 轴姿态数据 |

### 软件架构
```
ROS 2 Humble
├── RTAB-Map        # 3D SLAM 建图
├── Nav2            # 导航栈
├── robot_localization  # EKF 状态估计
└── PCL 1.12        # 点云处理
```

---

## 🚀 快速开始

### 系统要求
- **操作系统**: Ubuntu 22.04 LTS (Jetson AGX Orin)
- **ROS 2 版本**: Humble Hawksbill
- **依赖包**: 详见 [安装指南](#-安装)

### 一键启动测试

```bash
cd ~/Code/Demo5

# 1. 环境自检（确保 17 项全部 PASS）
./pre_test_check.sh

# 2. 启动 Bringup（底盘 + 传感器 + EKF）
./test_bringup.sh

# 3. 启动建图（新终端）
./test_mapping.sh

# 4. 启动导航（新终端，需先完成建图）
./test_navigation.sh
```

> **提示**: 每个脚本都会自动检查依赖并在退出时清理环境，无需手动 `killall`。

---

## 📦 安装

### 1. 安装 ROS 2 Humble

参考 [ROS 2 官方文档](https://docs.ros.org/en/humble/Installation.html) 完成基础安装。

### 2. 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-robot-localization \
  ros-humble-pcl-ros \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-nav2-bringup \
  ros-humble-nav2-behavior-tree \
  ros-humble-nav2-rviz-plugins \
  ros-humble-diagnostic-updater \
  ros-humble-rqt-robot-monitor \
  ros-humble-twist-mux
```

### 3. 克隆仓库并编译

```bash
cd ~/Code
git clone https://github.com/yeyanle6/Mega-Robot.git Demo5
cd Demo5

# 初始化子模块（如果包含）
git submodule update --init --recursive

# 编译项目
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. 配置 micro-ROS Agent（底盘通信）

```bash
# 安装 micro-ROS workspace（参考 Megarover3 文档）
cd ~/uros_ws
colcon build
source install/setup.bash

# 验证串口连接
ls /dev/ttyUSB*  # 应显示 /dev/ttyUSB0
```

---

## 🛠️ 使用指南

### Bringup（启动机器人基础系统）

```bash
# 方式 1：使用测试脚本（推荐）
./test_bringup.sh

# 方式 2：手动启动
source install/setup.bash
ros2 launch t_robot_bringup bringup.launch.py
```

**检查清单**:
- [ ] micro-ROS Agent 运行中
- [ ] `/rover_odo` 话题发布（轮速计数据）
- [ ] `/mid360/lidar` 话题发布（点云数据，~10Hz）
- [ ] `/odometry/filtered` 话题发布（EKF 融合输出，~50Hz）
- [ ] TF 树完整：`map → odom → base_footprint → base_link → mid360_lidar`

### 建图（RTAB-Map SLAM）

```bash
# 启动建图
./test_mapping.sh

# 或手动启动
ros2 launch t_robot_slam mapping.launch.py
```

**建图流程**:
1. 使用键盘控制移动机器人：
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
2. 缓慢移动（建议 < 0.2 m/s）以获得最佳建图效果
3. 返回起点以触发回环检测

**监控建图进度**:
```bash
# 查看建图信息
ros2 topic echo /rtabmap/info --once

# 实时监控性能
ros2 run t_robot_slam slam_monitor.py

# RViz 可视化
ros2 run rviz2 rviz2 -d src/t_robot_slam/rviz/mapping.rviz
```

### 导航（Nav2）

```bash
# 启动导航
./test_navigation.sh

# 或手动启动
ros2 launch t_robot_slam navigation.launch.py map:=/tmp/rtabmap_export/map.yaml
```

**导航步骤**:
1. 在 RViz 中使用 **2D Pose Estimate** 设置初始位姿
2. 使用 **2D Goal Pose** 设置目标点
3. 或通过命令行发送导航目标：
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     '{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}}'
   ```

---

## 📁 项目结构

```
Demo5/
├── src/                          # ROS 2 软件包源代码
│   ├── t_robot_bringup/         # 机器人启动配置 [自研]
│   │   ├── launch/              # 分层 Launch 文件 (base/sensors/state_estimation)
│   │   ├── params/              # EKF 和传感器驱动参数
│   │   ├── config/              # 静态 TF 和滤波器配置
│   │   └── scripts/             # IMU QoS 转换、时间同步监控
│   ├── t_robot_slam/            # SLAM 与导航 [自研]
│   │   ├── src/                 # C++ 点云预处理节点
│   │   ├── launch/              # 建图/定位/导航 Launch
│   │   ├── params/              # RTAB-Map 参数（60+ 调优参数）
│   │   ├── scripts/             # 地图管理、性能监控脚本
│   │   └── rviz/                # RViz 可视化配置
│   ├── megarover3_ros2/         # Megarover3 底盘驱动 [第三方]
│   ├── livox_ros_driver2/       # Livox MID360 驱动 [第三方]
│   ├── realsense-ros/           # Intel RealSense 驱动 [第三方，可选]
│   └── vs_rover_options_description/  # 配件 URDF [第三方]
├── docs/                         # 项目文档
│   ├── project_plan.md          # 详细开发计划与里程碑
│   ├── README_TEST.md           # 完整测试手册
│   ├── AGENTS.md                # 开发者规范
│   └── DIRECTORY_STRUCTURE.md   # 目录结构详解
├── scripts/                      # 自动化测试脚本
│   ├── pre_test_check.sh        # 环境自检（17 项检查）
│   ├── test_bringup.sh          # Bringup 测试
│   ├── test_mapping.sh          # 建图测试
│   └── test_navigation.sh       # 导航测试
├── build/                        # 编译输出 [.gitignore]
├── install/                      # 安装目录 [.gitignore]
├── log/                          # ROS 日志 [.gitignore]
└── README.md                     # 本文件
```

> **详细说明**: 参见 [docs/DIRECTORY_STRUCTURE.md](docs/DIRECTORY_STRUCTURE.md)

---

## 🔧 配置说明

### 网络配置（MID360 LiDAR）

Livox MID360 通过以太网连接，需确保与主机在同一网段：

| 设备 | IP 地址 | 说明 |
|------|---------|------|
| MID360 | `192.168.1.3` | 固定 IP（出厂默认） |
| Jetson eno1 | `192.168.1.5/24` | 有线网卡 |

**验证连接**:
```bash
ping -c 3 192.168.1.3
```

配置文件: `src/t_robot_bringup/params/livox_driver.yaml`

### TF 树架构

```
map
 └─ odom (pub_odom 发布)
     └─ base_footprint (EKF 估计位姿)
         └─ base_link
             ├─ mid360_base
             │   └─ mid360_lidar (点云坐标系)
             ├─ mid360_imu
             └─ ... (其他传感器)
```

**关键节点**:
- `pub_odom`: 发布 `odom → base_footprint` TF + `/odom` 消息 (100Hz)
- `ekf_filter_node`: 融合轮速计 + IMU，输出 `/odometry/filtered` (50Hz)，**不发布 TF**
- `robot_state_publisher`: 发布静态 TF（传感器到 base_link）

### 参数调优

| 参数文件 | 用途 | 关键参数 |
|---------|------|----------|
| `params/ekf.yaml` | EKF 融合配置 | `odom0_config`, `imu0_config`, `publish_tf: false` |
| `params/rtabmap.yaml` | RTAB-Map SLAM | `Reg/Strategy: 1`, `Icp/VoxelSize: 0.05` |
| `config/navigation/nav2_params.yaml` | Nav2 导航 | 局部/全局规划器、Costmap 层 |
| `config/preprocessing_params.yaml` | 点云预处理 | 体素大小、地面分割阈值 |

---

## 📊 性能指标

### 点云预处理性能
- **输入**: ~20,000 点/帧 (MID360 原始数据)
- **输出**: 障碍物 ~5,200 点 + 地面 ~1,900 点
- **延迟**: ~50ms/帧
- **频率**: ~20Hz

### RTAB-Map 建图性能（Jetson AGX Orin）
- **关键帧创建**: 1Hz
- **处理延迟**: ~40-50ms
- **内存占用**: ~10MB (68 秒静止测试，68 节点)
- **地图分辨率**: 5cm 栅格地图

### 状态估计
- **EKF 输出频率**: 50Hz
- **轮速计频率**: 100Hz
- **IMU 频率**: ~100Hz (MID360 内置)

---

## 🐛 故障排查

### 常见问题

**问题 1: MID360 无数据**
```bash
# 检查网络连接
ping 192.168.1.3
ip addr show eno1 | grep 192.168.1

# 检查驱动节点
ros2 node list | grep livox
ros2 topic hz /mid360/lidar
```

**问题 2: EKF 无输出**
```bash
# 确认 micro-ROS Agent 运行
ps aux | grep micro_ros_agent

# 检查轮速计数据
ros2 topic hz /rover_odo
ros2 topic echo /odom --once
```

**问题 3: TF 树断裂**
```bash
# 生成 TF 树可视化
timeout 10 ros2 run tf2_tools view_frames
evince frames.pdf

# 检查关键 TF
ros2 run tf2_ros tf2_echo odom base_footprint
```

**问题 4: RTAB-Map "Did not receive data"**
- 检查话题时间戳同步: 确保 `approx_sync: true` (已配置)
- 验证输入话题: `/odometry/filtered` 和 `/mid360/points_filtered` 都有数据

> **更多排查方法**: 参见 [docs/README_TEST.md § 故障排查](docs/README_TEST.md#5-故障排查与修复)

---

## 📚 文档

| 文档 | 描述 |
|------|------|
| [README_TEST.md](docs/README_TEST.md) | 完整的测试手册，包含硬件配置、网络设置、实测流程 |
| [project_plan.md](docs/project_plan.md) | 项目规划、阶段目标、里程碑与待办事项 |
| [AGENTS.md](docs/AGENTS.md) | 开发者规范、构建命令、代码风格 |
| [DIRECTORY_STRUCTURE.md](docs/DIRECTORY_STRUCTURE.md) | 详细的目录结构与文件说明 |

---

## 🗺️ 路线图

### 当前状态（v0.9）
- ✅ 硬件驱动与 Bringup 完成
- ✅ EKF 多传感器融合
- ✅ 点云预处理管线
- ✅ RTAB-Map 3D LiDAR SLAM 配置
- ✅ 静止状态系统稳定性验证
- 🔄 **进行中**: 移动建图与回环检测验证

### 下一步 (v1.0)
- [ ] 实车移动建图测试
- [ ] 地图保存与加载流程
- [ ] Nav2 多点导航验证
- [ ] 动态避障性能优化
- [ ] 完整文档与教程

### 未来计划
- [ ] RealSense 深度相机集成（可选）
- [ ] Gazebo 仿真环境
- [ ] 多地图管理工具
- [ ] Web 远程控制界面

---

## 🤝 贡献

欢迎贡献代码、报告问题或提出改进建议！

### 开发流程
1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'feat: Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 提交 Pull Request

### 代码规范
- 遵循 [AGENTS.md](docs/AGENTS.md) 中的编码规范
- 使用 Conventional Commits 格式
- 运行 `colcon test` 确保通过所有检查

---

## 📄 许可证

本项目采用 [Apache License 2.0](LICENSE) 许可证。

第三方依赖的许可证请参见各子模块：
- `megarover3_ros2`: BSD-3-Clause
- `livox_ros_driver2`: BSD
- `realsense-ros`: Apache 2.0

---

## 🙏 致谢

- [RTAB-Map](https://github.com/introlab/rtabmap_ros) - 强大的 3D SLAM 框架
- [Nav2](https://github.com/ros-planning/navigation2) - ROS 2 导航栈
- [Livox SDK](https://github.com/Livox-SDK/livox_ros_driver2) - MID360 驱动支持
- [VerteXobotics](https://github.com/vstoneofficial/megarover_ros2) - Megarover3 ROS 2 驱动

---

## 📬 联系方式

- **作者**: yeyanle6
- **GitHub**: [https://github.com/yeyanle6/Mega-Robot](https://github.com/yeyanle6/Mega-Robot)
- **问题反馈**: [GitHub Issues](https://github.com/yeyanle6/Mega-Robot/issues)

---

<p align="center">
  <i>基于 NVIDIA Jetson 平台，赋能下一代移动机器人 🤖</i>
</p>
