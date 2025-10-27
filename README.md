# Mega-Robot 自主导航系统

**ROS2 Humble | RTAB-Map SLAM | Nav2导航 | 多传感器融合**

---

## 快速开始

### 1. 编译系统
```bash
cd /home/wang/Code/Demo6
colcon build
source install/setup.bash
```

### 2. 启动SLAM建图
```bash
# 交互式启动(推荐) - 自动检测传感器并选择模式
./start_navigation.sh

# 或手动指定模式
ros2 launch megarover_navigation modular_rtabmap.launch.py \
    force_mode:=fusion \
    rviz:=true
```

### 3. 启动导航
```bash
ros2 launch megarover_navigation navigation.launch.py mode:=slam_nav
```

---

## 硬件配置

| 组件 | 型号 | 说明 |
|------|------|------|
| 底盘 | MegaRover Ver.3.0 | micro-ROS, 控制话题`/rover_twist` |
| 激光雷达 | Livox Mid-360 | 非重复扫描, PointCloud2输出 |
| 深度相机 | Intel RealSense D455 | RGB-D, 需USB 3.0 |

---

## 传感器配置

支持三种传感器配置（自动检测）：

| 模式 | 传感器 | 精度 | 场景 |
|------|--------|------|------|
| **Fusion** (推荐) | Mid-360 + D455 | ⭐⭐⭐⭐⭐ | 复杂室内外环境 |
| LiDAR Only | 仅Mid-360 | ⭐⭐⭐⭐ | 开阔空间、长走廊 |
| RGB-D Only | 仅D455 | ⭐⭐⭐ | 特征丰富的室内 |

---

## 文档导航

### 核心文档
- **[project_overview.md](project_overview.md)** - 项目概要(硬件配置、系统架构、技术栈)
- **[project_progress.md](project_progress.md)** - 当前进度(实时更新)
- **[project_issues.md](project_issues.md)** - 问题记录(已解决问题和FAQ)

### 详细文档
- **[USAGE_GUIDE.md](USAGE_GUIDE.md)** - 完整使用指南
- **[TESTING_GUIDE.md](TESTING_GUIDE.md)** - 测试指南

---

## 系统架构

```
[硬件层]
├── MegaRover3底盘 (pub_odom)
├── Livox Mid-360 (livox_ros_driver2)
└── RealSense D455 (realsense2_camera)

[SLAM层]
├── icp_odometry (激光里程计)
├── rtabmap (SLAM核心)
└── rtabmap_viz (可视化)

[导航层]
├── planner_server (路径规划)
├── controller_server (路径跟踪)
└── bt_navigator (行为树)
```

---

## 常用命令

### 查看系统状态
```bash
# 查看节点
ros2 node list

# 查看话题
ros2 topic list

# 查看TF树
ros2 run tf2_tools view_frames

# 查看传感器频率
ros2 topic hz /livox/lidar
ros2 topic hz /camera/color/image_raw
```

### 验证系统
```bash
# 验证所有修复
./verify_all_fixes.sh

# 基础功能测试
./test_basic_functions.sh
```

---

## 故障排查

### 传感器连接问题
```bash
# Livox权限
sudo usermod -a -G dialout $USER

# RealSense检测
lsusb | grep Intel
rs-enumerate-devices
```

### TF错误
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查TF变换
ros2 run tf2_ros tf2_echo map base_link
```

详见 **[project_issues.md](project_issues.md)** 的FAQ部分。

---

## 🎉 重构完成！(2025-10-22)

✅ **基于官方例程完成架构级重构**，创建全新的LiDAR SLAM系统！

**已解决的核心问题**:
- ✅ 添加Lidar Deskewing (运动畸变消除)
- ✅ 修正frame_id设计 (使用`mid360_lidar`)
- ✅ 优化ICP参数 (`MaxCorrespondenceDistance: 1.0`)
- ✅ 实现IMU stabilized frame

**新Launch文件**: `rtabmap_lidar_slam.launch.py` - 完整的去畸变流程

---

## 开发进度

**总体进度**: 🟢 70% █████████████████████░░░░░░░░░ 100%

| 阶段 | 状态 | 备注 |
|------|------|------|
| 硬件驱动验证 | ✅ 完成 | 驱动已集成 |
| 传感器配置 | ✅ 完成 | 重构完成，遵循官方最佳实践 |
| SLAM建图 | 🟢 代码就绪 | 待硬件测试 |
| 定位验证 | ⏳ 待测试 | 需硬件连接 |
| 导航集成 | 🟢 代码就绪 | 待硬件测试 |
| 性能优化 | ⏳ 未开始 | 需实测数据 |

详见 **[project_progress.md](project_progress.md)**

---

## ROS2包结构

```
src/
├── megarover_navigation/      # 主导航包(自研)
│   ├── config/                # 配置文件
│   ├── launch/                # Launch文件
│   └── megarover_navigation/  # Python模块
├── megarover3_ros2/           # 底盘驱动(官方)
├── livox_ros_driver2/         # Livox驱动(官方)
├── realsense-ros/             # RealSense驱动(官方)
├── rtabmap/                   # RTAB-Map核心(官方)
└── rtabmap_ros/               # RTAB-Map ROS2(官方)
```

---

## 关键约束

⚠️ **MegaRover3底盘**:
- 速度控制话题为`/rover_twist` (非标准`/cmd_vel`)
- Base frame必须是`base_footprint`

⚠️ **Livox Mid-360**:
- 非重复扫描，输出PointCloud2
- 需USB权限: `sudo usermod -a -G dialout $USER`

⚠️ **RealSense D455**:
- 必须USB 3.0连接
- 必须启用`align_depth`

---

## 技术支持

- **问题反馈**: 查看或更新 [project_issues.md](project_issues.md)
- **进度跟踪**: 查看 [project_progress.md](project_progress.md)
- **系统详情**: 查看 [project_overview.md](project_overview.md)

---

## 许可证

Apache-2.0

---

**维护者**: Wang
**创建日期**: 2025-10-22
**ROS2版本**: Humble (Ubuntu 22.04)
**工作空间**: `/home/wang/Code/Demo6`
