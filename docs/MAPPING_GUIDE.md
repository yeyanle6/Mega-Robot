# RTAB-Map 3D LiDAR SLAM 使用指南

本指南说明如何使用优化后的 RTAB-Map 配置进行 3D LiDAR SLAM 建图和导航。

## 目录
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [建图模式](#建图模式)
- [定位模式](#定位模式)
- [地图管理](#地图管理)
- [参数调优](#参数调优)
- [故障排除](#故障排除)

---

## 系统要求

### 硬件
- Jetson AGX Orin (或类似计算平台)
- Livox MID360 3D LiDAR
- Megarover3 移动底盘
- 内置 IMU (MID360)

### 软件
- ROS 2 Humble
- RTAB-Map (`ros-humble-rtabmap-ros`)
- PCL 1.12
- robot_localization (`ros-humble-robot-localization`)

---

## 快速开始

### 1. 启动机器人基础系统 (Bringup)

首先启动传感器驱动和状态估计：

```bash
# 终端 1: 启动 Bringup
cd ~/Code/Demo5
source install/setup.bash
ros2 launch t_robot_bringup bringup.launch.py
```

**检查清单**：
- ✓ `/livox/lidar` 话题有数据 (~10Hz)
- ✓ `/odometry/filtered` 话题有数据 (~50Hz)
- ✓ TF 树完整 (map → odom → base_link → sensors)

### 2. 启动建图

```bash
# 终端 2: 启动 RTAB-Map 建图
cd ~/Code/Demo5
source install/setup.bash

# 方式 1: 新建图 (删除旧数据库)
ros2 launch t_robot_slam mapping.launch.py delete_db:=true

# 方式 2: 继续已有地图
ros2 launch t_robot_slam mapping.launch.py delete_db:=false
```

### 3. 控制机器人移动

```bash
# 终端 3: 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**建图建议**：
- 缓慢移动 (< 0.2 m/s)
- 定期返回起点触发回环检测
- 确保视野中有足够的特征点 (墙壁、障碍物等)

---

## 建图模式

### Launch 文件参数

```bash
ros2 launch t_robot_slam mapping.launch.py \
  delete_db:=true \              # 删除已有数据库，创建新地图
  use_viz:=false \               # 是否启动 rtabmapviz 可视化
  database_path:=/path/to/db \   # 数据库路径 (默认: ~/.ros/rtabmap.db)
  launch_bringup:=false          # 是否自动启动 bringup
```

### 监控建图进度

#### 方法 1: 查看 RTAB-Map 信息

```bash
# 查看统计信息
ros2 topic echo /rtabmap/info --once

# 监控内存和性能
ros2 run t_robot_slam slam_monitor.py
```

#### 方法 2: 使用 RViz 可视化

```bash
ros2 run rviz2 rviz2 -d src/t_robot_slam/rviz/mapping.rviz
```

在 RViz 中可以看到：
- `/rtabmap/mapData`: 全局地图
- `/rtabmap/mapGraph`: 位姿图
- `/mid360/points_filtered`: 过滤后的点云
- `/odometry/filtered`: EKF 融合的里程计

### 保存地图

#### 自动保存

RTAB-Map 会自动将地图保存到数据库文件 (`~/.ros/rtabmap.db`)。
退出时无需手动保存。

#### 导出地图

使用提供的导出脚本：

```bash
# 导出所有格式 (2D 地图 + 3D 点云 + 轨迹)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o ./maps/my_map \
  --all \
  --name my_office_map

# 仅导出 2D 占用栅格地图 (用于 Nav2)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o ./maps/2d_map \
  --2d \
  --resolution 0.05

# 仅导出 3D 点云
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o ./maps/cloud.pcd \
  --3d \
  --voxel 0.01
```

---

## 定位模式

使用已有地图进行定位（AMCL 替代方案）：

```bash
# 启动定位模式
ros2 launch t_robot_slam mapping.launch.py \
  localization:=true \
  delete_db:=false \
  database_path:=/path/to/existing/map.db
```

**定位模式特点**：
- `Mem/IncrementalMemory: false` - 不添加新节点
- `Mem/InitWMWithAllNodes: true` - 加载所有已有节点
- 仅进行位姿估计，不更新地图

---

## 地图管理

### 使用地图管理工具

```bash
# 查看所有地图
./src/t_robot_slam/scripts/map_manager.py list

# 添加新地图到管理系统
./src/t_robot_slam/scripts/map_manager.py add \
  ./maps/cloud.pcd \
  --name office_map \
  --description "办公室一层地图"

# 设置活动地图
./src/t_robot_slam/scripts/map_manager.py set-active office_map

# 导出地图
./src/t_robot_slam/scripts/map_manager.py export office_map ./export_dir

# 删除地图 (会自动备份)
./src/t_robot_slam/scripts/map_manager.py delete office_map
```

### 地图目录结构

```
maps/
├── pcd/              # 3D 点云文件
│   └── map_name.pcd
├── pgm/              # 2D 占用栅格地图
│   └── map_name/
│       ├── map.pgm
│       └── map.yaml
├── backup/           # 删除的地图备份
└── maps_metadata.json  # 地图元数据
```

---

## 参数调优

### 关键参数说明

#### ICP 配置 (`src/t_robot_slam/params/rtabmap.yaml`)

```yaml
# 点云配准策略
Reg/Strategy: "1"                    # 1=ICP (纯 LiDAR)
Icp/PointToPlane: "true"             # Point-to-plane ICP
Icp/VoxelSize: "0.05"                # 下采样体素大小
Icp/MaxCorrespondenceDistance: "0.5" # 最大对应点距离
Icp/CorrespondenceRatio: "0.1"       # 最小匹配点比例
```

**调优建议**：
- `Icp/VoxelSize` ↓ → 精度 ↑，速度 ↓
- `Icp/MaxCorrespondenceDistance` ↑ → 鲁棒性 ↑，精度 ↓
- `Icp/CorrespondenceRatio` ↓ → 更宽松的匹配

#### 回环检测

```yaml
RGBD/ProximityBySpace: "true"        # 基于空间距离检测回环
RGBD/LinearUpdate: "0.1"             # 平移 10cm 更新
RGBD/AngularUpdate: "0.1"            # 旋转 ~6° 更新
Rtabmap/LoopThr: "0.11"              # 回环相似度阈值
```

**调优建议**：
- 如果回环检测过于频繁 → 降低 `LoopThr` (更严格)
- 如果检测不到回环 → 增加 `LoopThr`，减小更新阈值

#### 地图生成

```yaml
Grid/CellSize: "0.05"                # 栅格分辨率 5cm
Grid/RangeMax: "20.0"                # 最大建图距离 20m
Grid/MaxObstacleHeight: "2.0"        # 障碍物最大高度
Grid/MinGroundHeight: "-0.5"         # 地面最小高度
Grid/MaxGroundHeight: "0.05"         # 地面最大高度
```

#### 点云预处理 (`src/t_robot_slam/launch/mapping.launch.py`)

```yaml
voxel_grid.leaf_size: 0.05           # 体素下采样
range_filter.min_range: 0.5          # 最小距离
range_filter.max_range: 30.0         # 最大距离
outlier_removal.mean_k: 50           # 离群点过滤
```

---

## 故障排除

### 问题 1: RTAB-Map 报错 "Did not receive data"

**原因**：话题时间戳不同步

**解决方案**：
```bash
# 检查输入话题频率
ros2 topic hz /odometry/filtered
ros2 topic hz /mid360/points_filtered

# 确认配置中启用了近似同步
# rtabmap.yaml: approx_sync: true
```

### 问题 2: 建图漂移严重

**原因**：
- 点云配准失败
- 缺少特征丰富的环境
- 移动速度过快

**解决方案**：
- 降低移动速度 (< 0.2 m/s)
- 增加 `Icp/MaxCorrespondenceDistance`
- 在特征丰富的环境中建图

### 问题 3: 回环检测失败

**原因**：
- 阈值设置过严
- 没有返回起点
- 视角差异过大

**解决方案**：
```yaml
# 放宽回环检测阈值
Rtabmap/LoopThr: "0.15"  # 默认 0.11，增加到 0.15
RGBD/ProximityAngle: "60"  # 默认 45°，增加到 60°
```

### 问题 4: 内存占用过高

**解决方案**：
```yaml
# 减少内存占用
Mem/STMSize: "20"                    # 减小短期记忆 (默认 30)
Grid/RangeMax: "15.0"                # 减小建图范围 (默认 20m)
Icp/VoxelSize: "0.1"                 # 增大体素大小 (默认 0.05)
```

### 问题 5: TF 查找超时

**原因**：TF 树不完整或时间戳问题

**解决方案**：
```bash
# 检查 TF 树
timeout 5 ros2 run tf2_tools view_frames
evince frames.pdf

# 检查关键 TF
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

### 问题 6: 点云预处理节点崩溃

**原因**：点数过多或参数配置不当

**解决方案**：
```yaml
# 增加下采样力度
voxel_grid.leaf_size: 0.1  # 从 0.05 增加到 0.1
range_filter.max_range: 20.0  # 从 30.0 减少到 20.0
```

---

## 性能基准

### Jetson AGX Orin 性能指标

| 指标 | 典型值 |
|------|--------|
| 点云预处理延迟 | ~50ms |
| RTAB-Map 处理延迟 | ~40-50ms |
| 关键帧创建频率 | ~1Hz |
| 内存占用 (1分钟建图) | ~10-50 MB |
| CPU 占用 | ~30-40% |

### 优化建议

**高精度建图**：
```yaml
Icp/VoxelSize: "0.02"
Grid/CellSize: "0.025"
Icp/Iterations: "50"
```

**高速度建图**：
```yaml
Icp/VoxelSize: "0.1"
Grid/CellSize: "0.1"
Icp/Iterations: "20"
Rtabmap/DetectionRate: "2.0"  # 降低处理频率
```

---

## 参考资源

- [RTAB-Map 官方文档](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map ROS 2 教程](https://github.com/introlab/rtabmap_ros/tree/ros2)
- [Livox MID360 技术规格](https://www.livoxtech.com/mid-360)
- [项目主 README](../README.md)

---

**最后更新**: 2025-10-16
**维护者**: yeyanle6
