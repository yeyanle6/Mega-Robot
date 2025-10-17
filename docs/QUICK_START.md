# 快速开始指南

本指南说明如何使用测试脚本快速启动和测试机器人系统。

---

## 准备工作

### 1. 确保系统已编译

```bash
cd ~/Code/Demo5
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 运行环境检查

```bash
./scripts/pre_test_check.sh
```

确保所有 17 项检查都通过 ✓

---

## 使用测试脚本

### 方式 1: 使用 Shell 脚本（推荐）

#### Step 1: 启动基础系统 (Bringup)

```bash
cd ~/Code/Demo5
./scripts/test_bringup.sh
```

**这个脚本会启动**：
- ✅ micro-ROS Agent (底盘通信)
- ✅ Livox MID360 驱动
- ✅ IMU 数据中继
- ✅ EKF 状态估计
- ✅ 静态 TF 发布

**检查清单**：
- `/livox/lidar` 有数据 (~10Hz)
- `/odometry/filtered` 有数据 (~50Hz)
- TF 树完整 (会生成 `frames.pdf`)

#### Step 2: 启动建图 (新终端)

```bash
cd ~/Code/Demo5
source install/setup.bash

# 方式 A: 创建新地图 (删除旧数据库)
DELETE_DB=true ./scripts/test_mapping.sh

# 方式 B: 继续已有地图 (默认)
./scripts/test_mapping.sh
```

**这个脚本会启动**：
- ✅ 点云预处理节点
- ✅ RTAB-Map SLAM 节点
- ✅ 自动检测节点和话题状态

#### Step 3: 控制机器人移动 (新终端)

```bash
cd ~/Code/Demo5
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**建图建议**：
- 🐢 慢速移动 (< 0.2 m/s)
- 🔄 定期返回起点触发回环检测
- 🏢 在特征丰富的环境中建图

#### Step 4: 监控建图进度 (可选，新终端)

```bash
cd ~/Code/Demo5
source install/setup.bash

# 实时监控 SLAM 性能
ros2 run t_robot_slam slam_monitor.py

# 或查看 RTAB-Map 信息
ros2 topic echo /rtabmap/info --once
```

#### Step 5: 导出地图

建图完成后，在新终端导出地图：

```bash
cd ~/Code/Demo5
source install/setup.bash

# 导出所有格式 (2D 地图 + 3D 点云 + 轨迹)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o ./maps/my_map \
  --all \
  --name "office_map_20251016"

# 或仅导出 2D 地图 (用于 Nav2)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o ./maps/2d_map \
  --2d
```

地图将保存在 `./maps/` 目录下。

---

### 方式 2: 手动启动（高级用户）

如果你想更精细地控制启动过程：

#### Terminal 1: Bringup

```bash
cd ~/Code/Demo5
source install/setup.bash
ros2 launch t_robot_bringup bringup.launch.py
```

#### Terminal 2: Mapping

```bash
cd ~/Code/Demo5
source install/setup.bash

# 新建地图
ros2 launch t_robot_slam mapping.launch.py \
  delete_db:=true

# 或继续已有地图
ros2 launch t_robot_slam mapping.launch.py \
  delete_db:=false
```

#### Terminal 3: 可视化 (可选)

```bash
cd ~/Code/Demo5
source install/setup.bash

# 启动 RViz
ros2 run rviz2 rviz2 -d src/t_robot_slam/rviz/mapping.rviz

# 或启动 rtabmapviz
ros2 launch t_robot_slam mapping.launch.py \
  use_viz:=true \
  launch_bringup:=false
```

---

## 常用命令速查

### 检查系统状态

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /livox/lidar
ros2 topic hz /odometry/filtered

# 查看 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf

# 检查 TF 变换
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

### 地图管理

```bash
# 列出所有地图
./src/t_robot_slam/scripts/map_manager.py list

# 设置活动地图
./src/t_robot_slam/scripts/map_manager.py set-active my_map

# 查看当前活动地图
./src/t_robot_slam/scripts/map_manager.py get-active

# 导出地图
./src/t_robot_slam/scripts/map_manager.py export my_map ./export_dir
```

### 清理和重启

```bash
# 清理 ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# 删除数据库 (重新建图)
rm ~/.ros/rtabmap.db

# 杀死所有 ROS 进程
killall -9 ros2
pkill -9 -f "ros2|rtabmap|micro_ros"
```

---

## 环境变量

测试脚本支持以下环境变量：

### test_bringup.sh

```bash
# micro-ROS Agent 设备
MICRO_ROS_AGENT_DEV=/dev/ttyUSB0 ./scripts/test_bringup.sh

# micro-ROS Agent 参数
MICRO_ROS_AGENT_ARGS="serial --dev /dev/ttyUSB1 -v4" ./scripts/test_bringup.sh

# TF 输出文件
TF_OUTPUT_FILE=my_frames.pdf ./scripts/test_bringup.sh
```

### test_mapping.sh

```bash
# 删除旧数据库，创建新地图
DELETE_DB=true ./scripts/test_mapping.sh

# 继续已有地图 (默认)
DELETE_DB=false ./scripts/test_mapping.sh
```

---

## 故障排查

### 问题 1: micro-ROS Agent 连接失败

**症状**：`/rover_odo` 或 `/odom` 话题没有数据

**解决**：
```bash
# 检查设备
ls /dev/ttyUSB*

# 确认 micro-ROS workspace
ls ~/uros_ws/install/setup.bash

# 手动启动 Agent
source ~/uros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
```

### 问题 2: MID360 没有数据

**症状**：`/livox/lidar` 话题没有数据

**解决**：
```bash
# 检查网络连接
ping -c 3 192.168.1.3

# 检查网卡配置
ip addr show eno1 | grep 192.168.1

# 重新配置网络
sudo ifconfig eno1 192.168.1.5 netmask 255.255.255.0
```

### 问题 3: RTAB-Map 报错 "Did not receive data"

**症状**：RTAB-Map 启动后持续报错

**解决**：
```bash
# 检查输入话题
ros2 topic hz /odometry/filtered
ros2 topic hz /mid360/points_filtered

# 确认配置中启用了近似同步
grep "approx_sync" src/t_robot_slam/params/rtabmap.yaml
# 应该是: approx_sync: true
```

### 问题 4: 编译错误

**解决**：
```bash
# 清理并重新编译
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 检查依赖
sudo apt update
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-robot-localization \
  ros-humble-pcl-ros
```

---

## 性能优化建议

### 对于 Jetson AGX Orin

如果发现性能不足，可以调整以下参数：

**1. 降低点云处理负载** (`src/t_robot_slam/launch/mapping.launch.py`):
```python
'voxel_grid.leaf_size': 0.1,  # 从 0.05 增加到 0.1
'range_filter.max_range': 20.0,  # 从 30.0 减少到 20.0
```

**2. 降低 RTAB-Map 处理频率** (`src/t_robot_slam/params/rtabmap.yaml`):
```yaml
Rtabmap/DetectionRate: "2.0"  # 每 2 秒处理一次
Icp/VoxelSize: "0.1"  # 增大体素大小
```

**3. 限制地图范围**:
```yaml
Grid/RangeMax: "15.0"  # 从 20m 减少到 15m
```

---

## 下一步

完成基础建图后，可以继续：

1. **导航测试**：参考 `docs/README_TEST.md`
2. **参数调优**：参考 `docs/MAPPING_GUIDE.md`
3. **多地图管理**：使用 `map_manager.py` 工具

---

**提示**：所有测试脚本都会在退出时自动清理进程，无需手动 killall。

**文档**：
- 详细建图指南: `docs/MAPPING_GUIDE.md`
- 项目主文档: `README.md`
- 测试手册: `docs/README_TEST.md`
