# T-Robot SLAM 测试脚本使用指南

本目录包含用于测试T-Robot SLAM系统的各种脚本。

---

## 脚本概览

### 1. `test_full_mapping.sh` ⭐ (推荐)
**一键启动完整建图流程**

集成了基础系统、建图、可视化和导出功能的完整脚本。

**基本用法**:
```bash
./scripts/test_full_mapping.sh
```

**特点**:
- ✅ 自动启动所有必需组件 (micro-ROS Agent, Bringup, Mapping, RViz)
- ✅ 彩色输出，清晰的进度提示
- ✅ 实时数据流验证
- ✅ 可选的自动地图导出
- ✅ 完善的错误处理和资源清理
- ✅ 详细的操作指南

**环境变量**:
```bash
# 创建新地图（删除旧数据库）
DELETE_DB=true ./scripts/test_full_mapping.sh

# 不启动 RViz（节省资源）
LAUNCH_RVIZ=false ./scripts/test_full_mapping.sh

# 自动导出地图（结束时）
AUTO_EXPORT=true ./scripts/test_full_mapping.sh

# 导出时使用的体素大小
EXPORT_VOXEL=0.02 AUTO_EXPORT=true ./scripts/test_full_mapping.sh

# 指定 micro-ROS Agent 设备
MICRO_ROS_AGENT_DEV=/dev/ttyUSB1 ./scripts/test_full_mapping.sh

# 组合使用
DELETE_DB=true LAUNCH_RVIZ=true AUTO_EXPORT=true ./scripts/test_full_mapping.sh
```

**适用场景**:
- 首次测试系统
- 完整的建图任务
- 需要可视化观察建图过程
- 演示系统功能

---

### 2. `test_bringup.sh`
**基础系统启动测试**

只启动基础硬件驱动和状态估计，不启动SLAM。

**基本用法**:
```bash
./scripts/test_bringup.sh
```

**启动组件**:
- micro-ROS Agent (底盘通讯)
- Livox MID360 驱动
- IMU 数据中继
- EKF 状态估计
- 静态 TF 发布器

**测试内容**:
1. 节点状态检查
2. 话题数据验证
3. 频率测试
4. TF 树生成

**适用场景**:
- 测试硬件连接
- 调试传感器驱动
- 验证TF配置
- 作为其他测试的基础

---

### 3. `test_mapping.sh`
**建图系统启动测试**

在已运行的bringup基础上，启动SLAM建图系统。

**基本用法**:
```bash
# 先启动 bringup (终端1)
./scripts/test_bringup.sh

# 再启动 mapping (终端2)
./scripts/test_mapping.sh

# 或者新建地图
DELETE_DB=true ./scripts/test_mapping.sh
```

**启动组件**:
- 点云预处理器
- RTAB-Map SLAM

**测试内容**:
1. 节点依赖检查
2. 点云处理链验证
3. 性能监控

**适用场景**:
- 需要手动控制bringup和mapping分离启动
- 调试建图参数
- 性能测试

---

## 典型工作流程

### 场景1: 首次使用 / 完整测试
```bash
# 一键启动所有组件（推荐）
./scripts/test_full_mapping.sh

# 在新终端控制机器人移动
cd ~/Code/Demo5
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 建图5-10分钟后按 Ctrl+C 停止
```

### 场景2: 测试硬件连接
```bash
# 只启动基础系统
./scripts/test_bringup.sh

# 观察传感器数据是否正常
# 按 Ctrl+C 停止
```

### 场景3: 分步启动（高级用户）
```bash
# 终端1: 启动基础系统
./scripts/test_bringup.sh

# 终端2: 启动建图
DELETE_DB=true ./scripts/test_mapping.sh

# 终端3: 启动RViz
source install/setup.bash
ros2 run rviz2 rviz2 -d src/t_robot_slam/rviz/mapping.rviz

# 终端4: 控制机器人
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 场景4: 自动导出地图
```bash
# 启动时设置自动导出
DELETE_DB=true AUTO_EXPORT=true EXPORT_VOXEL=0.05 ./scripts/test_full_mapping.sh

# 建图后按 Ctrl+C，脚本会自动导出地图到 maps/YYYYMMDD_HHMMSS/
```

---

## 常用命令

### 控制机器人移动
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘控制:
- `i`: 前进
- `k`: 停止
- `j`: 左转
- `l`: 右转
- `u/o`: 左前/右前
- `m/,`: 左后/右后
- `q/z`: 增加/减少速度

### 监控RTAB-Map状态
```bash
# 查看一次状态
ros2 topic echo /rtabmap/info --once

# 持续监控
ros2 topic echo /rtabmap/info

# 查看回环检测
tail -f /tmp/full_mapping_mapping.log | grep -i loop
```

### 查看话题频率
```bash
# 原始 LiDAR 数据
ros2 topic hz /mid360/lidar

# 过滤后点云
ros2 topic hz /mid360/points_filtered

# 里程计
ros2 topic hz /odometry/filtered

# 地图更新
ros2 topic hz /map
```

### 导出地图
```bash
# 导出3D点云 (PCD)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/cloud.pcd \
  --3d --voxel 0.05

# 导出轨迹
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/trajectory.txt \
  --poses

# 保存2D地图 (需要RTAB-Map运行中)
ros2 run nav2_map_server map_saver_cli -f maps/2d_map
```

---

## 日志文件位置

所有脚本生成的日志保存在`/tmp/`目录：

| 脚本 | 日志文件 |
|------|---------|
| `test_full_mapping.sh` | `/tmp/full_mapping_*.log` |
| `test_bringup.sh` | `/tmp/bringup_test.log` |
| `test_mapping.sh` | `/tmp/mapping_test.log` |
| micro-ROS Agent | `/tmp/micro_ros_agent.log` |

**查看日志**:
```bash
# 实时查看建图日志
tail -f /tmp/full_mapping_mapping.log

# 查看错误信息
grep -i error /tmp/full_mapping_*.log
```

---

## 故障排查

### 问题1: micro-ROS Agent 连接失败

**症状**: 没有 `/odom` 或 `/rover_odo` 话题

**检查**:
```bash
# 检查USB设备
ls /dev/ttyUSB*

# 查看日志
cat /tmp/micro_ros_agent.log
```

**解决**:
- 检查USB线连接
- 确认设备路径（可能是 `/dev/ttyUSB1`）
- 设置正确的设备: `MICRO_ROS_AGENT_DEV=/dev/ttyUSB1 ./scripts/test_full_mapping.sh`

### 问题2: MID360 无数据

**症状**: 没有 `/mid360/lidar` 话题数据

**检查**:
```bash
# 检查网络连接
ping -c 3 192.168.1.3

# 查看网卡IP
ip addr show eno1
```

**解决**:
```bash
# 重新配置网络
sudo ifconfig eno1 192.168.1.5 netmask 255.255.255.0
```

### 问题3: RTAB-Map 报错

**症状**: "Did not receive data since X seconds"

**检查**:
```bash
# 检查输入话题
ros2 topic hz /odometry/filtered
ros2 topic hz /mid360/points_filtered
```

**解决**:
- 确认 `test_bringup.sh` 已启动
- 检查点云预处理器是否运行: `ros2 node list | grep pointcloud`

### 问题4: RViz 显示问题

**症状**: RViz 无法显示点云或地图

**解决**:
- 检查 Fixed Frame 是否设置为 `map`
- 确认话题名称正确
- 等待机器人移动后再观察（2D地图需要移动后才会出现）

### 问题5: 地面识别失败

**症状**: 地面点云（绿色）很少或没有

**检查**:
```bash
# 查看地面点云
ros2 topic echo /cloud/ground --once

# 查看障碍物点云
ros2 topic echo /cloud/obstacles --once
```

**解决**:
- 参考 `docs/TILT_OPTIMIZATION_2025-10-16.md`
- 检查LiDAR倾角是否正确（应该是30°）
- 调整RANSAC参数

---

## 性能优化

### 降低CPU占用

如果发现CPU占用过高：

```bash
# 不启动RViz
LAUNCH_RVIZ=false ./scripts/test_full_mapping.sh

# 或修改 mapping.launch.py 中的参数
'voxel_grid.leaf_size': 0.1,  # 从 0.05 增加
'range_filter.max_range': 10.0,  # 从 15.0 减少
```

### 提高建图质量

如果需要更高质量的地图：

```bash
# 修改 rtabmap.yaml
Rtabmap/DetectionRate: "0.5"  # 从 1.0 降低（更慢，更精确）
Icp/VoxelSize: "0.02"  # 从 0.05 减小（更精细）
```

---

## 相关文档

- 完整建图指南: `docs/MAPPING_GUIDE.md`
- 30°倾角优化: `docs/TILT_OPTIMIZATION_2025-10-16.md`
- 地图导出指南: `docs/MAP_EXPORT_GUIDE.md`
- 快速开始: `docs/QUICK_START.md`
- 测试结果: `docs/TEST_RESULTS_2025-10-16.md`

---

## 贡献和反馈

如发现脚本问题或有改进建议，请提交Issue。

**最后更新**: 2025-10-16
