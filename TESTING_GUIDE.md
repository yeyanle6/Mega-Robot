# MegaRover3 导航系统完整测试指南

**版本**: 1.0
**日期**: 2025-10-22
**目的**: 提供系统化的测试流程，确保所有功能正常工作

---

## 📋 测试概览

导航系统测试分为**4个阶段**，必须**按顺序**进行：

```
阶段1: 基础功能测试 (30分钟)
  ↓
阶段2: SLAM建图测试 (1小时)
  ↓
阶段3: 里程计融合验证 (15分钟)
  ↓
阶段4: 导航功能测试 (1小时)
```

---

## 🎯 阶段1: 基础功能测试

### 目的
验证所有硬件和基础软件正常工作

### 前提条件
- ✅ 所有传感器已连接
- ✅ 代码已编译 (`colcon build`)
- ✅ 环境已source (`source install/setup.bash`)

### 测试步骤

#### 1.1 硬件检测测试
```bash
# 检测Livox MID360
ls -l /dev/ttyUSB*
# 期望: 看到 /dev/ttyUSB0 或类似设备

# 检测RealSense D455
lsusb | grep Intel
# 期望: 看到 Intel Corp. RealSense

# 检测权限
groups
# 期望: 包含 dialout 组
```

**如果没有dialout权限**:
```bash
sudo usermod -a -G dialout $USER
# 然后注销重新登录
```

#### 1.2 消息定义测试
```bash
# 查看自定义消息
ros2 interface list | grep megarover_navigation

# 期望输出:
# megarover_navigation/msg/HealthStatus
# megarover_navigation/msg/SensorStatus
# megarover_navigation/srv/SwitchMode

# 查看详细定义
ros2 interface show megarover_navigation/msg/SensorStatus
```

#### 1.3 传感器独立测试

**测试Livox MID360**:
```bash
# 启动Livox
ros2 launch megarover_navigation livox_mid360.launch.py &
LIVOX_PID=$!
sleep 5

# 检查节点
ros2 node list | grep livox

# 检查话题
ros2 topic list | grep livox
# 期望: /livox/lidar, /livox/imu, /scan

# 检查数据频率
timeout 3 ros2 topic hz /livox/lidar
# 期望: ~10Hz

# 清理
kill $LIVOX_PID
```

**测试RealSense D455**:
```bash
# 启动RealSense
ros2 launch megarover_navigation realsense_d455.launch.py &
RS_PID=$!
sleep 8

# 检查话题
ros2 topic list | grep camera
# 期望: /camera/color/image_raw, /camera/depth/...

# 检查数据
timeout 2 ros2 topic echo /camera/color/image_raw --once

# 清理
kill $RS_PID
```

#### 1.4 里程计融合单独测试
```bash
# 运行测试脚本
./test_odometry_fusion.sh

# 期望: 所有步骤显示 ✓
```

### 验收标准
- [ ] 所有传感器硬件检测通过
- [ ] 自定义消息可正常查看
- [ ] Livox MID360数据正常发布
- [ ] RealSense D455数据正常发布
- [ ] 里程计融合节点启动成功

---

## 🗺️ 阶段2: SLAM建图测试

### 目的
验证SLAM系统能正确建图和定位

### 前提条件
- ✅ 阶段1测试全部通过
- ✅ 机器人可以移动（底盘正常）
- ✅ 有足够空间进行建图（建议>5m x 5m）

### 测试场景

#### 场景A: 融合模式SLAM（推荐）
**适用**: 两个传感器都正常

```bash
# 步骤1: 启动系统
./scripts/start_navigation.sh

# 在菜单中选择:
# 模式: 1 (fusion) 或直接回车（使用推荐）
# RViz: Y

# 步骤2: 等待系统初始化
# 观察RViz窗口，应该看到:
# - 机器人模型
# - 3D点云（来自Livox）
# - RGB-D图像（来自RealSense）
```

**建图操作**:
```bash
# 在另一个终端，使用键盘或手柄控制机器人
# 建议轨迹:
# 1. 向前移动2-3米
# 2. 右转90度
# 3. 向前移动2-3米
# 4. 再次转90度
# 5. 尝试回到起点（闭环检测）

# 重要提示:
# - 移动速度 < 0.3 m/s
# - 避免急转弯
# - 定期回到已知区域
```

**监控指标**:
```bash
# 在第三个终端监控

# 1. 检查RTABMAP状态
ros2 topic hz /rtabmap/info
# 期望: ~1-2 Hz

# 2. 检查地图点数
ros2 topic echo /rtabmap/mapData --once | grep "points"

# 3. 检查闭环检测
ros2 topic echo /rtabmap/info --once | grep "loop"
```

#### 场景B: LiDAR-Only模式
**适用**: 只有Livox MID360工作

```bash
./scripts/start_navigation.sh
# 选择模式: 2 (lidar_only)
```

#### 场景C: RGB-D-Only模式
**适用**: 只有RealSense D455工作

```bash
./scripts/start_navigation.sh
# 选择模式: 3 (rgbd_only)
```

### 保存地图
```bash
# 方法1: 使用RTABMAP工具
rtabmap-databaseViewer ~/.ros/rtabmap.db
# 在界面中: File -> Export -> Grid Map

# 方法2: 使用ROS2命令
ros2 run nav2_map_server map_saver_cli -f my_test_map
# 会生成: my_test_map.yaml 和 my_test_map.pgm
```

### 验收标准
- [ ] SLAM系统成功启动
- [ ] 能够看到实时地图更新
- [ ] 机器人位姿跟踪正常
- [ ] 成功检测到至少1次闭环（如果有回环）
- [ ] 地图保存成功

---

## 🔄 阶段3: 里程计融合验证

### 目的
验证里程计融合提高了定位精度

### 测试方法

#### 测试A: 对比测试
```bash
# 终端1: 启动系统（会自动启用融合）
./scripts/start_navigation.sh

# 终端2: 同时监控融合前后的数据
# 监控轮式里程计
ros2 topic echo /rover_odo --once

# 监控融合后里程计
ros2 topic echo /odom --once

# 对比姿态角度的差异
```

#### 测试B: 快速转向测试
```bash
# 1. 启动系统
./scripts/start_navigation.sh

# 2. 快速转向机器人（例如原地旋转）
# 3. 观察融合里程计的平滑度

# 监控
ros2 topic hz /odom
# 期望: 稳定50Hz

# 检查融合节点日志
ros2 node info /odometry_fusion
```

### 验收标准
- [ ] 融合节点正常运行（50Hz）
- [ ] /odom话题正常发布
- [ ] TF树包含 odom→base_footprint 变换
- [ ] 快速转向时姿态角更平滑
- [ ] CPU使用<5%

---

## 🎮 阶段4: 导航功能测试

### ⚠️ 重要前提条件

**在进行导航测试前，必须满足以下条件**:

1. **已有地图**
   - 选项A: 使用阶段2建立的地图
   - 选项B: 使用预先保存的地图

2. **已知起始位置**
   - 机器人在地图中的初始位姿已知
   - 或能够通过AMCL进行定位

3. **环境准备**
   - 测试区域无动态障碍物
   - 有明确的导航目标点
   - 路径上没有不可逾越的障碍

### 测试场景

#### 场景1: SLAM+导航模式（边建图边导航）

```bash
# 步骤1: 启动完整系统
./scripts/start_navigation.sh
# 选择 fusion 模式
# 启用 RViz

# 步骤2: 在RViz中设置导航目标
# 点击工具栏 "Nav2 Goal"
# 在地图上点击目标位置

# 系统会:
# 1. 规划路径
# 2. 控制机器人移动
# 3. 同时更新地图
```

**监控命令**:
```bash
# 终端2: 监控导航状态
ros2 topic echo /cmd_vel
# 期望: 看到速度命令

ros2 topic echo /plan
# 期望: 看到规划的路径

# 检查Nav2节点
ros2 node list | grep nav
```

#### 场景2: 纯导航模式（使用已有地图）

**前提**: 已有保存的地图文件

```bash
# 步骤1: 修改launch文件参数
# （需要创建专门的导航模式launch文件）

# 或者手动启动:
# 终端1: 加载地图
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/path/to/my_test_map.yaml

# 终端2: 启动AMCL定位
ros2 run nav2_amcl amcl

# 终端3: 启动Nav2
ros2 launch nav2_bringup navigation_launch.py

# 终端4: 设置初始位姿
# 在RViz中使用 "2D Pose Estimate"
```

### 导航测试用例

#### 用例1: 直线导航
```bash
# 目标: 向前方3米处导航

# 发送导航目标（命令行方式）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, \
   pose: {position: {x: 3.0, y: 0.0, z: 0.0}, \
   orientation: {w: 1.0}}}}"

# 观察机器人是否:
# - 规划出合理路径
# - 平滑移动到目标
# - 到达后停止
```

#### 用例2: 避障导航
```bash
# 1. 在路径上放置障碍物
# 2. 发送导航目标
# 3. 观察机器人是否绕过障碍物
```

#### 用例3: 多点导航
```bash
# 发送多个航点
ros2 action send_goal /navigate_through_poses \
  nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0}}}, \
            {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}]}"
```

### 验收标准
- [ ] 导航系统成功启动
- [ ] 路径规划生成合理路径
- [ ] 机器人能够跟踪路径
- [ ] 动态避障功能正常
- [ ] 到达目标点误差<20cm
- [ ] 无碰撞发生

---

## 🧪 快速测试脚本

为了方便测试，我将创建自动化测试脚本：

### 使用方法

```bash
# 阶段1: 编译测试
./scripts/test_build.sh

# 阶段2: SLAM测试（需要手动控制机器人）
./scripts/test_new_lidar_slam.sh

# 阶段3: 导航测试（需要已有地图）
./scripts/test_navigation.sh
```

---

## 🔍 故障排查

### 问题1: SLAM无法启动

**检查清单**:
```bash
# 1. 检查所有节点是否运行
ros2 node list

# 2. 检查话题连接
ros2 topic list
ros2 topic info /scan -v

# 3. 查看日志
ros2 node info /rtabmap

# 4. 检查TF树
ros2 run tf2_tools view_frames
```

### 问题2: 导航无响应

**检查清单**:
```bash
# 1. 检查/cmd_vel是否发布
ros2 topic hz /cmd_vel

# 2. 检查代价地图
ros2 topic echo /local_costmap/costmap_raw --once

# 3. 清除代价地图
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap

# 4. 检查Nav2状态
ros2 node info /bt_navigator
```

### 问题3: 里程计融合异常

**检查清单**:
```bash
# 1. 检查输入数据
ros2 topic hz /rover_odo
ros2 topic hz /livox/imu

# 2. 检查节点日志
ros2 node info /odometry_fusion

# 3. 重启融合节点
ros2 lifecycle set /odometry_fusion shutdown
```

---

## 📊 测试报告模板

测试完成后，填写此报告：

```markdown
# 测试报告

**日期**: ___________
**测试人**: ___________

## 阶段1: 基础功能
- [ ] 硬件检测: PASS / FAIL
- [ ] 消息定义: PASS / FAIL
- [ ] Livox传感器: PASS / FAIL
- [ ] RealSense传感器: PASS / FAIL
- [ ] 里程计融合: PASS / FAIL

## 阶段2: SLAM建图
- [ ] 系统启动: PASS / FAIL
- [ ] 实时建图: PASS / FAIL
- [ ] 闭环检测: PASS / FAIL
- [ ] 地图保存: PASS / FAIL

## 阶段3: 里程计融合
- [ ] 融合节点运行: PASS / FAIL
- [ ] 数据发布正常: PASS / FAIL
- [ ] 姿态更准确: PASS / FAIL

## 阶段4: 导航功能
- [ ] 导航启动: PASS / FAIL
- [ ] 路径规划: PASS / FAIL
- [ ] 路径跟踪: PASS / FAIL
- [ ] 避障功能: PASS / FAIL
- [ ] 目标到达: PASS / FAIL

## 问题记录
___________

## 总体评价
PASS / FAIL
```

---

## 💡 最佳实践

### SLAM建图时
1. **缓慢移动** - 速度<0.3m/s
2. **避免快速转向** - 给SLAM时间处理
3. **回到已知区域** - 帮助闭环检测
4. **保持光照稳定** - RGB-D模式需要

### 导航测试时
1. **先测试短距离** - 确保基本功能正常
2. **逐步增加难度** - 从简单到复杂
3. **观察安全距离** - 随时准备急停
4. **记录异常情况** - 便于后续改进

---

**重要提示**:
- 务必按照阶段顺序测试
- 每个阶段通过后再进行下一阶段
- 记录所有问题和解决方案
- 保存测试数据和日志

---

*版本: 1.0*
*最后更新: 2025-10-22*
