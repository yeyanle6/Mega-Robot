# MegaRover3 导航系统使用指南

**版本**: 1.0
**更新日期**: 2025-10-22
**适用系统**: ROS2 Humble, Ubuntu 22.04

---

## 🚀 快速开始

### 方式1：交互式启动（推荐）

```bash
cd ~/Code/Demo6
./start_navigation.sh
```

**启动流程**：
1. 自动检测传感器（Livox MID360, RealSense D455）
2. 显示推荐的SLAM模式
3. 让你选择使用的模式
4. 询问是否启动RViz
5. 确认后启动系统

**示例交互**：
```
==========================================
  MegaRover3 SLAM模式选择
==========================================

检测到的传感器：
  ✓ Livox MID360
  ✓ RealSense D455

推荐模式: 融合模式（最高精度）

可用模式：
  1) fusion       - 融合模式 (MID360 + D455, 最高精度)
  2) lidar_only   - 激光雷达模式 (仅MID360)
  3) rgbd_only    - RGB-D模式 (仅D455)
  4) odom_only    - 纯里程计模式 (降级)
  0) auto         - 自动模式 (推荐: fusion)

请选择模式 [0-4，直接回车使用推荐模式]:
```

### 方式2：命令行启动

```bash
# 使用自动检测模式
ros2 launch megarover_navigation modular_rtabmap.launch.py

# 强制使用特定模式
ros2 launch megarover_navigation modular_rtabmap.launch.py force_mode:=fusion

# 不启动RViz
ros2 launch megarover_navigation modular_rtabmap.launch.py rviz:=false

# 组合参数
ros2 launch megarover_navigation modular_rtabmap.launch.py force_mode:=lidar_only rviz:=true
```

---

## 📋 SLAM模式说明

### 1. Fusion模式（融合）
- **传感器**: Livox MID360 + RealSense D455
- **精度**: ⭐⭐⭐⭐⭐ 最高
- **适用场景**: 复杂室内外环境
- **CPU使用**: 60-80%
- **优势**: 视觉+激光互补，最佳精度

### 2. LiDAR Only模式（激光雷达）
- **传感器**: 仅Livox MID360
- **精度**: ⭐⭐⭐⭐
- **适用场景**: 开阔空间、长走廊
- **CPU使用**: 40-60%
- **优势**: 大范围3D建图

### 3. RGB-D Only模式
- **传感器**: 仅RealSense D455
- **精度**: ⭐⭐⭐
- **适用场景**: 特征丰富的室内
- **CPU使用**: 50-70%
- **优势**: 丰富的视觉特征

### 4. Odometry Only模式（降级）
- **传感器**: 仅轮式里程计
- **精度**: ⭐⭐
- **适用场景**: 传感器故障应急
- **CPU使用**: 10-20%
- **优势**: 最低资源消耗

---

## 🎮 操作指南

### 在RViz中设置导航目标

1. **启动系统**（带RViz）
   ```bash
   ./start_navigation.sh
   # 选择启动RViz: Y
   ```

2. **设置初始位姿**（仅使用已有地图时需要）
   - 点击工具栏 "2D Pose Estimate"
   - 在地图上点击机器人当前位置
   - 拖动设置朝向

3. **发送导航目标**
   - 点击工具栏 "Nav2 Goal"
   - 在地图上点击目标位置
   - 机器人将自动规划路径并移动

### 命令行控制导航

```bash
# 发送单个导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 发送多个航点
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}, \
            {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}]}"

# 取消当前导航
ros2 action send_goal /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

---

## 🔄 里程计融合

系统已自动集成里程计融合功能，无需额外配置。

### 工作原理
- **输入**: 轮式里程计 + IMU数据
- **输出**: 融合后的高精度里程计 (`/odom`)
- **方法**: 简单加权平均（轮式里程计70% + IMU 30%）

### 优势
- ✅ 提高姿态角度精度
- ✅ 减少轮子打滑误差
- ✅ 更平滑的里程计输出
- ✅ CPU开销<5%

### 详细信息
参见 [ODOMETRY_FUSION_GUIDE.md](ODOMETRY_FUSION_GUIDE.md) 获取完整使用指南。

---

## 🔧 传感器单独启动

### 启动Livox MID360

```bash
ros2 launch megarover_navigation livox_mid360.launch.py
```

**发布话题**:
- `/livox/lidar` - 3D点云
- `/livox/imu` - IMU数据
- `/scan` - 2D激光扫描

### 启动RealSense D455

```bash
ros2 launch megarover_navigation realsense_d455.launch.py
```

**发布话题**:
- `/camera/color/image_raw` - RGB图像
- `/camera/depth/aligned_depth_to_color/image_raw` - 深度图像
- `/camera/imu` - IMU数据

### 启动所有传感器

```bash
# 启动所有传感器
ros2 launch megarover_navigation all_sensors.launch.py

# 仅启动激光雷达
ros2 launch megarover_navigation all_sensors.launch.py enable_realsense:=false

# 仅启动相机
ros2 launch megarover_navigation all_sensors.launch.py enable_livox:=false
```

---

## 📊 监控和调试

### 查看系统状态

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic hz /odom

# 查看TF树
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 查看传感器数据

```bash
# 查看点云数据
ros2 topic echo /livox/lidar --once

# 查看激光扫描
ros2 topic echo /scan --once

# 查看里程计
ros2 topic echo /odom --once
```

### 查看导航状态

```bash
# 查看速度命令
ros2 topic echo /cmd_vel

# 查看导航计划
ros2 topic echo /plan

# 查看诊断信息
ros2 topic echo /diagnostics
```

---

## 🗺️ 地图管理

### 保存地图

```bash
# 方式1：从RTABMAP数据库导出
rtabmap-databaseViewer ~/.ros/rtabmap.db
# 在界面中: Export -> Grid Map

# 方式2：使用map_saver
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 使用已有地图

```bash
# 启动时加载地图（AMCL定位模式）
ros2 launch megarover_navigation navigation.launch.py \
  mode:=nav_only \
  map:=/path/to/my_map.yaml
```

---

## ⚠️ 故障排查

### 传感器无法连接

**Livox MID360**:
```bash
# 检查USB设备
ls -l /dev/ttyUSB*

# 添加权限
sudo usermod -a -G dialout $USER
# 注销并重新登录

# 手动设置权限（临时）
sudo chmod 666 /dev/ttyUSB0
```

**RealSense D455**:
```bash
# 检查USB连接
lsusb | grep Intel

# 重置USB
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo

# 测试相机
rs-enumerate-devices
```

### TF变换错误

```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo map base_link

# 查看TF延迟
ros2 run tf2_ros tf2_monitor
```

### 导航无响应

```bash
# 检查/cmd_vel是否发布
ros2 topic hz /cmd_vel

# 清除代价地图
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap

# 重启导航
# 先停止（Ctrl+C），然后重新启动
```

---

## 📁 文件结构

```
~/Code/Demo6/
├── start_navigation.sh              # 交互式启动脚本（推荐）
├── src/megarover_navigation/
│   ├── launch/
│   │   ├── megarover_nav2.launch.py    # Nav2导航启动
│   │   ├── modular_rtabmap.launch.py   # SLAM系统启动
│   │   └── sensors/                     # 传感器启动文件
│   │       ├── livox_mid360.launch.py
│   │       ├── realsense_d455.launch.py
│   │       └── all_sensors.launch.py
│   ├── config/
│   │   ├── nav2_params.yaml            # Nav2参数
│   │   ├── rtabmap_fusion.yaml         # Fusion模式配置
│   │   ├── rtabmap_lidar_only.yaml     # LiDAR模式配置
│   │   └── rtabmap_rgbd_only.yaml      # RGB-D模式配置
│   ├── msg/                             # 自定义消息
│   │   ├── SensorStatus.msg
│   │   └── HealthStatus.msg
│   └── srv/                             # 自定义服务
│       └── SwitchMode.srv
├── TEST_REPORT.md                   # 测试报告
└── USAGE_GUIDE.md                   # 使用指南（本文档）
```

---

## 💡 最佳实践

### 建图时
- ✓ 缓慢移动（< 0.3 m/s）
- ✓ 避免急转弯
- ✓ 定期回到已知区域（闭环检测）
- ✓ 确保良好照明（RGB-D模式）

### 导航时
- ✓ 先验证地图质量
- ✓ 设置合理的速度限制
- ✓ 保持传感器清洁
- ✓ 监控系统资源

### 系统维护
- ✓ 定期备份地图数据库
- ✓ 清理日志文件
- ✓ 更新软件包
- ✓ 校准传感器

---

## 🆘 获取帮助

### 常用命令速查
```bash
# 查看包信息
ros2 pkg list | grep megarover

# 查看Launch参数
ros2 launch megarover_navigation modular_rtabmap.launch.py --show-args

# 查看消息定义
ros2 interface show megarover_navigation/msg/SensorStatus
```

### 相关文档
- [TEST_REPORT.md](TEST_REPORT.md) - 系统测试报告
- [ARCHITECTURE_PLAN_V2.md](ARCHITECTURE_PLAN_V2.md) - 系统架构设计
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - 快速参考卡

---

**享受导航！** 🎉
