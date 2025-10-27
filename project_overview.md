# 项目概要文档 (Project Overview)

**项目名称**: Mega-Robot 自主导航系统
**创建日期**: 2025-10-22
**ROS2版本**: Humble (Ubuntu 22.04)
**主要语言**: Python 3
**工作空间**: `/home/wang/Code/Demo6`

---

## 一、项目目标

构建基于MegaRover Ver.3.0底盘的自主导航机器人系统，实现：
- 室内环境3D建图（SLAM）
- 自主定位与导航
- 多传感器融合
- 故障自动降级

---

## 二、硬件配置

### 2.1 底盘平台
- **型号**: MegaRover Ver.3.0
- **通信**: micro-ROS
- **Base Frame**: `base_footprint`
- **控制话题**: `/rover_twist` (⚠️ 非标准`/cmd_vel`)
- **里程计话题**: `/rover_odo` (需转换为`/odom`)
- **关键节点**: `pub_odom` (发布odom→base_footprint的TF)

### 2.2 传感器配置

#### 激光雷达 - Livox Mid-360
- **类型**: 非重复扫描固态激光雷达
- **输出格式**: PointCloud2
- **话题**:
  - `/livox/lidar` - 3D点云 (~10 Hz)
  - `/livox/imu` - IMU数据 (~200 Hz)
- **Frame**: `mid360_lidar`
- **连接**: USB (需dialout组权限)
- **安装角度**: **向前倾斜30度** (2025-10-22修改)
  - 后面增高，前面向下倾斜
  - Pitch = -30° = -0.5236 rad
  - 目的: 扫描低处障碍物
- **特点**: 点云密集，建议降采样

#### 深度相机 - Intel RealSense D455
- **分辨率**: 640x480 @ 30fps (推荐配置)
- **话题**:
  - `/camera/color/image_raw` - RGB图像
  - `/camera/depth/aligned_depth_to_color/image_raw` - 对齐深度图
  - `/camera/color/camera_info` - 相机参数
  - `/camera/imu` - IMU数据
- **Frame**: `d455_link`
- **连接**: USB 3.0 (必须)
- **配置**: 启用`align_depth` (深度对齐到彩色)

---

## 三、传感器配置方案

系统支持三种传感器配置，**当前采用**：**Fusion模式（推荐）**

| 配置方案 | 使用传感器 | 精度 | CPU占用 | 适用场景 |
|---------|-----------|------|---------|---------|
| **Fusion** (推荐) | Mid-360 + D455 | ⭐⭐⭐⭐⭐ | 60-80% | 复杂室内外环境 |
| LiDAR Only | 仅Mid-360 | ⭐⭐⭐⭐ | 40-60% | 开阔空间、长走廊 |
| RGB-D Only | 仅D455 | ⭐⭐⭐ | 50-70% | 特征丰富的室内 |
| Odometry Only | 轮式里程计 | ⭐⭐ | 10-20% | 应急降级模式 |

**配置记录位置**:
- Launch参数: `force_mode:=fusion`
- 配置文件: `config/rtabmap_fusion.yaml`

---

## 四、系统架构

### 4.1 技术栈

| 模块 | 技术选型 | 版本 |
|------|---------|------|
| SLAM与定位 | RTAB-Map | ROS2 Humble |
| 导航 | Nav2 | ROS2 Humble |
| 点云转换 | pointcloud_to_laserscan | ROS2 Humble |
| TF管理 | robot_state_publisher | ROS2 Humble |
| 底盘驱动 | megarover3_bringup | 自研 |
| 传感器管理 | sensor_detector.py | 自研 |

### 4.2 核心节点架构

```
[硬件层]
├── MegaRover3底盘 (micro_ros_agent + pub_odom)
│   └── 发布: /rover_odo, /rover_twist
├── Livox Mid-360 (livox_ros_driver2_node)
│   └── 发布: /livox/lidar, /livox/imu
└── RealSense D455 (camera/camera节点)
    └── 发布: /camera/color/*, /camera/depth/*, /camera/imu

[感知层]
├── pointcloud_to_laserscan_node
│   └── 转换: /livox/lidar → /scan (2D激光)
└── sensor_detector (自研)
    └── 自动检测传感器并选择配置

[SLAM层]
├── icp_odometry (RTAB-Map)
│   └── 输入: /livox/lidar → 输出: icp_odom, odom_filtered_input_scan
├── rtabmap (RTAB-Map核心)
│   └── 订阅: scan_cloud, rgb, depth, odom, imu
│   └── 发布: /rtabmap/grid_map, /rtabmap/cloud_map
└── rtabmap_viz (可视化)
    └── 3D点云地图可视化

[导航层] (Nav2)
├── planner_server (全局路径规划)
├── controller_server (局部路径跟踪)
├── bt_navigator (行为树)
└── lifecycle_manager (生命周期管理)

[TF层]
├── robot_state_publisher (URDF → TF)
├── pub_odom (odom → base_footprint)
├── rtabmap (map → odom, odom → base_link)
└── static_transform_publisher (mid360_lidar → mid360_imu)
```

### 4.3 TF树结构

```
map (RTABMAP发布)
 └── odom (pub_odom发布)
      └── base_footprint (RTABMAP发布 map→odom)
           └── base_link (icp_odometry发布 odom→base_link)
                ├── mid360_base_link (URDF静态)
                │    ├── mid360_lidar (URDF静态)
                │    └── mid360_imu (静态TF广播器)
                └── d455_link (URDF静态)
                     ├── d455_color_optical_frame
                     ├── d455_depth_optical_frame
                     └── ...其他相机frame
```

---

## 五、话题映射关系

### 5.1 底盘话题映射
```
/rover_odo → /odom (pub_odom节点转换)
/rover_twist ← /cmd_vel (Nav2输出需remap)
```

### 5.2 RTABMAP话题订阅 (Fusion模式)
```yaml
# 视觉部分
rgb/image        ← /camera/color/image_raw
depth/image      ← /camera/depth/aligned_depth_to_color/image_raw
rgb/camera_info  ← /camera/color/camera_info

# 激光雷达部分
scan_cloud       ← /livox/lidar

# 里程计和IMU
odom             ← /odom (融合里程计)
imu              ← /livox/imu (优先使用Mid-360的IMU)
```

---

## 六、ROS2包结构

```
/home/wang/Code/Demo6/src/
├── megarover_navigation/          # 主导航包 (自研)
│   ├── config/                    # 配置文件
│   │   ├── rtabmap_fusion.yaml   # Fusion模式配置
│   │   ├── rtabmap_lidar_only.yaml
│   │   ├── rtabmap_rgbd_only.yaml
│   │   ├── rtabmap_odom_only.yaml
│   │   └── nav2_params.yaml       # Nav2参数
│   ├── launch/                    # Launch文件
│   │   ├── modular_rtabmap.launch.py  # 核心SLAM启动文件
│   │   ├── navigation.launch.py
│   │   ├── megarover_nav2.launch.py
│   │   └── sensors/               # 传感器启动文件
│   ├── megarover_navigation/      # Python模块
│   │   ├── sensor_detector.py    # 传感器自动检测
│   │   ├── health_monitor.py     # 系统健康监控
│   │   └── odometry_fusion.py    # 里程计融合
│   └── msg/srv/                   # 自定义消息和服务
│
├── megarover3_ros2/               # 底盘驱动 (官方)
│   ├── megarover3_bringup/        # 底盘启动
│   │   └── src/pub_odom.cpp      # 里程计发布节点
│   └── megarover_description/     # URDF模型
│
├── livox_ros_driver2/             # Livox驱动 (官方)
├── realsense-ros/                 # RealSense驱动 (官方)
├── rtabmap/                       # RTAB-Map核心 (官方)
└── rtabmap_ros/                   # RTAB-Map ROS2接口 (官方)
```

---

## 七、核心设计原则

### 7.1 KISS原则
- 优先使用成熟开源方案 (RTAB-Map, Nav2)
- 配置扁平化，避免嵌套过深
- 节点职责单一

### 7.2 验证优先
- 每个方案反复验证是否符合硬件配置
- 不确定的内容明确标注待验证
- 充分的测试脚本和验证流程

### 7.3 渐进式开发
1. ✅ 阶段1: 硬件驱动验证
2. ✅ 阶段2: 传感器配置选择
3. ✅ 阶段3: SLAM建图
4. 🔄 阶段4: 定位验证
5. ⏳ 阶段5: 导航集成
6. ⏳ 阶段6: 性能优化

---

## 八、关键约束和注意事项

### 8.1 MegaRover3底盘
⚠️ **速度控制话题为`/rover_twist`** (非标准`/cmd_vel`)
⚠️ **Base frame必须是`base_footprint`**
⚠️ **里程计话题`/rover_odo`需转换为`/odom`**

### 8.2 Livox Mid-360
⚠️ **非重复扫描**，输出PointCloud2
⚠️ **点云密集**，建议降采样 (VoxelSize: 0.075)
⚠️ **需USB权限**: `sudo usermod -a -G dialout $USER`

### 8.3 RealSense D455
⚠️ **必须USB 3.0**连接
⚠️ **必须启用`align_depth`** (深度对齐到彩色)
⚠️ **建议640x480@30fps** (平衡性能和精度)

### 8.4 RTAB-Map配置
⚠️ **建图模式**: `Mem/IncrementalMemory=true`
⚠️ **定位模式**: `Mem/IncrementalMemory=false`
⚠️ **不同传感器组合需要不同subscribe参数**

### 8.5 话题映射管理
⚠️ **统一在launch文件中管理remappings**
⚠️ **YAML配置文件中的`*_topic`参数会被launch remappings覆盖**
⚠️ **避免配置冲突，单一来源管理**

---

## 九、重要修复记录

### 9.1 TF空frame_id错误修复 (2025-10-22)
- **文件**: `src/megarover3_ros2/megarover3_bringup/src/pub_odom.cpp`
- **问题**: `pub_odom`节点发布包含空frame_id的TF变换
- **修复**: 在构造函数中初始化TF消息的frame_id
- **影响**: 消除数百个TF错误，系统稳定性大幅提高

### 9.2 mid360_imu frame静态TF (2025-10-22)
- **文件**: `src/megarover_navigation/launch/modular_rtabmap.launch.py`
- **问题**: URDF定义了`mid360_imu` frame，但没有TF广播器
- **修复**: 添加静态TF广播器 (mid360_lidar → mid360_imu)
- **影响**: TF树完整，支持需要IMU frame的滤波器

### 9.3 配置文件清理 (2025-10-22)
- **文件**: `config/rtabmap_*.yaml`
- **问题**: YAML中的话题参数被launch remappings覆盖，造成混淆
- **修复**: 移除YAML中的`*_topic`参数，统一在launch文件管理
- **影响**: 配置清晰，避免冲突

### 9.4 Mid-360倾斜角度修改 (2025-10-22) ⭐
- **文件**: `src/megarover3_ros2/megarover_description/urdf/mega3.xacro`
- **问题**: 水平安装无法扫描低处障碍物
- **修改**: 向前倾斜30度 (pitch = -30° = -0.5236 rad)
- **影响**: 能够检测地面和低处障碍物，改善避障性能
- **详细文档**: `MID360_TILT_MODIFICATION.md`

---

## 十、参考文档

### 10.1 核心文档（本次创建）
- **project_overview.md** - 本文档，项目概要
- **project_progress.md** - 当前进度和任务
- **project_issues.md** - 问题记录和解决方案

### 10.2 详细技术文档
- `COMPLETE_FIX_SUMMARY.md` - 完整修复摘要
- `USAGE_GUIDE.md` - 使用指南
- `TESTING_GUIDE.md` - 测试指南

### 10.3 外部参考
- [RTAB-Map Wiki](http://wiki.ros.org/rtabmap_ros)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [RealSense ROS](https://github.com/IntelRealSense/realsense-ros)

---

## 十一、系统性能指标

### 11.1 目标性能
- **导航精度**: ±20cm
- **响应时间**: <500ms
- **CPU使用**: <70% (Fusion模式)
- **内存使用**: <3GB
- **稳定运行**: >1小时无故障

### 11.2 实测性能 (待补充)
- 建图精度: 待测试
- 定位精度: 待测试
- 导航成功率: 待测试
- 系统资源占用: 待测试

---

**文档版本**: v1.0
**最后更新**: 2025-10-22
**维护者**: Wang
**状态**: 第一次对话创建

---

## 变更记录

| 日期 | 版本 | 变更内容 | 修改人 |
|------|------|---------|--------|
| 2025-10-22 | v1.0 | 初始创建项目概要文档 | Claude Code |

---

**注意**: 本文档为**稳定文档**，硬件配置、架构等基础信息确定后很少修改。
**日常进度跟踪请查看** `project_progress.md`。
