# MegaRover3 机器人系统启动指南

## 系统架构

### 硬件组成
- **底盘**: MegaRover Ver.3.0 (micro-ROS)
- **激光雷达**: Livox MID360
- **深度相机**: Intel RealSense D455 (当前主导航链已验证前置相机)
- **计算平台**: Jetson (Ubuntu 22.04 + ROS2 Humble)

### 通信架构
```
┌─────────────────┐   Serial:ttyUSB0  ┌──────────────────┐
│  MegaRover MCU  │◄─────────────────►│  micro_ros_agent │
│  (micro-ROS)    │    115200 baud    │                  │
└─────────────────┘                   └──────────────────┘
        │                                      │
        │ /rover_twist (Twist)                 │ ROS2 Topics
        │ /rover_odo (Twist)                   │
        ▼                                      ▼
┌─────────────────────────────────────────────────────────┐
│                      ROS2 Network                       │
└─────────────────────────────────────────────────────────┘
```

### ROS2 话题
| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/rover_twist` | geometry_msgs/Twist | PC → MCU | 速度指令 |
| `/rover_odo` | geometry_msgs/Twist | MCU → PC | 里程计反馈 |
| `/rover_sensor` | std_msgs/Int16MultiArray | MCU → PC | bumper 数字输入 + 电池电压 |
| `/odom` | nav_msgs/Odometry | pub_odom 发布 | 标准里程计消息 |
| `/livox/lidar` | sensor_msgs/PointCloud2 | MID360 | 点云数据 |
| `/livox/imu` | sensor_msgs/Imu | MID360 | IMU 数据 |

### TF 树
```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── left_wheel_1
                ├── right_wheel_1
                ├── pillar_fl_link / fr / bl / br
                ├── top_plate_link
                ├── mid360_base (高度56cm, 水平安装)
                │    └── mid360_lidar
                └── d455_front_bottom_screw_frame (前方, 倒置)
                     └── d455_front_link
                          ├── d455_front_depth_frame → d455_front_depth_optical_frame
                          ├── d455_front_color_frame → d455_front_color_optical_frame
                          └── d455_front_imu_frame
```

---

## 启动命令

### 1. 底盘系统 (micro-ROS Agent)

**当前实际接入方式: 有线串口**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4
```

**启动成功的典型日志特征:**
- `running... | fd: 3`
- `session established`
- `participant created`
- `datareader created`
- `datawriter created`

示例状态表示 MCU 已经通过串口连上 agent，ROS2 侧已经建立 XRCE-DDS session。

### 2. Livox MID360 激光雷达
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 3. Intel RealSense D455 深度相机

**重要**: Jetson 上必须使用 `LD_PRELOAD` 强制加载源码编译的 librealsense：
```bash
export LD_PRELOAD=/usr/local/lib/librealsense2.so
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

**前方相机启动 (d455_front) - SN: 239222302509 - 倒置安装:**
```bash
# 设置环境变量
export LD_PRELOAD=/usr/local/lib/librealsense2.so
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# 启动相机 (publish_tf:=false 使用 URDF 定义的 TF)
ros2 launch realsense2_camera rs_launch.py \
    camera_name:=d455_front \
    serial_no:="'239222302509'" \
    depth_module.depth_profile:=640x480x15 \
    pointcloud.enable:=true \
    publish_tf:=false

# 启动后动态启用点云
ros2 param set /camera/d455_front pointcloud__neon_.enable true
```

**后方相机启动 (d455_rear) - SN: 234222303981 - 正常安装:**
```bash
# 设置环境变量
export LD_PRELOAD=/usr/local/lib/librealsense2.so
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# 启动相机
ros2 launch realsense2_camera rs_launch.py \
    camera_name:=d455_rear \
    serial_no:="'234222303981'" \
    depth_module.depth_profile:=640x480x15 \
    pointcloud.enable:=true \
    publish_tf:=false

# 启动后动态启用点云
ros2 param set /camera/d455_rear pointcloud__neon_.enable true
```

**Costmap 使用的话题（经 relay 时间戳修正）:**
- `/d455_front_restamped` - 前方点云（NaN 清理 + LiDAR 时间戳替换，由 pointcloud_relay 发布）
- `/patchworkpp/nonground` - LiDAR 非地面点云（base_link 坐标系）
- Costmap 使用 SpatioTemporalVoxelLayer (STVL) 插件，动态障碍物 2-3 秒自动衰减

### 4. 机器人模型可视化 (仅查看)
```bash
# 基础模型
ros2 launch megarover_description mega3_view.launch.py

# 完整模型 (含 MID360 + D455)
ros2 launch megarover_description mega3_full_view.launch.py
```

### 5. 机器人 Bringup (实际运行)
```bash
# 基础启动 (URDF + pub_odom + RViz)
ros2 launch megarover3_bringup robot.launch.py

# 带 YDLiDAR TG30
ros2 launch megarover3_bringup robot.launch.py option:=_lrf

# 带深度相机
ros2 launch megarover3_bringup robot.launch.py option:=_depthcam
```

### 6. FASTLIO2 + OctoMap (建图)
```bash
ros2 launch fastlio2 fastlio2_octomap.launch.py
```

**数据流:**
```
MID360 → FASTLIO2 → /body_cloud → Patchwork++ → /patchworkpp/nonground → OctoMap → /map
```

### 7. FASTLIO2 + Nav2 (导航)

**当前唯一推荐主入口:**
```bash
# SLAM 模式 (建图)
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam

# 导航模式 (已有地图)
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py \
  mode:=nav \
  pcd_map:=/path/to/map.pcd \
  map:=/path/to/map.yaml
```

默认会内建启动 MID-360 驱动。如果 Livox 已经由其他终端或控制面板启动，追加 `start_lidar:=false`。

`fastlio2_navigation.launch.py` 和 `fastlio2_localization.launch.py` 目前仅保留作历史参考，不再作为现场启动入口。

---

## 当前已验证结论

截至 2026-03-09，以下链路已经现场验证通过：

- `micro_ros_agent(serial)` ↔ `/megarover` ↔ `/rover_twist` ↔ `/rover_odo`
- MID-360 驱动 ↔ `/livox/lidar`、`/livox/imu`
- FAST-LIO2 ↔ `/lio_odom`、`/body_cloud`
- Patchwork++ ↔ `/patchworkpp/nonground`
- D455 ↔ `/d455_front_restamped`
- OctoMap ↔ `/map`
- Nav2 ↔ `/plan`、`/cmd_vel`
- 底盘执行闭环 ↔ `/rover_twist`、`/rover_odo`

已实际确认：
- 现有地图 `/home/ros/Code/Demo8/maps/test_pgo_save.pcd` + `/home/ros/Code/Demo8/maps/test_pgo_save.yaml` 可用于 `mode:=nav`
- 在 RViz 中先执行 `2D Pose Estimate`，再发布导航目标，机器人可按目标运动，现场表现基本正常
- `nav_initializer.py` 已修正为将 RViz 的 `base_footprint` 初始位姿转换到 `lio_base` 后再调用 localizer
- `/nav_initializer/status` 只有在 localizer 真正确认有效后才进入 `LOCALIZED`
- `nav mode` 现已改为由统一的 `lifecycle_manager_navigation` 管理 `map_server`，避免静态地图节点掉回 `inactive` 导致 goal 发出后机器人不动
- `nav mode` 现已默认启用物理 bumper 安全监控节点 `bumper_safety_monitor.py`

### 物理 bumper 传感器说明

`/rover_sensor` 的字段定义如下：
- `data[0]`: `MU16_IM_DI`，即 bumper 数字输入位图
- `data[1]`: 当前电池电压 `[mV]`

实测 6 个 bumper 的位图映射为：

| 位置 | bit | 十六进制 |
|------|-----|----------|
| 前左 | bit0 | `0x0001` |
| 前中 | bit1 | `0x0002` |
| 前右 | bit2 | `0x0004` |
| 后右 | bit4 | `0x0010` |
| 后中 | bit5 | `0x0020` |
| 后左 | bit6 | `0x0040` |

说明：
- bumper 共 6 个，前 3 后 3
- `bit3` 未使用，对应官方说明书中的 “バンパー4 は欠番”

### 物理 bumper 当前处理逻辑

主导航入口默认会启动 `bumper_safety_monitor.py`。

当前逻辑：
1. 检测 `/rover_sensor.data[0]`
2. 一旦检测到 bumper 触发，先取消当前 `NavigateToPose`
3. 保持零速短暂停住
4. 若前 bumper 触发，则向正后方退让 5cm
5. 若后 bumper 触发，则向正前方退让 5cm
6. 退让完成后保持锁定，不自动恢复导航

默认参数：
- `bumper_stop_hold:=0.3`
- `bumper_retreat_distance:=0.05`
- `bumper_retreat_speed:=0.05`

手动复位命令：
```bash
source /home/ros/Code/Demo8/install/setup.bash
ros2 service call /bumper_safety/reset std_srvs/srv/Trigger "{}"
```

---

## 当前标准现场流程

### 导航推荐流程

```bash
# Terminal 1: 底盘通信
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4

# Terminal 2: 主导航入口
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py \
  mode:=nav \
  rviz:=false \
  map:=/home/ros/Code/Demo8/maps/test_pgo_save.yaml \
  pcd_map:=/home/ros/Code/Demo8/maps/test_pgo_save.pcd

# Terminal 3: RViz
rviz2 -d /home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/rviz/fastlio2_nav.rviz
```

### 导航操作顺序

1. 确认底盘串口 agent 已建立 session
2. 等待 `nav` 模式启动完成
3. 在 RViz 中执行一次 `2D Pose Estimate`
4. 确认 `/nav_initializer/status` 已进入 `LOCALIZED`
5. 再发布 `Nav2 Goal`

### 成功判据

- `/nav_initializer/status` 进入 `LOCALIZED`
- `/plan` 能生成路径
- `/cmd_vel` 与 `/rover_twist` 有速度输出
- `/rover_odo` 有底盘反馈
- 机器人能按目标点运动

### 当前禁止混用项

- 不要同时再手动启动旧入口 `fastlio2_navigation.launch.py` 或 `fastlio2_localization.launch.py`
- 不要在 FAST-LIO2 主链运行时再额外把 `pub_odom` 当主里程计源使用
- 控制面板如果已经单独启动 Livox 驱动，主入口需追加 `start_lidar:=false`

### 当前仍待优化项

- `livox_ros_driver2_node` 在 Ctrl-C 退出时仍有 `exit code -7`
- RViz 在当前显卡/OpenGL 环境下仍有纹理相关噪声日志
- costmap 在启动初期会短暂等待 `odom` TF，随后恢复正常
- `nav` 模式启动后，如果尚未执行 `2D Pose Estimate` 或 localizer 未进入 `LOCALIZED`，不要立即发导航目标

### 8. 遥控
```bash
# 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=rover_twist

# 鼠标遥控
ros2 launch megarover3_bringup mouse_teleop.launch.py
```

---

## 典型启动流程

### 场景 1: 建图 (SLAM)

```bash
# Terminal 1: 底盘通信
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4

# Terminal 2: Livox MID360
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Terminal 3: D455 前方相机 (可选, costmap 优化)
export LD_PRELOAD=/usr/local/lib/librealsense2.so LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 launch realsense2_camera rs_launch.py camera_name:=d455_front serial_no:="'239222302509'" depth_module.depth_profile:=640x480x15 pointcloud.enable:=true publish_tf:=false
# 启动后执行: ros2 param set /camera/d455_front pointcloud__neon_.enable true

# Terminal 4: D455 后方相机 (可选, costmap 优化)
export LD_PRELOAD=/usr/local/lib/librealsense2.so LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 launch realsense2_camera rs_launch.py camera_name:=d455_rear serial_no:="'234222303981'" depth_module.depth_profile:=640x480x15 pointcloud.enable:=true publish_tf:=false
# 启动后执行: ros2 param set /camera/d455_rear pointcloud__neon_.enable true

# Terminal 5: 主系统建图入口
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam

# Terminal 6: 键盘遥控 (可选)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=rover_twist
```

### 场景 2: 自主导航

```bash
# Terminal 1: 底盘通信
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4

# Terminal 2: Livox MID360
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Terminal 3: D455 前方相机 (costmap 优化)
export LD_PRELOAD=/usr/local/lib/librealsense2.so LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 launch realsense2_camera rs_launch.py camera_name:=d455_front serial_no:="'239222302509'" depth_module.depth_profile:=640x480x15 pointcloud.enable:=true publish_tf:=false
# 启动后执行: ros2 param set /camera/d455_front pointcloud__neon_.enable true

# Terminal 4: D455 后方相机 (costmap 优化)
export LD_PRELOAD=/usr/local/lib/librealsense2.so LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 launch realsense2_camera rs_launch.py camera_name:=d455_rear serial_no:="'234222303981'" depth_module.depth_profile:=640x480x15 pointcloud.enable:=true publish_tf:=false
# 启动后执行: ros2 param set /camera/d455_rear pointcloud__neon_.enable true

# Terminal 5: FASTLIO2 + Nav2
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py \
  mode:=nav \
  pcd_map:=/path/to/map.pcd \
  map:=/path/to/map.yaml
```

### 场景 3: 仅查看模型

```bash
ros2 launch megarover_description mega3_full_view.launch.py
```

---

## Launch 文件汇总

| 文件 | 包 | 功能 |
|------|-----|------|
| `mega3_view.launch.py` | megarover_description | 基础模型可视化 |
| `mega3_full_view.launch.py` | megarover_description | 完整模型可视化 |
| `robot.launch.py` | megarover3_bringup | 机器人 Bringup |
| `mouse_teleop.launch.py` | megarover3_bringup | 鼠标遥控 |
| `msg_MID360_launch.py` | livox_ros_driver2 | MID360 驱动 |
| `fastlio2_octomap.launch.py` | fastlio2 | FASTLIO2 + OctoMap 建图 |
| `fastlio2_pgo_navigation.launch.py` | megarover3_navigation | 当前唯一推荐主入口: FASTLIO2 + PGO + Localizer + Nav2 |
| `fastlio2_navigation.launch.py` | megarover3_navigation | legacy: 旧版 FASTLIO2 + Nav2 入口，不再推荐现场使用 |
| `fastlio2_localization.launch.py` | megarover3_navigation | legacy: 旧版定位入口，不再推荐现场使用 |
| `slam.launch.py` | megarover3_navigation | SLAM Toolbox |

---

## 传感器配置

### MID360 安装参数 (相对 base_link)
| 参数 | 当前值 | 备用值 | 说明 |
|------|--------|--------|------|
| X | 0.0 m | - | 左右居中 |
| Y | 0.09 m | 0.12 m | 前方偏移 (已后移 3cm) |
| Z | 0.56 m | - | 高度 56cm |
| Pitch | 0° | 30° (0.5236 rad) | 水平安装 |

- 配置文件: `megarover_description/urdf/calibration_offsets.xacro`
- 坐标系与 D455 相同: X=左右, Y=前后, Z=高度

### Intel RealSense D455 安装参数
| 相机 | 序列号 | 高度 | 位置 | 朝向 | 安装方式 |
|------|--------|------|------|------|----------|
| d455_front | 239222302509 | 16cm | 前方 | 前方 (+Y) | 倒置 (roll=180°) |
| d455_rear | 234222303981 | 16cm | 后方 (前相机后32cm) | 后方 (-Y) | 正常 |

**D455 物理参数:**
- 尺寸: 124mm x 29mm x 26mm
- 基线: 95mm
- 深度范围: 0.4m - 6m (室内)
- FOV: 87° x 58° (深度), 90° x 65° (RGB)
- 内置 IMU: BMI055

**优势 (相比 ZED Mini):**
- 不需要 NVIDIA GPU
- 可在任何平台运行 (Jetson / x86 / ARM)
- ROS2 原生支持 (realsense2_camera)

---

## 注意事项

1. 启动顺序: 先启动 `micro_ros_agent`，再启动其他节点
2. 确保 MID360 的 IP 配置正确 (默认 192.168.1.1xx)
3. Nav2 导航需要先有地图文件
4. 建图时建议低速移动，避免 FASTLIO2 跟踪丢失

## 现场启动检查清单

### 1. 底盘通信成功判据
- `micro_ros_agent` 日志里出现 `session established`
- 后续出现 `participant created`
- 后续出现 `datareader created`
- 后续出现 `datawriter created`

### 2. MID360 成功判据
- `ros2 topic list` 能看到 `/livox/lidar` 和 `/livox/imu`
- RViz 中能看到实时点云更新

### 3. D455 成功判据
- `ros2 topic list` 能看到 `/camera/d455_front/depth/color/points`
- 执行 `ros2 param set /camera/d455_front pointcloud__neon_.enable true` 后点云持续发布

### 4. 主系统建图成功判据
- `ros2 topic list` 能看到 `/lio_odom`、`/body_cloud`、`/patchworkpp/nonground`
- 若启用 D455，能看到 `/d455_front_restamped`
- 若启用 OctoMap，能看到 `/map`

### 5. 主系统导航成功判据
- `nav mode` 启动后 `map_server` 正常激活
- `nav_initializer` 状态进入 `LOCALIZED`
- RViz 中可以正常下发 `2D Goal Pose`

---

## 故障排除

### RealSense "bad optional access" 错误

**问题**: 在 Jetson 上启动 RealSense 相机时出现 "bad optional access" 错误。

**原因**: ROS2 的 realsense2_camera 节点默认链接到 `/opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so`，该库与 Jetson ARM64 平台不兼容。

**解决方案**: 使用 `LD_PRELOAD` 强制加载源码编译的 librealsense：
```bash
# 必须同时设置 LD_PRELOAD 和 LD_LIBRARY_PATH
export LD_PRELOAD=/usr/local/lib/librealsense2.so
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# 验证相机连接
/usr/local/bin/rs-enumerate-devices -s

# 然后启动 ROS2 相机节点
ros2 launch realsense2_camera rs_launch.py ...
```

**注意**: 仅设置 `LD_LIBRARY_PATH` 不够，因为 ROS2 库路径优先级更高。

**当前安装版本**: librealsense 2.57.2 (源码编译)

### USB 带宽问题

两个 D455 相机共享 10 Gbps USB hub 带宽。推荐使用 costmap 优化配置 (640x480x15, 仅深度) 以减少带宽占用。

---

## URDF 配置文件

| 文件 | 路径 | 说明 |
|------|------|------|
| `mega3.xacro` | `megarover_description/urdf/` | 主机器人描述文件 |
| `calibration_offsets.xacro` | `megarover_description/urdf/` | 传感器安装位置参数 |
| `mid360.xacro` | `megarover_description/urdf/sensors/` | MID360 激光雷达定义 |
| `d455.xacro` | `megarover_description/urdf/sensors/` | D455 深度相机定义 |

### 坐标系说明

- `base_link` 坐标系: 机器人本体坐标系 (X轴向左, Y轴向前)
- `base_footprint` 相对 `base_link`: xyz=(-0.1, 0, 0), yaw=-90°
- 机器人前进方向: `base_link` 的 +Y 方向

### D455 安装坐标 (相对 base_link)

| 相机 | X | Y | Z | Roll | Pitch | Yaw | 说明 |
|------|---|---|---|------|-------|-----|------|
| d455_front | 0.0 | 0.135 | 0.16 | 180° | 0° | 90° | 前方, 倒置 |
| d455_rear | 0.0 | -0.185 | 0.16 | 0° | 0° | -90° | 后方, 正常 |

---

## 待完成任务

- [x] 测试 D455 相机 TF 变换是否正确 (2026-01-29 已验证)
- [ ] 配置 Nav2 costmap 参数

---

## D455 TF 验证结果 (2026-01-29)

| 检查项 | d455_front | 状态 |
|--------|------------|------|
| 帧存在性 | 全部 6 个帧正常 | ✓ |
| 安装位置 | (0, 0.135, 0.16) m | ✓ |
| 安装姿态 | R=180°, P=0°, Y=90° | ✓ |
| 光学帧约定 | Z-forward, X-right, Y-down | ✓ |
| 视线方向 | (0, 1, 0) 指向机器人前方 | ✓ |
| 点云帧名称 | `d455_front_depth_optical_frame` | ✓ |

---

*最后更新: 2026-01-29 09:30*
