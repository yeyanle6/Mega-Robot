# Demo8 项目状态记录

**更新日期**: 2026-03-10
**ROS2 版本**: Humble (Ubuntu 22.04)
**机器人**: MegaRover3 差速驱动平台

---

## 1. 项目概述

基于 FAST-LIO2 + Nav2 的室内自主导航系统。使用 Livox MID-360 3D LiDAR 做 SLAM/定位，Intel RealSense D455 深度相机做近距障碍物检测，Nav2 做路径规划和运动控制。

### 当前项目状态结论

项目已完成从传感器、定位、地图、导航到底盘执行的现场闭环验证，不再处于“仅代码集成完成”的阶段。

2026-03-09 实测已验证：
- `test_pgo_save.pcd + test_pgo_save.yaml` 可用于 `mode:=nav`
- RViz 中先做 `2D Pose Estimate`，再发布导航目标，可完成实际导航
- `/plan`、`/cmd_vel`、`/rover_twist`、`/rover_odo` 全链路闭环正常
- 机器人已现场验证可按目标运动，表现基本正常
- 物理 bumper 触发已可通过 `/rover_sensor` 被 ROS2 侧接管处理

当前项目主线已经进入“稳定性优化和现场流程固化”阶段。

### 硬件传感器

| 传感器 | 型号 | 用途 | 安装位置 |
|--------|------|------|----------|
| 3D LiDAR | Livox MID-360 | SLAM + 360 障碍物检测 + clearing | 顶部 (0.09m前, 0.56m高) |
| 深度相机 | Intel D455 | 前方近距障碍物补充 (0.2-2.5m) | 前方 |
| 底盘 MCU | micro-ROS (串口 `/dev/ttyUSB0`, 115200) | 电机控制 + 轮式里程计 | 底盘 |
| IMU | Livox 内置 | LiDAR-惯性融合 | 与 LiDAR 一体 |

### 软件架构

```
MID-360 → FAST-LIO2 → PGO(回环优化) → odom→lio_base TF
                ↓                              ↓
          /body_cloud                   lio_base→base_footprint (静态TF, yaw+90°)
                ↓
        Patchwork++ (地面分割)
                ↓
    /patchworkpp/nonground ──────────────────→ Nav2 costmap (global+local)
                ↓
      pointcloud_relay.py ← D455 /depth/points
          ↓           ↓
    /merged_cloud   /d455_front_restamped ──→ Nav2 local costmap only
          ↓
    OctoMap → /map (2D栅格)
                      ↓
                 Nav2 (全局规划 + 局部控制)
                      ↓
              /rover_twist → micro-ROS → 电机
```

---

## 2. 项目结构

```
/home/ros/Code/Demo8/
├── src/
│   ├── megarover3_ros2/                        [主项目 -- 4个子包]
│   │   ├── megarover3/                         (元包)
│   │   ├── megarover3_navigation/              [核心导航包 -- 下文详述]
│   │   ├── megarover3_bringup/                 (底盘启动: robot.launch.py, 遥控)
│   │   └── megarover_description/              (URDF: mega3.xacro + d455/mid360传感器)
│   │
│   ├── FASTLIO2_ROS2/                          [LiDAR SLAM 系统 -- 5个子包]
│   │   ├── fastlio2/                           (核心LIO: lio_node)
│   │   ├── localizer/                          (基于PCD地图定位: localizer_node)
│   │   ├── pgo/                                (位姿图优化: pgo_node + save_maps服务)
│   │   ├── hba/                                (层次化BA)
│   │   └── interface/                          (srv定义: SaveMaps, Relocalize等)
│   │
│   ├── livox_ros_driver2/                      (Livox LiDAR驱动)
│   ├── patchwork-plusplus/                      (地面分割算法)
│   ├── spatio_temporal_voxel_layer/            (STVL costmap插件 -- 1月29日新增)
│   │   └── openvdb_vendor/                     (OpenVDB依赖)
│   ├── octomap_server2-master/                 (OctoMap 3D→2D投影)
│   ├── vs_rover_options_description/           (附加URDF描述)
│   ├── micro_ros_setup/                        (micro-ROS配置)
│   └── uros/                                   (micro-ROS Agent + msgs)
│
├── build/                                      (编译产物)
├── install/                                    (安装目录, symlink-install)
├── log/                                        (colcon日志)
└── maps/                                       (保存的地图)
```

### megarover3_navigation 详细结构

```
megarover3_navigation/
├── config/
│   ├── fastlio2_nav2_params.yaml       ★ Nav2主配置 (FASTLIO2模式, D455+LiDAR)
│   ├── fastlio2_nav2_params_lidar_only.yaml  Nav2配置 (純LiDAR模式, 无D455)
│   ├── mega3_nav2_params.yaml            Nav2配置 (原始模式)
│   ├── f120a_nav2_params.yaml            Nav2配置 (F120A模式)
│   ├── mapper_params_online_async.yaml   SLAM Toolbox异步配置
│   └── mapper_params_online_sync.yaml    SLAM Toolbox同步配置
│
├── launch/
│   ├── fastlio2_pgo_navigation.launch.py  ★ 主launch (slam/nav/slam_simple)
│   ├── fastlio2_navigation.launch.py       FASTLIO2简易launch
│   ├── fastlio2_localization.launch.py     定位模式launch
│   ├── navigation.launch.py               通用导航launch
│   ├── slam.launch.py                     SLAM Toolbox launch
│   └── ... (其他legacy launch)
│
├── scripts/
│   ├── pointcloud_relay.py             ★ 点云中继 (LiDAR+D455合并, 地面过滤)
│   ├── planar_compensation.py            平面补偿 (lio_odom → map/odom TF)
│   ├── nav_initializer.py               导航初始化 (加载PCD地图+初始位姿)
│   ├── bumper_safety_monitor.py          bumper急停/退让/锁定/手动复位
│   ├── nav_loop_regression.py            两点循环导航回归测试 (统计耗时/成功率)
│   ├── costmap_residual_probe.py         动态障碍物移除后costmap残留衰减测量
│   ├── nav_debug_logger.py              导航调试日志记录
│   ├── calibrate_d455.py               D455外参自动标定 (ground/wall/full/verify)
│   ├── calibration_diagnostic.py        传感器标定诊断 (LiDAR vs D455对比)
│   ├── measure_d455_height.py           D455高度测量 (地面平面RANSAC)
│   └── control_panel.py                 Qt GUI控制面板入口
│
├── gui/
│   ├── main_window.py                   主窗口
│   ├── process_manager.py               进程管理
│   ├── ros2_monitor.py                  ROS2状态监控
│   ├── map_save_dialog.py               地图保存对话框
│   ├── map_saver_thread.py              地图保存线程
│   ├── log_handler.py                   日志处理
│   ├── config/panel_config.yaml         面板配置
│   ├── resources/styles.qss             Qt样式表
│   └── widgets/                         自定义控件
│
├── maps/                                存储的地图文件 (pgm+yaml+pcd)
├── rviz/                                RViz配置 (fastlio2_nav.rviz等)
├── CMakeLists.txt                       构建配置
└── package.xml                          包描述
```

---

## 3. 运行模式

通过 `fastlio2_pgo_navigation.launch.py` 的 `mode` 参数切换：

### slam 模式 (建图)
```bash
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam
```
- FAST-LIO2 → PGO(回环检测) → 优化点云
- OctoMap → 实时2D栅格地图
- Nav2 全栈运行（可边建图边导航）

### nav 模式 (导航)
```bash
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=nav \
  pcd_map:=/path/to/map.pcd map:=/path/to/map.yaml
```
- FAST-LIO2 + Localizer (scan-to-map定位)
- 加载预建2D地图 → Nav2导航

### slam_simple 模式
- 仅 FAST-LIO2（无PGO回环优化）

---

## 4. 关键 TF 链

```
              map
               │
         (planar_compensation.py)
               │
             odom
               │
         (FAST-LIO2 输出)
               │
           lio_base
               │
     (静态TF: yaw=+90°, y=0.1m)
               │
         base_footprint
               │
           base_link
            /      \
     livox_frame   d455_front_link
                       │
               d455_front_optical
```

**关键修改**: FAST-LIO2 输出 `odom→lio_base`，但 lio_base 中 +Y=物理前方。通过静态 TF (`lio_base→base_footprint`, yaw=90°) 将坐标系对齐到 ROS 标准 (+X=前方)。

---

## 5. 当前修改点详细记录

### 5.0 2026-03-09 新验证与修正摘要

本轮完成的关键验证与修正如下：

#### 已验证通过
- 串口底盘通信：`micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4`
- MID-360 驱动、FAST-LIO2、Patchwork++、D455、OctoMap、Nav2
- 导航执行链：`/plan -> /cmd_vel -> /rover_twist -> /rover_odo`

#### 已修复问题
- `/map` QoS 与 Nav2 global costmap 不兼容
  - 修复位置：`octomap_server.cpp`
  - 结果：`/map` 改为 `TRANSIENT_LOCAL`，global costmap 可正常接收

- `pointcloud_relay.py` / `planar_compensation.py` Ctrl-C 退出噪声
  - 修复位置：对应 Python 脚本 `main()` 关停逻辑
  - 结果：两者现已 clean shutdown

- `nav_initializer.py` 将“relocalize 请求已提交”误判为“定位已成功”
  - 修复位置：`nav_initializer.py`
  - 结果：新增 `VERIFYING` 状态，并通过 `/localizer/relocalize_check` 作为真正成功判据

### 5.1 2026-03-10 导航整段测试现状

本轮重点已经从“单点能否到达”转为“整段路径里机器人如何运动、为何中途退化或中止”。

当前结论：
- D455 链路已不再是唯一主故障，`pointcloud_relay.py` 已支持 costmap 分支体素降采样与统计日志。
- Nav2 末端 `yaw` 收敛问题已通过诊断确认存在：
  - 将 `yaw_goal_tolerance` 临时调到 `3.14` 后，单点 `p1` 从 `148.4s` 直接缩到 `7.6s`。
  - 说明“目标点附近左右徘徊”的根因与 `RotateToGoal` 主导的末端朝向对齐高度相关。
- 但 4 点回归表明，系统仍存在更底层的局部控制退化：
  - 某些段里 `global_plan_length` 仍有值，但 `local_plan_length -> 0.0`
  - 此时 `cmd.linear_x -> 0`，`cmd.angular_z` 仍非零
  - 机器人进入“只转不走”的局部控制状态，最终 action `aborted`

当前正式参数方向：
- D455 几何过滤保持折中值：
  - `costmap_max_range = 1.8`
  - `costmap_max_height = 0.7`
- `global_costmap` 动态障碍残留已收紧：
  - `voxel_decay = 3.0`
- near-goal / DWB 正式参数：
  - `xy_goal_tolerance = 0.35`
  - `yaw_goal_tolerance = 1.0`
  - `general_goal_checker.stateful = False`
  - `FollowPath.stateful = False`
  - `RotateToGoal.scale = 5.0`
  - `RotateToGoal.lookahead_time = 1.5`

当前 4 个固定测试点：
- `p1 = (-0.92, -6.47, 1.307)`
- `p2 = (0.35, -1.77, 1.497)`
- `p3 = (0.81, 4.49, 3.010)`
- `p4 = (-10.15, 5.94, -0.931)`

### 5.2 2026-03-10 回归脚本日志增强

`nav_loop_regression.py` 已扩展为“双日志输出”：

- 人工可读摘要日志：
  - `~/nav_regression_logs/nav_loop_*.log`
- 结构化全过程轨迹日志：
  - `~/nav_regression_logs/nav_loop_*.jsonl`

`.jsonl` 会按时间顺序记录：
- `meta`
- `goal_dispatch`
- `goal_accepted`
- `sample`
- `event`
- `goal_summary`
- `run_summary`

其中 `sample` 包含：
- 当前 run 名称
- 目标点 `name/x/y/yaw`
- 机器人位姿 `pose.x / pose.y / pose.yaw`
- 当前控制指令 `cmd.linear_x / cmd.angular_z`
- 当前里程计速度
- 当前 `global_plan_length`
- 当前 `local_plan_length`

这样后续可以完整回放“机器人在整段路径中何时开始只转不走、何时 local plan 塌成 0、何时 action 中止”。

### 5.3 2026-03-10 已记录的关键测试结果

#### 单点 `p1` 诊断

诊断日志：
- `~/nav_regression_logs/nav_loop_20260310_114248_new_map_20260310_0918_p1_single_diag_cycles1.log`
- `~/nav_regression_logs/nav_loop_20260310_121822_new_map_20260310_0918_p1_yawdiag314_cycles1.log`

关键结果：
- 原参数单点 `p1`：`148.4s`, `succeeded`
- `yaw_goal_tolerance = 3.14` 诊断版：`7.6s`, `succeeded`

结论：
- `p1` 的“目标附近左右徘徊”已实锤是末端 yaw 收敛问题

#### 四点整段 full-trace 测试

日志：
- `~/nav_regression_logs/nav_loop_20260310_123030_new_map_20260310_0918_fourpoint_loop3_fulltrace_cycles6.log`
- `~/nav_regression_logs/nav_loop_20260310_123030_new_map_20260310_0918_fourpoint_loop3_fulltrace_cycles6.jsonl`

已跑出的结果：
- `run_01_p1`: `aborted`, `114.2s`
- `run_02_p2`: `aborted`, `0.9s`
- `run_03_p3`: `aborted`, `68.4s`
- `run_04_p4`: `aborted`, `45.6s`

整段分析结论：
- `p1 / p3 / p4` 期间全局路径长度显著缩短，说明机器人确实推进了很长一段
- 但在关键局部状态下：
  - `local_plan_length -> 0.0`
  - `cmd.linear_x -> 0`
  - `cmd.angular_z` 仍非零
- 即便还有全局路径，DWB 也可能退化成“只转不走”

这说明当前阶段的主问题已经收敛到：
- 局部规划器在某些几何状态下无法生成可执行前进轨迹
- 不是简单的“没有全局路径”
- 也不是单纯的 D455 性能瓶颈

---

## 6. 当前推荐启动命令

### 6.1 导航启动

```bash
source /home/ros/Code/Demo8/install/setup.bash
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py \
  mode:=nav \
  map:=/home/ros/Code/Demo8/maps/new_map_20260310_0918/map.yaml \
  pcd_map:=/home/ros/Code/Demo8/maps/new_map_20260310_0918/map.pcd \
  use_last_pose:=true \
  rviz:=true \
  nav2_params_file:=/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/config/fastlio2_nav2_params_d455_tuned.yaml
```

启动后流程：
- 在 RViz 中执行 `2D Pose Estimate`
- 不要再手动点击 `2D Goal Pose`
- 由回归脚本独占发目标

### 6.2 四点三轮全过程回归

```bash
source /home/ros/Code/Demo8/install/setup.bash
ros2 run megarover3_navigation nav_loop_regression.py \
  --goal p1 -0.92 -6.47 1.307 \
  --goal p2 0.35 -1.77 1.497 \
  --goal p3 0.81 4.49 3.010 \
  --goal p4 -10.15 5.94 -0.931 \
  --loops 3 \
  --timeout 180 \
  --settle 3 \
  --map-name new_map_20260310_0918_fourpoint_loop3_fulltrace
```

### 6.3 单点 `p1` 诊断

```bash
source /home/ros/Code/Demo8/install/setup.bash
ros2 run megarover3_navigation nav_loop_regression.py \
  --goal-a -0.92 -6.47 1.307 \
  --goal-b -0.92 -6.47 1.307 \
  --cycles 1 \
  --timeout 180 \
  --settle 3 \
  --map-name new_map_20260310_0918_p1_single_diag
```

- RViz `2D Pose Estimate` 与 localizer 初始位姿坐标系不一致
  - 根因：RViz 输入语义为 `base_footprint`，localizer 需要 `lio_base`
  - 修复位置：`nav_initializer.py`
  - 结果：现在会先做 `base_footprint -> lio_base` 转换，再调用 `/localizer/relocalize`
  - 验证结果：修复后 `2D Pose Estimate` 可正确进入 `LOCALIZED`

- `nav` 模式下 `map_server` 偶发掉回 `inactive`，导致再次发布 goal 时机器人不动
  - 根因：`map_server` 由独立 `lifecycle_manager_map` 管理，存在生命周期竞态
  - 修复位置：`fastlio2_pgo_navigation.launch.py`
  - 结果：`nav` 模式改为只保留一个 `lifecycle_manager_navigation`，并将 `map_server` 纳入统一管理
  - 验证结果：修复后 `map_server` 可稳定保持 `active`，再次发布 goal 可继续进入执行

- 物理 bumper 触发后系统此前无任何 ROS2 监听与处理
  - 根因：底盘虽然发布 `/rover_sensor`，但导航主链没有订阅并接管该信号
  - 资料依据：
    - `/rover_sensor.data[0] = MU16_IM_DI`（数字输入）
    - `/rover_sensor.data[1] = 电池电压[mV]`
    - 来源：`メガローバー3.0_ROS 2取扱説明書_20240328.pdf`
  - 修复位置：
    - `megarover3_navigation/scripts/bumper_safety_monitor.py`
    - `fastlio2_pgo_navigation.launch.py`
  - 当前处理逻辑：
    - 取消当前 `NavigateToPose`
    - 保持零速短暂停住
    - 如果前 bumper 触发，则向正后方退让 5cm
    - 如果后 bumper 触发，则向正前方退让 5cm
    - 退让完成后保持锁定，等待手动复位
  - 手动复位服务：
    - `ros2 service call /bumper_safety/reset std_srvs/srv/Trigger "{}"`

#### 5.0.1 物理 bumper 位图映射（2026-03-09 实测）

基于 `/rover_sensor.data[0]`（`MU16_IM_DI`）逐个手动触发确认：

| 位置 | bit | 十六进制 |
|------|-----|----------|
| 前左 | bit0 | `0x0001` |
| 前中 | bit1 | `0x0002` |
| 前右 | bit2 | `0x0004` |
| 后右 | bit4 | `0x0010` |
| 后中 | bit5 | `0x0020` |
| 后左 | bit6 | `0x0040` |

说明：
- 总数为 6 个 bumper，前 3 后 3
- `bit3` 未使用，对应说明书中的“バンパー4 は欠番”

#### 5.0.2 物理 bumper 急停/退让链验证（2026-03-09 实测）

前中 bumper 实测日志已确认以下顺序：

1. 检测到 `/rover_sensor.data[0] = 0x0002`（`front_center`）
2. `bumper_safety_monitor` 触发
3. 取消当前导航目标
4. 控制器先执行 `Stopping the robot`
5. 保持零速 `0.3s`
6. 以 `-0.05m/s` 后退约 `1.0s`（约 5cm）
7. 进入 `latch_stop`，等待手动复位

当前默认参数：
- `bumper_stop_hold = 0.3`
- `bumper_retreat_distance = 0.05`
- `bumper_retreat_speed = 0.05`

### 5.1 导航方向修复 (已完成)

**问题**: FAST-LIO2 body frame 中 +Y=前方，Nav2 期望 +X=前方，导致导航方向反转。

**修复**:
- `lio_megarover.yaml`: `body_frame: lio_base` (从 `base_footprint` 改为中间帧)
- `fastlio2_pgo_navigation.launch.py`: 添加静态 TF `lio_base→base_footprint` (yaw=+90°)

### 5.2 Nav2 参数调优 -- 进度检查器冲突修复 (已完成, 实验验证)

**问题**: DWB RotateToGoal 强制纯旋转 (vx=0)，与 SimpleProgressChecker 的移动要求冲突，导致近目标反复触发 "Failed to make progress"，陷入 recovery 循环。

**修复** (`fastlio2_nav2_params.yaml`):
```yaml
# 之前 → 之后
progress_checker:
  required_movement_radius: 0.5 → 0.25    # 降低移动距离要求
  movement_time_allowance: 10.0 → 20.0    # 延长允许时间窗口

general_goal_checker:
  yaw_goal_tolerance: 0.25 → 0.50         # 放宽朝向容忍度
```

**验证结果**: "Failed to make progress" 从每次导航 2-6 次降至 0 次。

### 5.3 D455 地面误识别修复 (已完成代码, 待测试)

**问题**: D455 深度相机将地面点识别为障碍物（尤其在门附近），在 costmap 中形成虚假墙壁。

**修复方案 (三层)**:

#### 第一层: pointcloud_relay.py 地面过滤
新增基于 TF 的高度裁剪，在 `_publish_costmap_cloud()` 中：
- 距离过滤: 仅保留 0.2~2.5m 范围内的点
- 地面高度过滤: 通过缓存的静态 TF (camera_optical→base_footprint) 计算每个点在 base_footprint 帧的 z 值，仅保留 0.15m~1.2m 高度的点
- 实现方式: 纯 numpy 矩阵运算，无 PCL 依赖

新增参数:
```python
costmap_min_height = 0.15   # 低于此高度视为地面
costmap_max_height = 1.2    # 高于此高度视为天花板
costmap_min_range = 0.2     # D455最小可靠深度
costmap_max_range = 2.5     # 超出让LiDAR处理
```

#### 第二层: Nav2 costmap 配置调整

**Global costmap**:
- 完全移除 D455 (`observation_sources: lidar_cloud`)
- 全局规划只依赖 LiDAR 360° 数据

**Local costmap**:
- D455 设为 `clearing: false` (窄 FOV 不做清除，避免旋转时 costmap 闪烁)
- D455 仅 `marking: true` (标记近距障碍物)
- `min_obstacle_height: 0.10` (二次保险)
- `min_z: 0.15`
- `voxel_decay: 8.0` (从 15.0 降低，加速障碍物消散)

#### 第三层: 架构分工
```
MID-360 (Patchwork++过滤后):
  - Global costmap: marking + clearing (360° 全方位)
  - Local costmap:  marking + clearing (360° 全方位)

D455 (pointcloud_relay地面过滤后):
  - Global costmap: 不参与
  - Local costmap:  marking only (0.2-2.5m 近距补充)
```

### 5.4 Costmap 参数调整 (已完成)

```yaml
# 机器人尺寸 + 膨胀
robot_radius: 0.18
inflation_radius: 0.22
cost_scaling_factor: 15.0

# Local costmap STVL
voxel_decay: 8.0        # 障碍物 8 秒消散 (原 15.0)

# Global costmap STVL
voxel_decay: 30.0       # 全局保持 30 秒
```

### 5.5 RViz QoS 修复 (已完成)

修改 `fastlio2_nav.rviz`，将点云/地图话题的 QoS 从 Reliable 改为 Best Effort，匹配传感器 QoS。

### 5.6 PGO/Localizer 话题匹配 (已完成)

- `pgo.yaml`: 配置输入话题为 `/body_cloud`、`/lio_odom` (FAST-LIO2 输出)
- `localizer.yaml`: 配置输入话题为 `/body_cloud`、`/lio_odom`
- `fastlio2_pgo_navigation.launch.py`: remapping 确保话题一致

---

## 6. 待完成事项

### P0 -- 紧急 (影响基本导航)

- [ ] **测试 D455 地面过滤效果**: 重启导航后，确认 `/d455_front_restamped` 不再包含地面点，门前虚假障碍消失
- [ ] **实验 2: RotateToGoal 参数调优**: 修改 `slowing_factor: 5.0→2.0`、`lookahead_time: -1.0→1.0`，减少近目标旋转振荡

### P1 -- 重要

- [ ] **地图保存流程修复** (`map_saver_thread.py`): 当前保存的 3D 地图来自 FAST-LIO 原始输出（有漂移），应改为调用 `/pgo/save_maps` 服务获取 PGO 优化后的地图。计划文件已写: `/home/ros/.claude/plans/temporal-painting-ladybug.md`
- [ ] **NavFn 改用 A***: `use_astar: false→true`，提升全局规划效率

### P2 -- 优化

- [ ] **BaseObstacle.scale 调高**: 当前 0.02 太低，可能导致路径太贴近障碍物
- [ ] **D455 地面过滤升级 RANSAC**: 当前用简单高度裁剪，如遇到斜面/台阶场景不够用时，可升级为 RANSAC 平面拟合
- [ ] **Costmap 残留衰减验证**: 使用 `costmap_residual_probe.py` 实测动态障碍物移除后衰减时间是否符合预期
- [ ] **循环导航回归测试**: 使用 `nav_loop_regression.py` 在两点间循环导航，验证长时间运行稳定性

---

## 7. 配置文件快速参考

### FAST-LIO2 (`lio_megarover.yaml`)
```yaml
body_frame: lio_base          # 中间帧，非直接 base_footprint
world_frame: odom
r_il: [0, -1, 0, 1, 0, 0, 0, 0, 1]   # Rz(+90°) livox→body
t_il: [0.09, 0.0, 0.56]               # LiDAR 在 body 帧的位置
```

### Nav2 控制器 (`fastlio2_nav2_params.yaml`)
```yaml
max_vel_x: 0.35              # 最大前进速度
max_vel_theta: 0.6            # 最大旋转速度
RotateToGoal.slowing_factor: 5.0   # [待调优→2.0]
RotateToGoal.lookahead_time: -1.0  # [待调优→1.0]
BaseObstacle.scale: 0.02          # [待调优→0.05]
```

### pointcloud_relay 启动参数 (launch中)
```yaml
lidar_throttle_factor: 4     # LiDAR ~5Hz (从 20Hz)
camera_throttle_factor: 3    # D455 OctoMap ~2fps
costmap_throttle_factor: 2   # D455 costmap ~7fps
camera_max_range: 2.0        # OctoMap 路径用
```

---

## 8. 调试工具

```bash
# 启动导航调试日志记录器
ros2 run megarover3_navigation nav_debug_logger.py
# 日志保存到: ~/nav_debug_logs/nav_YYYYMMDD_HHMMSS.log

# 动态修改 Nav2 参数 (不需要重启)
ros2 param set /controller_server progress_checker.required_movement_radius 0.25

# 检查 D455 过滤效果
ros2 topic echo /d455_front_restamped --field header.frame_id
ros2 topic hz /d455_front_restamped

# 检查 costmap
ros2 topic echo /local_costmap/costmap_raw --once

# Costmap 残留衰减测量 (引入临时障碍物后观察衰减时间)
ros2 run megarover3_navigation costmap_residual_probe.py

# 两点循环导航回归测试 (自动在两个目标间往返)
ros2 run megarover3_navigation nav_loop_regression.py \
  --goal-a 2.82 -0.14 0.0 --goal-b -2.41 0.79 3.14 --cycles 6 --timeout 180

# D455 外参自动标定
python3 src/megarover3_ros2/megarover3_navigation/scripts/calibrate_d455.py --mode verify

# Bumper 手动复位
ros2 service call /bumper_safety/reset std_srvs/srv/Trigger "{}"
```

---

## 9. 构建说明

```bash
# 全量构建
cd /home/ros/Code/Demo8
colcon build --symlink-install

# 单包构建
colcon build --packages-select megarover3_navigation --symlink-install

# 注意: 使用 symlink-install 后, Python 脚本和 config 文件修改无需重新构建
# 但需要重启对应节点才能生效
```
