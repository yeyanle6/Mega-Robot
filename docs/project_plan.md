# 项目规划

## 项目目标
- 基于 Jetson AGX Orin（JetPack 6.2 + ROS 2 Humble）集成 Megarover3 底盘与 Livox MID360，完成室内自主移动。
- 采用 RTAB-Map 构建 3D/2D 地图，并使用 Nav2 导航栈实现建图后导航、动态避障与点到点移动。
- 保持系统可扩展，后续可接入更多传感器（如 RealSense）或仿真环境。

## 当前状态
- 工作空间已清理：移除 FAST-LIO 及相关临时脚本（2025-xx-xx）。
- 自有包保留：`t_robot_bringup`、`t_robot_slam`（已完成 RTAB-Map 流程重构的主体部分）。
- 第三方包：`megarover3_ros2`（含 URDF/驱动）、`livox_ros_driver2`、`realsense-ros`（可选）、`vs_rover_options_description`。
- 核心依赖（`rtabmap_ros`、`robot_localization`、Nav2 套件等）已安装并通过 `colcon build` 验证。
- 话题命名、静态 TF、点云预处理与 EKF 状态估计全部贯通，形成可用于建图的端到端数据流。

## 先决条件与依赖
- 系统包安装（Jetson 上执行）：
  ```bash
  sudo apt update
  sudo apt install \
    ros-humble-rtabmap-ros \
    ros-humble-robot-localization \
    ros-humble-pcl-ros \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-nav2-bringup \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-rviz-plugins \
    ros-humble-diagnostic-updater \
    ros-humble-rqt-robot-monitor
  ```
- 可选工具：`ros-humble-navigation2`, `ros-humble-twist-mux`, `ros-humble-foxglove-bridge`。
- 校对硬件参数：MID360 安装位姿、底盘里程计比例、IMU 对齐等。

## 实施阶段

### 阶段 0：基线确认（已完成）
- ✅ 清除 FAST-LIO 目录与残留依赖，更新 `t_robot_bringup`、`t_robot_slam` 的 `package.xml`。
- ✅ 检查第三方包中与 FAST-LIO 相关的 launch（如 `mega3_fastlio.launch.py`），标记为弃用或建立替代脚本。
- ✅ 整理 README/脚本中遗留的 FAST-LIO 说明。

### 阶段 1：机器人模型与 TF
- ✅ 审阅 `megarover_description/urdf/mega3.xacro`，确认 MID360 已挂载于 `top_plate_link` 并输出 `mid360_base`/`mid360_lidar` 帧。
- ✅ 建立 `t_robot_bringup` 配置骨架（`config/static_tf.yaml`、`config/filters.yaml` 等）和分层 launch 模板。
- ✅ 校准 MID360 实际安装偏移并更新 URDF/静态 TF 配置。
- ✅ 准备 RViz 校准配置（查看 `mid360_lidar` 点云、底盘姿态）。

### 阶段 2：基础 Bringup 与状态估计
- ✅ 搭建 bringup 框架：新增 `bringup.launch.py`，并按 base/sensors/state_estimation 分层。
- ✅ 提供 `params/ekf.yaml`、`params/livox_driver.yaml`、`config/twist_mux.yaml` 初稿。
- ✅ 在 `base.launch.py`、`sensors.launch.py` 中填充 Megarover3/MID360 实际节点。
- ✅ 在 `state_estimation.launch.py` 中增补静态 TF/诊断节点，并做实车验证。
- ✅ **TF架构修复（2025-01-10）**：
  - **问题诊断**：
    1. EKF参数名错误（`odometry0` → `odom0`）导致无法订阅轮速计数据
    2. IMU QoS不匹配（发布者RELIABLE vs 订阅者BEST_EFFORT）
    3. TF树断裂：`base_footprint` 未连接到 `odom` frame
    4. 90度旋转闪烁：EKF和pub_odom同时发布TF导致冲突
  - **最终解决方案（符合RTAB-Map标准架构）**：
    - **pub_odom**：发布 `odom → base_footprint` TF + `/odom` 消息（100Hz）
    - **imu_relay**：QoS改为BEST_EFFORT（匹配robot_localization预期）
    - **EKF**：`publish_tf: false`，只输出 `/odometry/filtered` 消息（50Hz）
    - **TF树结构**：`map → odom → base_footprint → base_link → sensors`
  - **配置文件更新**：
    - `ekf.yaml`: 修正参数名、禁用TF发布、增加sensor_timeout
    - `pub_odom.cpp`: 恢复TF发布功能
    - `imu_relay.py`: 输出QoS改为BEST_EFFORT
  - **验证结果**：TF树完整、无闪烁、RTAB-Map可使用融合后的 `/odometry/filtered`

### 阶段 3：RTAB-Map 建图流程
- ✅ 新增 `mapping.launch.py`、`localization.launch.py` 骨架，并支持可选 bringup include。
- ✅ 提供 `params/rtabmap.yaml` 基础模板。
- ✅ 整合点云预处理（voxel filter、ground filter）并接入 MID360 话题。
-   - 实测：输入约 2 万点 → 输出障碍物 ~5.2k 点、地面 ~1.9k 点，延迟 ~50ms、频率 ~20Hz，持续稳定运行。
- ✅ 更新脚本（`mapping_recorder.py` 等）以适配 RTAB-Map 输出并统一话题命名。
- ✅ **RTAB-Map 3D LiDAR 配置优化与验证（2025-01-10）**：
  - **同步问题修复**：
    1. 初始问题：精确时间同步（exact sync）要求所有输入时间戳完全一致，但点云和里程计时间戳有 ~2 秒差异
    2. 订阅了不存在的 `/odom_info` 话题导致同步失败
    3. 解决方案：启用近似同步（`approx_sync: true`），关闭 `subscribe_odom_info`，暂时禁用 IMU 订阅
  - **3D LiDAR-only 参数配置**：
    - **配准策略**：`Reg/Strategy: 1`（纯 ICP，禁用视觉特征）
    - **ICP 优化**：点到平面 ICP、体素大小 0.05m、使用 libpointmatcher 加速
    - **回环检测**：禁用 RGB-D，启用空间距离检测（`RGBD/ProximityBySpace: true`）
    - **内存优化**：禁用图像保存、短期记忆 30 帧（适配 Jetson AGX Orin）
    - **2D 地图生成**：从 3D 点云投影 2D 栅格地图（5cm 分辨率，用于 Nav2）
  - **配置文件更新**：
    - `rtabmap.yaml`: 新增 60+ 参数用于 3D LiDAR SLAM
    - `mapping.launch.py`: 更新订阅配置和 remap 设置
  - **验证结果（静止测试）**：
    - ✅ RTAB-Map 节点正常启动，订阅 `/odometry/filtered` + `/mid360/points_filtered`
    - ✅ 近似同步成功，不再报 "Did not receive data" 错误
    - ✅ 处理性能稳定：RTAB-Map 处理 ~40-50ms，总延迟 ~200ms
    - ✅ 关键帧频率：1Hz（每秒创建 1 个地图节点）
    - ✅ 静止状态创建 68 个节点（68 秒测试），内存占用 ~10MB
    - ✅ `/info`、`/mapData`、`/cloud_map` 等话题正常发布
    - ⚠️  "Map info is empty" 警告正常（机器人静止未移动，无法生成 2D 栅格地图）
  - **待验证项（需要实车移动测试）**：
    - [ ] 移动建图：机器人移动时地图扩展、距离累积
    - [ ] 回环检测：返回起点时触发 Loop Closure
    - [ ] 2D 地图生成：移动后生成可用于 Nav2 的 occupancy grid
    - [ ] 长时间稳定性：连续建图 5-10 分钟无崩溃
- [ ] 地图保存/转换流程与 Nav2 接口联调。
- 现场验证：
  - 实时建图，观察回环效果。
  - rosbag 回放验证参数稳定。

### 阶段 4：导航与任务执行
- 新建 `navigation` 模块（可沿用 `t_robot_slam` 或拆出 `t_robot_navigation`）：
  - `nav2_bringup.launch.py`：加载地图、启动 `nav2_lifecycle_manager`、行为树。
  - ✅ Costmap 配置：使用 `pointcloud_layer`/`voxel_layer` 并接入 `/cloud/obstacles`，完成 3D 点云适配，后续继续调优动态避障。
  - 局部/全局规划器选择：初期用 `nav2_smac_planner + dwb_controller`，后续按需求切换 `mppi`/`teb`。
  - 任务接口：导航目标发布脚本（或整合 Nav2 Simple Commander）。
- **RViz 导航可视化配置**（保存为 `t_robot_slam/rviz/navigation.rviz`）：
  - **固定坐标系**：`map`
  - **必要显示项**：
    1. **TF**：机器人位姿和坐标系
    2. **Map** (`/map`)：静态地图（灰度）
    3. **Map** (`/global_costmap/costmap`)：全局代价地图（彩色）
    4. **Map** (`/local_costmap/costmap`)：局部代价地图（彩色）
    5. **Path** (`/plan`)：全局规划路径（绿色）
    6. **Path** (`/local_plan`)：局部规划路径（红色）
    7. **PointCloud2** (`/cloud/obstacles`)：实时障碍物点云
    8. **Polygon** (`/local_costmap/footprint`)：机器人轮廓
    9. **PoseStamped** (`/goal_pose`)：导航目标点（绿色箭头）
  - **可选显示项**：
    - **Path** (`/waypoints`)：多点任务路径
    - **MarkerArray** (`/nav2_markers`)：导航行为树状态
    - **PointCloud2** (`/cloud_map`)：RTAB-Map 3D 地图（定位模式）

### 阶段 5：系统集成与验收
- 集成流程：`launch` 级联——`bringup` → `mapping`/`navigation`，通过 launch 参数切换模式。
- 制定测试用例：
  - 静止测试：传感器话题稳定、TF 完整。
  - 建图测试：行驶指定路线，检查地图质量与回环。
  - 导航测试：从命名点 A→B→C，评估避障与重规划。
- 文档与运维：
  - 更新 README、操作手册、常见问题。
  - 制作系统备份脚本（参数、地图、rosbag）。

## 里程碑
- M0：依赖安装完成，`colcon build` 通过（✅ 达成）。
- M1：Bringup 可以在实车运行，TF/里程计稳定（✅ 达成）。
- M2：RTAB-Map 实车建图成功，产出可复用地图（🔄 进行中）：
  - ✅ RTAB-Map 节点成功启动并接收数据
  - ✅ 3D LiDAR-only 参数配置完成
  - ✅ 静止状态系统稳定性验证通过
  - ⏳ 等待实车移动测试（建图、回环检测）
  - ⏳ 地图保存/转换流程与 Nav2 接口联调
- M3：Nav2 导航稳定完成多点任务、具备动态避障（待验证）。
- M4：整理文档、测试报告、发布初版系统映像（待启动）。

## RViz 可视化配置参考

### 建图模式（mapping.rviz）
**固定坐标系**：`map`

**核心显示配置**：
| 显示类型 | 话题 | 作用 | 颜色/样式 |
|---------|------|------|----------|
| TF | - | 机器人运动轨迹和坐标系 | 默认 |
| PointCloud2 | `/cloud_map` | RTAB-Map 3D 地图点云 | 白色/强度映射 |
| Map | `/map` | 2D 栅格地图（Nav2 用） | 灰度 |
| Path | `/mapPath` | 建图路径轨迹 | 绿色 |
| Odometry | `/odometry/filtered` | 融合里程计轨迹 | 蓝色箭头 |
| PointCloud2 | `/mid360/points_filtered` | 实时点云输入（可选） | 彩色 |
| PointCloud2 | `/cloud/obstacles` | 障碍物点云（可选） | 红色 |
| PointCloud2 | `/cloud/ground` | 地面点云（可选） | 蓝色 |
| MarkerArray | `/rtabmap/labels` | 地图节点标签（可选） | 文本 |

**配置要点**：
- PointCloud2 的 `Size (m)` 设为 0.01-0.05（视点云密度调整）
- Map 的 `Color Scheme` 选择 `map` 或 `costmap`
- Path 的 `Line Width` 设为 0.05-0.1

### 导航模式（navigation.rviz）
**固定坐标系**：`map`

**核心显示配置**：
| 显示类型 | 话题 | 作用 | 颜色/样式 |
|---------|------|------|----------|
| TF | - | 机器人位姿和坐标系 | 默认 |
| Map | `/map` | 静态地图 | 灰度 |
| Map | `/global_costmap/costmap` | 全局代价地图 | costmap 彩色 |
| Map | `/local_costmap/costmap` | 局部代价地图 | costmap 彩色 |
| Path | `/plan` | 全局规划路径 | 绿色 |
| Path | `/local_plan` | 局部规划路径 | 红色 |
| PointCloud2 | `/cloud/obstacles` | 实时障碍物点云 | 红色 |
| Polygon | `/local_costmap/footprint` | 机器人轮廓 | 黄色 |
| PoseStamped | `/goal_pose` | 导航目标点 | 绿色箭头 |

**交互工具**：
- 启用 **2D Pose Estimate**：手动设置机器人初始位姿（定位模式）
- 启用 **2D Goal Pose**：点击设置导航目标点
- 启用 **Publish Point**：发布自定义点位（可选）

### 通用优化建议
1. **性能优化**（Jetson 平台）：
   - 关闭不必要的可选显示项
   - 点云 `Decay Time` 设为 0（避免累积）
   - Map 更新频率限制到 1-2Hz
2. **视角设置**：
   - **建图**：Top-Down 俯视图，便于观察 2D 地图生成
   - **导航**：Third Person 跟随视角，便于观察路径规划
3. **保存配置**：
   - File → Save Config As → `t_robot_slam/rviz/mapping.rviz`
   - Launch 文件中自动加载：`rviz_config_file` 参数

## 风险与注意事项
- 传感器时钟与 TF 漂移是导航稳定度关键，必要时引入硬件时间同步或 `chrony`。
- MID360 点云较密，需在 Jetson 上控制滤波计算量，避免 RTAB-Map 过载。
- Nav2 costmap 对 3D 点云敏感，需优化地面去除/高度阈值，否则可能出现虚假障碍。
- 建图与导航模式的参数差异大，建议使用独立参数文件并通过 launch 参数切换。
- 保留 realsense-ros 作为可选项，但若短期内不用可推迟集成。
- **RViz 在 Jetson 上可能占用较多资源**，建议远程显示或降低更新频率。

## 当前待办（优先级）

### 立即执行（RTAB-Map 移动建图测试）
1. **实车移动建图测试**（`test_mapping.sh` 已运行，系统就绪）：
   - 使用键盘控制工具移动机器人：`ros2 run teleop_twist_keyboard teleop_twist_keyboard`
   - 建图路线：直线前进 2-3m → 旋转 90° → 前进 → 返回起点（测试回环检测）
   - 监控指标：
     - `Memory/Distance_travelled` 应持续增长（验证运动累积）
     - `Loop/Id` > 0（验证回环检测触发）
     - `/map` 话题发布 2D 栅格地图（验证 Nav2 地图生成）
   - **可视化配置**（RViz2 实时监控建图过程）：
     - 启动命令：`ros2 run rviz2 rviz2`
     - **固定坐标系**：设置为 `map`
     - **必要显示项**：
       1. **TF**：显示机器人运动轨迹和坐标变换关系
       2. **PointCloud2** (`/cloud_map`)：3D 地图点云（白色/彩色）
       3. **Map** (`/map`)：2D 栅格地图，用于 Nav2 导航
       4. **Path** (`/mapPath`)：建图路径轨迹（绿色线条）
       5. **Odometry** (`/odometry/filtered`)：融合后的里程计轨迹
     - **可选显示项**：
       - **PointCloud2** (`/mid360/points_filtered`)：实时点云输入
       - **PointCloud2** (`/cloud/obstacles`)：障碍物点云（红色）
       - **PointCloud2** (`/cloud/ground`)：地面点云（蓝色）
       - **MarkerArray** (`/rtabmap/labels`)：地图节点标签
     - **参考配置文件**：保存为 `t_robot_slam/rviz/mapping.rviz` 供后续使用
2. **地图保存与验证**：
   - 建图完成后导出数据库：`~/.ros/rtabmap.db`
   - 测试地图加载：`ros2 launch t_robot_slam localization.launch.py`
   - 验证 2D 栅格地图可用于 Nav2

### 后续任务
3. 完成 Nav2 地图加载与导航联调（阶段 3 最后一项）。
4. 根据实测结果更新导航启动脚本和参数，完成 Nav2 多点任务验证（阶段 4 里程碑）。
5. 整理测试日志、地图数据库与步骤，更新 README/运维文档中的操作建议。
6. 评估后续扩展（RealSense/仿真）所需硬件与软件资源，制定集成计划。

### 已完成的基础测试
- ✅ Bringup 系统测试：TF 树完整、里程计稳定、传感器数据流正常
- ✅ RTAB-Map 静止测试：节点启动、数据同步、性能验证通过
