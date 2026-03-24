# D455 Point Cloud Fusion into OctoMap — 变更记录

## 目标
MID360 LiDAR 在 56cm 高度有近处低矮障碍物盲区，D455 相机在 16cm 高度可以看到。
将 D455 点云融合进 OctoMap 建图管线。

## 已完成的变更

### 1. 新建文件：`scripts/pointcloud_relay.py`
- **路径**: `src/megarover3_ros2/megarover3_navigation/scripts/pointcloud_relay.py`
- **功能**: 点云中继节点，订阅 LiDAR + D455 → 统一发布到 `/merged_cloud`
- LiDAR (`/patchworkpp/nonground`): 直通，不处理
- D455 (`/camera/d455_front/depth/color/points`): 节流(每3帧取1帧≈2fps)、体素降采样(0.05m)、距离过滤(4m)
- D455 时间戳替换为最新 LiDAR 时间戳（解决 TF 缓存不匹配问题，见下方"时钟问题"）

### 2. 修改：`CMakeLists.txt`
- **路径**: `src/megarover3_ros2/megarover3_navigation/CMakeLists.txt`
- 添加 `install(PROGRAMS scripts/pointcloud_relay.py DESTINATION lib/${PROJECT_NAME})`

### 3. 修改：`package.xml`
- **路径**: `src/megarover3_ros2/megarover3_navigation/package.xml`
- 添加 `<exec_depend>rclpy</exec_depend>` 和 `<exec_depend>sensor_msgs</exec_depend>`

### 4. 修改：`fastlio2_navigation.launch.py`
- **路径**: `src/megarover3_ros2/megarover3_navigation/launch/fastlio2_navigation.launch.py`
- 添加 `pointcloud_relay` 节点（在 OctoMap 之前）
- OctoMap `cloud_in` 从 `/patchworkpp/nonground` → `/merged_cloud`
- OctoMap `occupancy_min_z`: 0.0 → 0.05（排除 D455 地面点）
- OctoMap `incremental_2D_projection`: False → True（减轻 CPU 负担）
- 静态 TF `base_link→livox_frame`: 修正旋转和平移（见下方"90度修正"）
- D455 `camera_throttle_factor`: 3（~2fps）

### 5. 修改：`lio_megarover.yaml`（FASTLIO2 外参）
- **路径**: `src/FASTLIO2_ROS2/fastlio2/config/lio_megarover.yaml`
- `r_il`: `[1,0,0, 0,1,0, 0,0,1]` → `[0,-1,0, 1,0,0, 0,0,1]` (Rz(+90°))
- `t_il`: `[0, 0.09, 0.56]` → `[0.09, 0, 0.56]`

### 6. 修改：`fastlio2_nav.rviz`
- **路径**: `src/megarover3_ros2/megarover3_navigation/rviz/fastlio2_nav.rviz`
- 添加 RobotModel 显示项（之前缺失，无法看到 URDF 模型）

---

## 调试过程中发现并修复的问题

### 90度朝向修正
- **现象**: 机器人 TF 朝向和实际移动方向相差 90°
- **根因**: MID-360 LiDAR 接口朝后安装，其坐标系 (X=左, Y=后, Z=上) 与 body frame (X=前, Y=左, Z=上) 相差 90°
- **旧配置**: `r_il = Identity`, `t_il = [0, 0.09, 0.56]`（错误地把"前方9cm"放在了 Y 轴）
- **新配置**: `r_il = Rz(+90°)`, `t_il = [0.09, 0, 0.56]`
- 静态 TF 同步修改: `x=0.09, y=0, z=0.56, yaw=+π/2`
- **首次尝试 Rz(-90°) 结果 180° 反转，改为 Rz(+90°) 后正确**

### D455 时间戳与 TF 缓存不匹配
- **现象**: OctoMap 丢弃所有 D455 消息，报 "timestamp earlier than all data in transform cache"
- **根因**: D455 使用系统时钟，而 FASTLIO2 发布的 TF 使用 LiDAR 时间戳（滞后系统时钟约 1.5s）。D455 的时间戳对 TF 缓存来说"太新了"
- **测量数据**:
  - D455 stamp → ros_now 偏差: 0.032s（几乎同步系统时钟）
  - LiDAR stamp → ros_now 偏差: 1.472s（显著滞后）
- **修复**: relay 节点中将 D455 时间戳替换为最新 LiDAR 时间戳，使其落入 TF 缓存范围

### 重复节点清理
- **现象**: `base_link_to_livox` 和 `map_to_odom` 各出现 3 个实例
- **原因**: launch 文件被多次启动
- **处理**: 手动 kill 多余进程

---

## 已解决问题（2026-01-29 ~ 2026-01-30）

### 1. Nav2 costmap 无法使用 D455 数据 → 已修复
- **现象**: costmap (local + global) 丢弃所有 D455 消息，"timestamp earlier" 错误
- **根因**: costmap 直接订阅 D455 原始话题（系统时钟），但 FASTLIO2 TF 使用 LiDAR 时间戳（滞后 ~1.5s）
- **修复**: relay 节点新增 costmap 专用输出 `/d455_front_restamped`（NaN 清理 + 时间戳替换，不做降采样）
- **修改文件**:
  - `scripts/pointcloud_relay.py` — 新增 `costmap_output_topic`/`costmap_throttle_factor` 参数、`costmap_pub` 发布器、`_publish_costmap_cloud()` 方法
  - `config/fastlio2_nav2_params.yaml` — costmap `d455_front_cloud.topic` → `/d455_front_restamped`
  - `launch/fastlio2_navigation.launch.py` — relay 参数添加 costmap 配置
- **验证结果**: `/d455_front_restamped` ~7.5Hz，costmap 订阅者 2 个（local+global），无 TF 错误

### 2. 动态障碍物（行人等）在 costmap 中不消失 → 已修复
- **现象**: 行人经过后 costmap 障碍物持久残留
- **根因（双重问题）**:
  1. 原 `ObstacleLayer` 仅靠 2D raycasting 清除，覆盖不全
  2. costmap LiDAR 源 `/world_cloud`（frame_id=odom），STVL 以 odom 原点构建清除视锥体，导致射线清除完全失效
- **修复**:
  - costmap obstacle_layer 插件从 `nav2_costmap_2d::ObstacleLayer` 替换为 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`（STVL）
  - 安装 STVL: 从 `https://github.com/SteveMacenski/spatio_temporal_voxel_layer` humble 分支源码编译
  - costmap LiDAR 源从 `/world_cloud`（odom 坐标系）改为 `/patchworkpp/nonground`（base_link 坐标系），使 STVL 能正确定位传感器原点
  - 时间衰减: local_costmap `voxel_decay=2.0s`，global_costmap `voxel_decay=3.0s`
- **修改文件**:
  - `config/fastlio2_nav2_params.yaml` — local/global costmap 替换为 STVL 配置
- **注意**: OctoMap 的 `/map` 是持久化建图，不受 STVL 影响。动态避障完全由 costmap (STVL) 负责

---

## 当前系统架构（2026-01-30 更新）

### 完整数据流
```
硬件传感器
  MID360 ─→ /livox/lidar (~10Hz) + /livox/imu (~200Hz)
  D455   ─→ /camera/d455_front/depth/color/points (~8Hz)

FASTLIO2 (LiDAR-惯性融合定位)
  输入: /livox/lidar + /livox/imu
  输出: /body_cloud (base_link, ~8Hz)
        /lio_odom (odom→base_link, ~8Hz)
        /world_cloud (odom, ~8Hz, 无人订阅)
        TF: odom→base_link (6DOF, 含Z/Roll/Pitch漂移)
  内部: ikd-Tree 地图 (scan-to-map匹配定位用, 不导出)

planar_compensation (平面运动补偿, SLAM模式)
  输入: /lio_odom
  输出: TF: map→odom (锁定Z=常数, Roll=0, Pitch=0, 保留X/Y/Yaw)
  作用: 修正FASTLIO2 6DOF估计导致的机器人悬浮和抖动

Patchwork++ (地面分割)
  输入: /body_cloud
  输出: /patchworkpp/nonground (base_link, ~10Hz)

pointcloud_relay (点云中继/融合)
  输入: /patchworkpp/nonground + /camera/d455_front/depth/color/points
  输出:
    /merged_cloud (~5Hz) ─→ OctoMap
      LiDAR: 节流1/4, 直通
      D455:  节流1/3, 体素0.05m, 距离≤2m, 重打LiDAR时间戳
    /d455_front_restamped (~4Hz) ─→ Nav2 Costmap
      D455:  节流1/2, NaN清理, 重打LiDAR时间戳

OctoMap Server2 [可选: octomap:=true/false]
  输入: /merged_cloud
  输出: /map (OccupancyGrid, odom帧, ~6Hz)
  参数: resolution=0.10m, occupancy_min_z=0.15m
  用途: RViz可视化 + 保存2D栅格地图, 不参与Nav2避障

Nav2 Costmap (STVL - SpatioTemporalVoxelLayer)
  输入: /patchworkpp/nonground + /d455_front_restamped (直接订阅, ~40ms延迟)
  Local:  3x3m, voxel_decay=15s
  Global: 50x50m, voxel_decay=30s
  用途: Nav2路径规划和实时避障的核心数据源

Nav2 导航栈
  controller_server → planner_server → behavior_server → bt_navigator
  velocity_smoother → /rover_twist (电机速度指令)
```

### TF 树
```
map ──[planar_compensation]──→ odom ──[FASTLIO2]──→ base_link
                                                     ├── livox_frame (静态TF: x=0.09, z=0.56, yaw=90°)
                                                     ├── d455_front_link (URDF: y=0.035, z=0.16)
                                                     │    └── d455_front_depth_optical_frame
                                                     ├── d455_rear_link (URDF: y=-0.185, z=0.16)
                                                     └── base_footprint
```

### 各组件职责
| 组件 | 功能 | 输出 |
|------|------|------|
| FASTLIO2 | LiDAR-惯性定位 (iESKF) | TF (odom→base_link), /body_cloud, /lio_odom |
| planar_compensation | 平面运动约束 | TF (map→odom), 锁定Z/Roll/Pitch |
| Patchwork++ | 地面分割 | /patchworkpp/nonground |
| pointcloud_relay | 点云融合/节流/时间戳修正 | /merged_cloud, /d455_front_restamped |
| OctoMap | 3D→2D 持久化建图 (可选) | /map (持久化，不衰减) |
| Nav2 STVL | 实时避障层 | 障碍物 15-30 秒自动衰减 |

### 关键话题 (实测)
| 话题 | frame_id | 频率 | 延迟 | 用途 |
|------|----------|------|------|------|
| /body_cloud | base_link | ~8Hz | 40ms | FASTLIO2→Patchwork++ |
| /patchworkpp/nonground | base_link | ~10Hz | 41ms | costmap + relay 输入 |
| /lio_odom | odom | ~8Hz | 39ms | 补偿节点 + Nav2 定位 |
| /d455_front_restamped | d455_optical | ~4Hz | 95ms | costmap D455 源 |
| /merged_cloud | mixed | ~5Hz | 58ms | OctoMap 输入 |
| /map (OctoMap) | odom | ~6Hz | 173ms | RViz 可视化 |
| /world_cloud | odom | ~8Hz | — | FASTLIO2 累积地图 (未使用) |

### 两套地图的关系
| | FASTLIO2 ikd-Tree | OctoMap |
|---|---|---|
| 内容 | 全部LiDAR原始点(含地面) | 过滤后障碍物(无地面, 含D455) |
| 用途 | scan-to-map定位 | RViz可视化 + 保存导航地图 |
| 传感器 | 仅MID360 | MID360 + D455 |
| Nav2使用 | 否(仅提供TF) | 否(Nav2用STVL实时数据) |

---

## 待解决问题

### 1. D455 与 MID360 障碍物位置不一致 → 已修复
- **现象**: D455 检测到的障碍物和 MID360 检测到的障碍物在 costmap/RViz 中位置有偏差（Y 轴约 10cm）
- **根因**: D455 外参标定（URDF 中 `d455_front_mount_y`）与实际安装有 ~10cm 误差
- **诊断过程**:
  1. 编写自动标定脚本 `scripts/calibration_diagnostic.py`
  2. 在机器人前方放置平板，同时采集 `/body_cloud`（LiDAR, base_link 帧）和 `/d455_front_restamped`（D455, 光学帧）
  3. 脚本将 D455 点云通过 TF 变换到 base_link 帧，过滤墙面区域（Z=0.10~1.5m, |X|<1.0m）
  4. 对两个点云分别做 RANSAC 平面拟合，比较墙面距离
  5. 三次测量结果（不同距离）：
     | 距离 | LiDAR 墙距 | D455 墙距 | Y 偏移 | 法向量夹角 |
     |------|-----------|----------|--------|-----------|
     | ~2.5m | 2.543m | 2.727m | +0.184m | 1.87° |
     | ~1.1m | 1.115m | 1.225m | +0.110m | 2.35° |
     | ~0.6m | 0.644m | 0.739m | +0.096m | 2.04° |
  6. 近距离偏移稳定在 ~+0.10m（D455 看到墙更远 10cm）
  7. 远距离偏移增大至 +0.18m，含约 3% D455 深度尺度误差
- **修正**: `calibration_offsets.xacro` 中 `d455_front_mount_y`: 0.135 → 0.035（减小 0.10m）
- **注意**: 尝试修改 MID360 的 `t_il` 会影响 FASTLIO2 EKF 内部状态估计，导致里程计失效，不可行
- **验证结果（修正后）**:
  - 墙距差异: +0.096m → **-0.003m**（从 10cm 偏差缩小到 3mm）
  - Y 峰值: LiDAR=0.975m, D455=0.975m（完全重合）
  - 法向量夹角: 2.47°（旋转标定正确）
- **修改文件**:
  - `src/megarover3_ros2/megarover_description/urdf/calibration_offsets.xacro` — `d455_front_mount_y`: 0.135 → 0.035
- **标定值** (`calibration_offsets.xacro`):
  - D455 front: xyz=(0.0, **0.035**, 0.16), rpy=(π, 0, π/2)
  - 含义: 居中，前方 3.5cm（标定后），高 16cm，倒置安装，朝前
- **Z 轴（高度）标定验证**:
  - 使用 2m 高板子进行测量，通过 Method 7（板顶边缘对比）验证
  - LiDAR 可见高度: Z = 0.39 ~ 1.41m（MID360 在 56cm 高度，下视角有限）
  - D455 可见高度: Z = 0.10 ~ 1.50m（D455 在 16cm 高度，视野更广）
  - 重叠区域: Z = 0.39 ~ 1.41m（充分重叠）
  - **板顶边缘 (P95) 差异: +0.010m（1cm）→ Z 轴标定准确，无需修改**
  - 板底差异 -0.22m 为正常 FOV 几何差异（D455 更低，看到更低位置）
  - 重叠区均值差 +0.09m 为点密度分布差异，非标定误差
- **最终标定结论**:
  - X 轴: 无需修改（偏差 < 0.04m）
  - Y 轴: 已修正（mount_y: 0.135 → 0.035，验证 diff = -0.003m）
  - Z 轴: 无需修改（mount_z = 0.16，板顶 diff = +0.010m）
  - 旋转: 无需修改（法向量夹角 < 2.5°）
- **诊断工具**: `scripts/calibration_diagnostic.py`
  - 订阅: `/body_cloud`（RELIABLE QoS）、`/d455_front_restamped`（BEST_EFFORT QoS）
  - 采集 8 秒，自动过滤离群点（10m）、D455 降采样（5000 pts/frame）
  - 分析方法（7 种）:
    1. 质心对比
    2. RANSAC 平面拟合（全局）
    3. 逐轴统计
    4. 墙面 inlier 对比
    5. Y 轴直方图峰值
    6. 墙面过滤分析 → **Y 轴标定用 `WALL DISTANCE` 结果**
    7. 板顶边缘 Z 对比 → **Z 轴标定用 `Z OFFSET (top edge)` 结果**
  - 使用方法: 前方放平板（≥1m 高）→ 运行脚本 → 查看 Method 6 和 Method 7 结果

---

## D455 外参标定操作流程

### 前提条件
- 导航栈正在运行（SLAM 模式），`/body_cloud` 和 `/d455_front_restamped` 话题活跃
- 机器人保持静止

### 步骤
1. **放置标定板**: 在机器人正前方 0.5~1.5m 处放置平坦的大板（宽 ≥ 60cm，高度从地面到 ≥ 1m）
2. **运行诊断脚本**:
   ```bash
   source install/setup.bash
   python3 src/megarover3_ros2/megarover3_navigation/scripts/calibration_diagnostic.py
   ```
3. **查看结果**: 重点看 **Method 6** 的 `WALL DISTANCE` 输出
   - `diff` 值 = D455 墙距 − LiDAR 墙距
   - 正值 → D455 看到墙更远 → 需减小 `d455_front_mount_y`
   - 负值 → D455 看到墙更近 → 需增大 `d455_front_mount_y`
   - 目标: `diff` 绝对值 < 0.01m
4. **修改标定文件**:
   ```
   src/megarover3_ros2/megarover_description/urdf/calibration_offsets.xacro
   ```
   调整 `d455_front_mount_y` 值（减去 diff 值）
5. **重建并重启**:
   ```bash
   colcon build --packages-select megarover_description --symlink-install
   # 重启导航栈
   ```
6. **再次运行脚本验证**, 重复直到 diff < 0.01m

### 注意事项
- **不要修改** FASTLIO2 的 `t_il` 参数 — 它是 EKF 紧耦合参数，改动会导致里程计失效
- 板子需要足够大，使 D455（16cm 高度）和 MID360（56cm 高度）都能看到同一面
- 板子太小或太远时，D455 会主要看到地面而非板子（脚本会警告 "NOT a wall"）
- 建议在 0.5~1.0m 距离测量（D455 近距离深度精度更高）
- 多次测量取平均值更可靠

---

### 2. 机器人在 RViz 中悬浮和抖动 → 已修复
- **现象**: 机器人移动时在 RViz 中悬浮（离开地面）且静止时抖动
- **根因**: FASTLIO2 是 6DOF 估计器，没有平面运动约束。IMU 噪声导致 Z/Roll/Pitch 持续波动
  - 实测 Pitch 漂移: ~1.1°, Z 漂移: ~5mm
- **修复**: 新建 `planar_compensation.py` 节点
  - 订阅 `/lio_odom`，发布 TF: map→odom
  - 数学: `Q_comp = Q_desired(0, 0, yaw) × Q_odom⁻¹`，Z 锁定为启动时参考值
  - 替换 SLAM 模式下的静态 map→odom identity TF
- **修改文件**:
  - 新建 `scripts/planar_compensation.py`
  - `CMakeLists.txt` — 添加 `planar_compensation.py` 安装
  - `launch/fastlio2_navigation.launch.py` — 替换静态 TF 为补偿节点
- **验证结果**:
  - map→base_link: Z=0.001m (锁定), Roll≈0°, Pitch≈0° (锁定), Yaw 保留
  - odom→base_link (原始): Z=-0.005m, Pitch=-1.13° (漂移)

### 3. OctoMap 性能优化 → 已修复
- **现象**: OctoMap 延迟 735ms，占端到端延迟 98%；merged_cloud 输入 45Hz 导致队列溢出
- **根因**: LiDAR 无节流直通 (~20Hz) + D455 (~3Hz) = 过高输入频率；OctoMap 0.05m 分辨率处理慢
- **修复**:
  1. `pointcloud_relay.py` 新增 `lidar_throttle_factor` 参数 (默认4, 20Hz→5Hz)
  2. OctoMap `resolution`: 0.05 → 0.10m (计算量 -75%)
  3. OctoMap `occupancy_min_z`: 0.05 → 0.15m (防止 Pitch 漂移导致地面误检)
  4. OctoMap 改为可选: `octomap:=true/false` launch 参数
  5. D455 `camera_max_range`: 4.0 → 2.0m (提高可靠性)
  6. Costmap D455 `obstacle_range`: local 2.5→2.0m, global 3.0→2.0m
- **性能对比**:
  | 指标 | 优化前 | 优化后 |
  |------|--------|--------|
  | merged_cloud 频率 | 45 Hz | 5 Hz |
  | OctoMap 延迟 | 735ms | 173ms (-78%) |
  | OctoMap /map 频率 | 2.8 Hz | 6.7 Hz |
  | 队列丢帧率 | ~42/s | ~1.4/s |
- **关键发现**: Nav2 的 local/global costmap 都不依赖 OctoMap `/map`，直接用 STVL 订阅实时点云 (~40ms 延迟)。OctoMap 仅用于 RViz 可视化和地图保存

### 4. STVL 参数调整为 GitHub 推荐值 → 已修复
- **现象**: STVL 衰减时间过短 (local 2s, global 3s)，障碍物几乎立刻消失
- **修复**: 参照 [GitHub 推荐](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
  - `voxel_decay`: local 2→15s, global 3→30s (推荐 local 5-15s, global 15-45s)
  - `track_unknown_space`: false→true
  - `publish_voxel_map`: false→true (3D 体素可视化)
- **修改文件**: `config/fastlio2_nav2_params.yaml`

---

## 待解决问题

### 1. 建图 + 导航模式验证
- **状态**: SLAM 模式已调通，需要实际建图后切换到 nav 模式验证
- **步骤**:
  1. SLAM 模式遥控建图
  2. `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map` 保存地图
  3. `mode:=nav map:=/path/to/map.yaml` 启动导航
  4. 验证 costmap STVL 动态障碍物衰减效果
  5. 验证自主导航路径规划和避障

### 2. OctoMap 动态障碍物残留（低优先级）
- **现象**: OctoMap `/map` 中行人等动态障碍物会残留
- **影响**: 仅影响 RViz 显示，不影响 costmap 避障（costmap 不订阅 `/map`）
- **可选方案**: 调整 `sensor_model/miss` (0.4→0.48) 和 `sensor_model/hit` (0.7→0.55) 加速清除

### 3. GPU 加速（低优先级）
- CUDA 12.6 可用，cupy 未安装
- 如需要: `pip install cupy-cuda12x`

---

## 构建与测试命令
```bash
# 构建（含 STVL）
colcon build --packages-select openvdb_vendor spatio_temporal_voxel_layer megarover3_navigation --symlink-install

# 启动
source install/setup.bash
# Terminal 1: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4
# Terminal 2: ros2 launch livox_ros_driver2 msg_MID360_launch.py
# Terminal 3: ros2 launch realsense2_camera rs_launch.py camera_name:=d455_front serial_no:="'239222302509'" depth_module.depth_profile:=640x480x15 pointcloud.enable:=true publish_tf:=false
# Terminal 4: ros2 launch megarover3_navigation fastlio2_navigation.launch.py mode:=slam
# Terminal 5: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=rover_twist

# 验证
ros2 topic hz /d455_front_restamped   # 预期 ~7.5Hz
ros2 topic hz /merged_cloud           # 预期 ~15Hz
ros2 topic info /d455_front_restamped  # subscription count = 2 (local+global costmap)
```
