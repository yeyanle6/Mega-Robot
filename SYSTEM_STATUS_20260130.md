# MegaRover3 系统状态记录
**日期**: 2026-01-30
**时间**: 16:58

---

## ✅ 正常工作的部分

### 1. 传感器系统
- **底盘 (Micro-ROS)**: ✅ 正常
  - `/megarover` 节点运行中
  - 发布 `/rover_twist`, `/rover_odo` 话题

- **MID360 激光雷达**: ✅ 正常
  - `/livox_lidar_publisher` 运行中
  - 点云数据正常发布

- **D455 前方相机**: ✅ 正常
  - `/camera/d455_front` 节点运行中
  - 深度点云 `/camera/d455_front/depth/color/points` 正常

### 2. SLAM 建图系统
- **FAST-LIO2**: ✅ 正常
  - `/fastlio2/fastlio2_lio` 节点运行中
  - LIO 里程计 `/fastlio2/lio_odom` 正常发布
  - 点云数据 `/fastlio2/body_cloud`, `/fastlio2/world_cloud` 正常
  - TF: `odom → base_link` 正常发布

- **地图保存功能**: ✅ 已修复
  - 异步保存实现完成
  - 正确保存顺序：2D 地图（SLAM 运行） → 停止 SLAM → 3D 地图（SLAM 停止）
  - 已成功保存地图：`maps/my_pgo_map.*`, `maps/1632.*`

### 3. TF 树状态
- ✅ **基础 TF 树正常**
  - `base_footprint → base_link → 传感器坐标系` 完整
  - `odom → base_link` 由 FastLIO2 发布（10 Hz）
  - 无重复发布器（已清理）

### 4. 控制面板
- ✅ PyQt5 控制面板运行正常
  - 组件启动/停止功能正常
  - 状态监控正常
  - 日志显示正常

---

## ❌ 存在问题的部分

### 1. 导航系统 (PGO Navigation)

#### 问题 1: 缺少 `map` 坐标系
**现象**:
```
[planner_server] Timed out waiting for transform from base_link to map
tf error: Invalid frame ID "map" passed to canTransform argument - frame does not exist
```

**原因**:
- Localizer 节点正在运行：`/localizer/localizer_node` ✅
- 但 Localizer **未初始化**，不发布 `map → odom` 变换
- 当前只有 `odom → base_link`，缺少 `map → odom`

**影响**:
- Nav2 无法工作（需要 map 坐标系）
- RViz2 大量错误：`Message Filter dropping message: frame 'odom'... queue is full`

#### 问题 2: 2D 地图未加载
**现象**:
- `/map` 话题不存在或无数据
- RViz2 看不到 2D 栅格地图

**检测到的节点**:
- `/lifecycle_manager_map` ✅ 存在
- `/map_server` ❓ 可能未正确启动或生命周期未激活

**影响**:
- 无法在 RViz2 中看到地图进行初始化
- 无法使用 "2D Pose Estimate" 工具

#### 问题 3: 初始化流程缺失
**问题链**:
1. Localizer 需要初始化才能发布 `map → odom`
2. 初始化需要调用 `/localizer/relocalize` 服务
3. Relocalize 服务需要参数：
   - `pcd_path`: 3D 点云地图路径
   - `x, y, z, yaw, pitch, roll`: 初始位姿
4. 但用户无法在 RViz2 中看到地图，不知道初始位置
5. 形成死锁

**当前运行的导航节点**:
```
/localizer/localizer_node       # 定位器（未初始化）
/lifecycle_manager_map          # 地图生命周期管理器
/controller_server              # 控制器
/local_costmap/local_costmap    # 局部代价地图
/pointcloud_relay               # 点云中继
```

**缺失或未正常工作的节点**:
```
/map_server          # 2D 地图服务器（可能未激活）
/planner_server      # 全局规划器
/behavior_server     # 行为服务器
/bt_navigator        # 导航器
/global_costmap      # 全局代价地图
```

---

## 🔧 待解决任务

### 优先级 1: 修复导航初始化流程

**方案 A: 盲初始化（推荐用于测试）**
1. 假设机器人在地图原点
2. 调用 relocalize 服务：
   ```bash
   ros2 service call /localizer/relocalize interface/srv/Relocalize "{
     pcd_path: '/path/to/1632.pcd',
     x: 0.0, y: 0.0, z: 0.0,
     yaw: 0.0, pitch: 0.0, roll: 0.0
   }"
   ```
3. 验证 `map → base_link` TF 是否出现
4. 如果位置错误，在 RViz2 中用 "2D Pose Estimate" 修正

**方案 B: 先建图确认位置**
1. 停止导航模式
2. 启动 SLAM 建图模式
3. 在 RViz2 中查看机器人当前位置坐标
4. 记录坐标后重启导航
5. 使用记录的坐标初始化

**方案 C: 修改启动文件支持自动初始化**
1. 在 launch 文件中添加默认初始位姿参数
2. 导航启动时自动调用 relocalize 服务
3. 需要修改：`fastlio2_pgo_navigation.launch.py`

### 优先级 2: 检查 map_server 状态

**检查项**:
1. map_server 节点是否存在
2. 生命周期状态是否为 `active`
3. `/map` 话题是否正常发布
4. 地图文件路径是否正确

**命令**:
```bash
# 检查 map_server 节点
ros2 node list | grep map_server

# 检查生命周期状态
ros2 lifecycle get /map_server

# 检查 /map 话题
ros2 topic echo /map --once

# 如果需要手动激活
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### 优先级 3: 验证地图文件

**已确认的地图文件**:
- 2D 地图: `/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/1632.yaml`
- 3D 地图: `/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/1632.pcd`

**验证**:
```bash
# 检查文件是否存在
ls -lh /home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/1632.*

# 检查 YAML 内容
cat /home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/1632.yaml
```

---

## 📊 系统诊断数据

### 当前节点数
```
总节点数: 13
- 传感器节点: 4 (底盘, 激光雷达, 相机, joint_state)
- SLAM 节点: 1 (fastlio2_lio)
- 导航节点: 4 (localizer, controller_server, local_costmap, lifecycle_manager)
- 工具节点: 3 (pointcloud_relay, tf2_echo, robot_state_publisher)
- 其他: 1 (base_link_to_livox)
```

### 重复节点（已发现）
```
/megarover x2 ← 需要清理
```

### 关键话题状态
```
✅ /fastlio2/lio_odom        # LIO 里程计
✅ /fastlio2/body_cloud      # 点云
✅ /rover_twist              # 底盘速度
✅ /camera/.../depth/color/points  # 深度点云
❌ /map                      # 2D 地图（缺失或无数据）
❓ /pgo/pgo_pose            # PGO 位姿（未运行）
```

### TF 树状态
```
当前 TF 树:
  odom
   └─ base_link
       ├─ livox_frame (mid360)
       ├─ d455_front_* (相机坐标系)
       └─ left/right_wheel_1

缺失:
  map (应由 localizer 发布)
   └─ odom
```

---

## 🎯 下一步行动建议

### 立即执行
1. **清理重复节点**: 检查并清理 `/megarover` 重复实例
2. **检查 map_server**: 验证是否正确启动和激活
3. **初始化 localizer**: 使用盲初始化方案测试

### 后续改进
1. **优化启动流程**: 自动初始化 localizer
2. **添加健康检查**: 在控制面板中检测 map 坐标系
3. **改进错误提示**: 明确提示用户需要初始化定位器

---

## 📝 备注

- 系统清理工作已完成，从 48 个重复节点减少到 13 个正常节点
- 地图保存功能已修复并验证
- 控制面板功能正常，可用于日常操作
- 主要问题集中在导航模式的初始化流程

**记录时间**: 2026-01-30 16:58
**记录人**: Claude Code Assistant
