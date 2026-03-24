# FASTLIO2 + OctoMap 使用说明

## 概述

本集成将FASTLIO2的3D点云通过OctoMap转换为3D体素占用地图（方块世界），然后自动提取并发布2D占用栅格地图。

## 数据流

```
FASTLIO2 LIO Node
    │
    ├── /world_cloud (PointCloud2) ──┐
    │                                 │
    └── TF: odom → base_link          │
                                       │
                                       ▼
                            OctoMap Server Node
                            (构建3D体素地图)
                                       │
                                       ├── 3D OctoMap (内部)
                                       │   └── 体素占用状态
                                       │
                                       └── /map (OccupancyGrid)
                                           └── 2D占用栅格地图
```

## 安装依赖

```bash
sudo apt install ros-humble-octomap-server ros-humble-octomap-msgs
```

## 启动方式

### 基础启动（仅FASTLIO2 + OctoMap）

```bash
ros2 launch fastlio2 fastlio2_octomap.launch.py
```

### 带参数启动

```bash
# 禁用RViz
ros2 launch fastlio2 fastlio2_octomap.launch.py rviz:=false

# 使用仿真时间
ros2 launch fastlio2 fastlio2_octomap.launch.py use_sim_time:=true
```

## 话题接口

### 订阅的话题

- `/world_cloud` (sensor_msgs/PointCloud2) - FASTLIO2发布的世界坐标系点云
- `/tf` (TF树) - 用于坐标变换

### 发布的话题

- `/map` (nav_msgs/OccupancyGrid) - **2D占用栅格地图**（主要输出）
- `/octomap_full` (octomap_msgs/Octomap) - 完整3D OctoMap（可选）
- `/octomap_binary` (octomap_msgs/Octomap) - 二进制OctoMap（可选）

## TF树结构

```
map (静态TF，或由PGO/Localizer发布)
 └── odom (FASTLIO2的world_frame)
      └── base_link (FASTLIO2的body_frame)
           └── base_footprint (静态TF)
```

## 配置参数

配置文件位置：`config/octomap_megarover.yaml`

### 关键参数说明

- **frame_id**: `odom` - OctoMap的坐标系（与FASTLIO2的world_frame一致）
- **resolution**: `0.05` - 体素分辨率（5cm）
- **occupancy_min_z**: `-0.5` - 2D地图最小高度（m）
- **occupancy_max_z**: `2.0` - 2D地图最大高度（m）
- **publish_2d_map**: `true` - 是否发布2D地图

## 使用场景

### 1. 纯SLAM建图

```bash
# 启动FASTLIO2 + OctoMap
ros2 launch fastlio2 fastlio2_octomap.launch.py

# 播放数据包
ros2 bag play your_bag_file
```

### 2. 与PGO集成（带回环检测）

```bash
# 终端1: 启动FASTLIO2 + OctoMap
ros2 launch fastlio2 fastlio2_octomap.launch.py

# 终端2: 启动PGO（会发布map->odom的TF）
ros2 launch pgo pgo_launch.py
```

### 3. 与Localizer集成（重定位）

```bash
# 终端1: 启动FASTLIO2 + OctoMap
ros2 launch fastlio2 fastlio2_octomap.launch.py

# 终端2: 启动Localizer（会发布map->odom的TF）
ros2 launch localizer localizer_launch.py
```

### 4. 与Nav2集成（导航）

```bash
# 启动FASTLIO2 + OctoMap + Nav2
ros2 launch megarover3_navigation fastlio2_navigation.launch.py mode:=slam
```

## 可视化

### RViz可视化

启动后会自动打开RViz，可以添加以下显示：

1. **Map** - 显示2D占用栅格地图
   - Topic: `/map`
   - Type: `Map`

2. **PointCloud2** - 显示原始点云
   - Topic: `/world_cloud`
   - Type: `PointCloud2`

3. **OctoMap** - 显示3D OctoMap（如果安装了octomap_rviz_plugins）
   - Topic: `/octomap_full`
   - Type: `OctoMap`

### 检查地图发布

```bash
# 检查2D地图是否发布
ros2 topic echo /map --once

# 检查地图信息
ros2 topic info /map

# 检查地图频率
ros2 topic hz /map
```

## 故障排查

### 问题1: OctoMap Server没有接收到点云

**检查：**
```bash
# 检查点云是否发布
ros2 topic echo /world_cloud --once

# 检查OctoMap Server日志
ros2 run octomap_server octomap_server_node --ros-args -p cloud_in:=/world_cloud
```

**解决：**
- 确保FASTLIO2正常运行
- 检查话题重映射是否正确

### 问题2: 2D地图为空或没有发布

**检查：**
```bash
# 检查OctoMap Server参数
ros2 param list /octomap_server

# 检查publish_2d_map参数
ros2 param get /octomap_server publish_2d_map
```

**解决：**
- 确保`publish_2d_map: true`
- 检查`occupancy_min_z`和`occupancy_max_z`设置是否合理
- 确保有足够的点云数据

### 问题3: TF错误

**检查：**
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查特定TF
ros2 run tf2_ros tf2_echo odom base_link
```

**解决：**
- 确保FASTLIO2正常发布TF
- 检查frame_id配置是否正确

## 性能优化

1. **调整体素分辨率**
   - 降低分辨率（增大值）可以减少内存使用
   - 提高分辨率（减小值）可以提高地图精度

2. **调整高度范围**
   - 减小`occupancy_max_z`可以减少处理的数据量

3. **调整发布频率**
   - 降低`publish_2d_map_rate`可以减少CPU使用

## 相关文档

- [OctoMap集成设计方案](./octomap_integration_design.md)
- [FASTLIO2配置说明](../config/lio_megarover.yaml)





