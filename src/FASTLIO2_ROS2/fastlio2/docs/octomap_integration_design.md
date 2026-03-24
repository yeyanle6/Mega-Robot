# FASTLIO2 + OctoMap 集成设计方案

## 1. 概述

将FASTLIO2的3D点云通过OctoMap转换为3D体素占用地图（方块世界），然后从中提取2D占用栅格地图用于导航。

## 2. 数据流设计

```
FASTLIO2 LIO Node
    │
    ├── /world_cloud (sensor_msgs/PointCloud2) ──┐
    │                                              │
    └── /odom (nav_msgs/Odometry) ───────────────┤
                                                   │
                                                   ▼
                                        OctoMap Server Node
                                        (3D体素地图构建)
                                                   │
                                                   ├── OctoMap 3D (内部)
                                                   │   └── 体素占用地图
                                                   │       (occupied/free/unknown)
                                                   │
                                                   ├── /octomap_full (octomap_msgs/Octomap)
                                                   │   └── 完整3D OctoMap消息
                                                   │
                                                   └── /octomap_binary (octomap_msgs/Octomap)
                                                       └── 二进制OctoMap消息
                                                   │
                                                   ▼
                                        OctoMap → 2D 投影
                                        (高度切片提取)
                                                   │
                                                   ▼
                                        /map (nav_msgs/OccupancyGrid)
                                        └── 2D占用栅格地图
                                            - 未知: -1
                                            - 自由: 0
                                            - 占用: 100
```

## 3. 架构设计

### 3.1 节点架构

```
┌─────────────────────────────────────────────────────────┐
│                    FASTLIO2 LIO Node                     │
│  - 输入: /livox/lidar, /livox/imu                        │
│  - 输出: /world_cloud (PointCloud2)                      │
│  - 输出: /odom (Odometry)                                │
│  - TF: odom → base_link                                  │
└─────────────────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────┐
│                  OctoMap Server Node                     │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │  1. 点云接收与预处理                               │  │
│  │     - 订阅 /world_cloud                           │  │
│  │     - 坐标变换到目标frame                          │  │
│  │     - 点云过滤（范围、高度）                       │  │
│  └──────────────────────────────────────────────────┘  │
│                        │                                 │
│                        ▼                                 │
│  ┌──────────────────────────────────────────────────┐  │
│  │  2. OctoMap构建                                   │  │
│  │     - 插入点云到OctoMap                           │  │
│  │     - 更新体素占用状态                             │  │
│  │     - 概率更新（占用/自由/未知）                   │  │
│  │     - 体素分辨率控制                               │  │
│  └──────────────────────────────────────────────────┘  │
│                        │                                 │
│                        ▼                                 │
│  ┌──────────────────────────────────────────────────┐  │
│  │  3. 3D OctoMap发布                               │  │
│  │     - /octomap_full (完整OctoMap)                 │  │
│  │     - /octomap_binary (二进制OctoMap)             │  │
│  │     - 可选：/octomap_marker (可视化)               │  │
│  └──────────────────────────────────────────────────┘  │
│                        │                                 │
│                        ▼                                 │
│  ┌──────────────────────────────────────────────────┐  │
│  │  4. 2D地图提取                                    │  │
│  │     - 高度切片：z_min ~ z_max                     │  │
│  │     - 投影到XY平面                                │  │
│  │     - 体素占用 → 栅格占用                          │  │
│  │     - 生成nav_msgs/OccupancyGrid                  │  │
│  └──────────────────────────────────────────────────┘  │
│                        │                                 │
│                        ▼                                 │
│  ┌──────────────────────────────────────────────────┐  │
│  │  5. 2D地图发布                                    │  │
│  │     - /map (nav_msgs/OccupancyGrid)               │  │
│  │     - frame_id: odom (或map)                      │  │
│  │     - 分辨率、原点、尺寸                           │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

### 3.2 TF树结构

```
map (可选，由PGO/Localizer发布)
 └── odom (FASTLIO2的world_frame)
      └── base_link (FASTLIO2的body_frame)
           └── base_footprint (静态TF)
```

**说明：**
- OctoMap Server使用 `odom` 作为frame_id（与FASTLIO2的world_frame一致）
- 如果使用PGO/Localizer，它们会发布 `map → odom` 的TF
- 2D地图的frame_id应该与OctoMap的frame_id一致

## 4. 节点设计

### 4.1 OctoMap Server Node

**功能：**
1. 接收FASTLIO2的点云
2. 构建3D OctoMap（体素占用地图）
3. 发布3D OctoMap消息
4. 从OctoMap提取并发布2D占用栅格地图

**话题接口：**

订阅：
- `/world_cloud` (sensor_msgs/PointCloud2) - FASTLIO2的世界坐标系点云
- `/tf` (TF树) - 用于坐标变换

发布：
- `/octomap_full` (octomap_msgs/Octomap) - 完整3D OctoMap
- `/octomap_binary` (octomap_msgs/Octomap) - 二进制OctoMap（压缩）
- `/map` (nav_msgs/OccupancyGrid) - 2D占用栅格地图
- `/octomap_marker` (visualization_msgs/MarkerArray) - 可视化标记（可选）

**服务接口：**
- `/octomap_server/reset` (std_srvs/Empty) - 重置OctoMap
- `/octomap_server/save_map` (std_srvs/Trigger) - 保存OctoMap到文件

**参数配置：**

```yaml
octomap_server:
  ros__parameters:
    # 基础配置
    frame_id: "odom"              # OctoMap的坐标系
    resolution: 0.05              # 体素分辨率 (5cm)
    max_depth: 16                 # OctoMap最大深度
    
    # 点云处理
    pointcloud_min_z: -0.5         # 最小高度过滤 (m)
    pointcloud_max_z: 2.0          # 最大高度过滤 (m)
    max_range: 30.0                # 最大传感器范围 (m)
    
    # 传感器模型
    sensor_model_type: "ray"       # "ray" 或 "endpoint"
    sensor_model_max_range: 30.0   # 传感器最大范围
    occupancy_min_z: -0.5         # 2D地图最小高度 (m)
    occupancy_max_z: 2.0          # 2D地图最大高度 (m)
    
    # 概率阈值
    prob_hit: 0.7                  # 命中概率
    prob_miss: 0.4                 # 未命中概率
    clamping_thresh_min: 0.1192    # 最小概率阈值
    clamping_thresh_max: 0.971     # 最大概率阈值
    
    # 2D地图配置
    publish_2d_map: true           # 是否发布2D地图
    map_2d_resolution: 0.05        # 2D地图分辨率 (m)
    map_2d_height_slice: 0.0       # 2D地图高度切片中心 (m)
    map_2d_height_slice_thickness: 2.5  # 2D地图高度切片厚度 (m)
    
    # 发布频率
    publish_3d_map_rate: 1.0       # 3D地图发布频率 (Hz)
    publish_2d_map_rate: 1.0       # 2D地图发布频率 (Hz)
    
    # 其他
    track_unknown_space: true      # 跟踪未知空间
    latch: false                   # 是否锁定话题
    use_sim_time: false            # 使用仿真时间
```

## 5. 实现方案

### 5.1 方案A：使用ros-humble-octomap-server（推荐）

**优点：**
- 现成的ROS2包，开箱即用
- 功能完整，支持3D和2D地图发布
- 维护良好

**缺点：**
- 参数配置可能需要调整
- 需要确保与FASTLIO2的坐标系一致

**实现步骤：**
1. 安装依赖：`sudo apt install ros-humble-octomap-server ros-humble-octomap-msgs`
2. 创建launch文件，配置octomap_server节点
3. 配置参数，订阅FASTLIO2的`/world_cloud`
4. 设置正确的frame_id和高度过滤参数
5. 测试2D地图发布

### 5.2 方案B：自定义OctoMap节点

**优点：**
- 完全可控，可以精确控制处理流程
- 可以针对FASTLIO2优化

**缺点：**
- 开发工作量大
- 需要维护代码

**实现步骤：**
1. 创建新的ROS2包 `fastlio2_octomap`
2. 实现OctoMap构建逻辑
3. 实现2D地图提取逻辑
4. 发布3D和2D地图

## 6. Launch文件设计

### 6.1 fastlio2_octomap.launch.py

```python
"""
FASTLIO2 + OctoMap集成启动文件

功能：
1. 启动FASTLIO2 LIO节点
2. 启动OctoMap Server节点
3. 发布静态TF
4. 可选启动RViz

数据流：
FASTLIO2 → OctoMap 3D → 2D地图
"""

# 节点列表：
# 1. FASTLIO2 LIO Node
# 2. OctoMap Server Node
# 3. Static TF Publishers
# 4. RViz (可选)
```

## 7. 配置参数设计

### 7.1 lio_megarover.yaml（FASTLIO2配置）

当前配置保持不变，但需要确保：
- `world_frame: odom` - 与OctoMap的frame_id一致
- `body_frame: base_link` - 标准body frame

### 7.2 octomap_megarover.yaml（OctoMap配置）

新建配置文件，包含上述所有OctoMap参数。

## 8. 测试方案

### 8.1 功能测试

1. **点云接收测试**
   - 检查OctoMap Server是否接收到`/world_cloud`
   - 检查点云数量和质量

2. **3D OctoMap构建测试**
   - 检查`/octomap_full`和`/octomap_binary`是否发布
   - 在RViz中可视化3D OctoMap
   - 检查体素分辨率是否正确

3. **2D地图发布测试**
   - 检查`/map`话题是否发布
   - 检查地图分辨率、尺寸、原点
   - 在RViz中可视化2D地图
   - 检查占用/自由/未知区域是否正确

### 8.2 性能测试

1. 点云处理频率
2. OctoMap更新频率
3. 2D地图发布频率
4. 内存使用情况

## 9. 集成到现有系统

### 9.1 与PGO集成

如果使用PGO，TF树变为：
```
map (PGO发布)
 └── odom (FASTLIO2发布)
      └── base_link
```

OctoMap Server应该使用`map`作为frame_id，或者保持`odom`但通过TF转换。

### 9.2 与Localizer集成

类似PGO，Localizer会发布`map → odom`的TF。

### 9.3 与Nav2集成

Nav2需要`/map`话题，OctoMap Server直接提供，无需额外转换。

## 10. 后续优化

1. **动态地图更新**：支持删除旧体素，保持地图大小
2. **多分辨率OctoMap**：支持不同分辨率的OctoMap
3. **地图保存/加载**：支持保存和加载OctoMap
4. **高度分层地图**：支持多个高度的2D地图切片

