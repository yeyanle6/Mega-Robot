# 地图查看指南

本指南说明如何查看已建立的RTAB-Map地图。

---

## 当前地图状态

**数据库位置**: `~/.ros/rtabmap.db`
**数据库大小**: 22 MB
**主要内容**:
- 扫描数据: 21 MB (97.49%) - LiDAR点云
- 统计数据: 169 KB (0.75%)
- 节点数据: 21 KB (0.10%)
- 链接数据: 54 KB (0.24%)

**地图配置**:
- 纯LiDAR SLAM（无RGB/深度图像）
- 栅格分辨率: 5cm
- 3D建图模式

---

## 方法1: 使用RViz实时查看（推荐）

### 快速启动

```bash
cd ~/Code/Demo5
./scripts/view_map.sh
```

这个脚本会：
1. 检查数据库是否存在
2. 启动必要的基础系统（如未运行）
3. 以**定位模式**启动RTAB-Map（不会修改地图）
4. 启动RViz可视化

### RViz显示内容

在RViz中你可以看到：

- **3D点云地图** (`/mapData`) - 白色/彩色点云
- **2D占用栅格** (`/map`) - 灰色格子
- **历史轨迹** (`/rtabmap/mapPath`) - 绿色路径
- **SLAM图节点** (`/rtabmap/graph/markers`) - 蓝色球体
- **当前位置** - 机器人模型
- **地面点云** (`/cloud/ground`) - 绿色
- **障碍物点云** (`/cloud/obstacles`) - 红色

### 手动启动（如果脚本不工作）

```bash
# 终端1: 启动基础系统（如未运行）
cd ~/Code/Demo5
source install/setup.bash
ros2 launch t_robot_bringup bringup.launch.py

# 终端2: 以定位模式启动RTAB-Map
cd ~/Code/Demo5
source install/setup.bash
ros2 launch t_robot_slam mapping.launch.py \
  launch_bringup:=false \
  delete_db:=false

# 终端3: 启动RViz
cd ~/Code/Demo5
source install/setup.bash
ros2 run rviz2 rviz2 -d src/t_robot_slam/rviz/mapping.rviz
```

---

## 方法2: 导出并查看点云文件

### 导出地图

已为你导出了一个快速预览版本：

**位置**: `maps/quick_view/cloud_cloud.ply`
**大小**: 0.14 MB
**体素大小**: 0.1m (低精度预览)

### 导出更高质量的地图

```bash
# 高质量版本（体素0.05m）
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/high_quality/cloud.pcd \
  --3d --voxel 0.05

# 超高质量版本（体素0.01m）
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/ultra_hq/cloud.ply \
  --3d --format ply --voxel 0.01
```

### 查看导出的点云

#### 使用 PCL Viewer (推荐)

```bash
# 安装PCL工具
sudo apt install pcl-tools

# 查看PCD文件
pcl_viewer maps/quick_view/cloud_cloud.ply

# 或
pcl_viewer maps/high_quality/cloud_cloud.pcd
```

**PCL Viewer 快捷键**:
- 鼠标左键拖动: 旋转视角
- 鼠标滚轮: 缩放
- 鼠标中键拖动: 平移
- `r`: 重置视角
- `j`: 截图
- `q`: 退出

#### 使用 CloudCompare

```bash
# 安装CloudCompare
sudo apt install cloudcompare

# 打开点云
cloudcompare maps/quick_view/cloud_cloud.ply
```

CloudCompare功能更强大：
- 点云对比
- 距离测量
- 法向量显示
- 多种渲染模式

#### 使用 RViz (静态查看)

```bash
cd ~/Code/Demo5
source install/setup.bash

# 创建一个简单的launch文件来加载点云
ros2 run rviz2 rviz2
# 然后在RViz中:
# 1. Add -> PointCloud2
# 2. Topic -> (手动加载PCD文件)
```

---

## 方法3: 使用RTAB-Map Database Viewer

RTAB-Map提供了专门的数据库查看器：

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

**功能**:
- ✅ 查看3D点云地图
- ✅ 查看SLAM图结构
- ✅ 查看每个节点的详细信息
- ✅ 查看回环检测
- ✅ 导出数据
- ✅ 数据库优化

**界面说明**:
- **3D Map**: 显示完整的3D地图
- **Graph**: 显示姿态图和回环
- **Constraints**: 显示节点间的约束
- **Statistics**: 显示建图统计数据

**注意**: 这个工具是图形界面，可能在SSH环境下需要X11转发。

---

## 方法4: 提取地图信息

### 查看地图统计

```bash
# 完整的数据库信息
rtabmap-info ~/.ros/rtabmap.db

# 只看统计数据
rtabmap-info ~/.ros/rtabmap.db | grep -A 20 "^Database"
```

**输出示例**:
```
Database size:      22 MB
Nodes size:         21 KB	(0.10%)
Links size:         54 KB	(0.24%)
Scans size:         21 MB	(97.49%)
Statistics size:    169 KB	(0.75%)
```

### 导出轨迹

```bash
# 导出TUM格式轨迹
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/trajectory.txt \
  --poses

# 查看轨迹数据
cat maps/trajectory.txt
```

**TUM格式说明**:
```
#timestamp x y z qx qy qz qw id
1760599699.531927 0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 0.000000 1
```

### 使用Python读取轨迹

```python
import numpy as np
import matplotlib.pyplot as plt

# 读取TUM格式轨迹
data = np.loadtxt('maps/trajectory.txt', comments='#')
timestamps = data[:, 0]
positions = data[:, 1:4]  # x, y, z

# 绘制2D轨迹
plt.figure(figsize=(10, 10))
plt.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
plt.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='Start')
plt.plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=10, label='End')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory (Top View)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('maps/trajectory_2d.png', dpi=300)
plt.show()

# 绘制3D轨迹
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
ax.plot([positions[0, 0]], [positions[0, 1]], [positions[0, 2]], 'go', markersize=10, label='Start')
ax.plot([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]], 'ro', markersize=10, label='End')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Robot Trajectory (3D View)')
ax.legend()
plt.savefig('maps/trajectory_3d.png', dpi=300)
plt.show()
```

---

## 地图质量检查

### 检查点云数量

```bash
# 查看点云文件大小
ls -lh maps/quick_view/

# 使用PCL查看点数
pcl_pcd_convert_NaN_nan maps/quick_view/cloud_cloud.ply temp.pcd
pcl_points temp.pcd
rm temp.pcd
```

### 检查地图覆盖范围

导出不同体素大小的地图对比：

```bash
# 0.01m - 最精细
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db -o maps/compare/v001.pcd --3d --voxel 0.01

# 0.05m - 标准
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db -o maps/compare/v005.pcd --3d --voxel 0.05

# 0.1m - 预览
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db -o maps/compare/v010.pcd --3d --voxel 0.1

# 对比文件大小
ls -lh maps/compare/
```

### 检查建图统计

如果RTAB-Map正在运行：

```bash
# 查看实时统计
ros2 topic echo /rtabmap/info --once

# 持续监控
ros2 topic echo /rtabmap/info | grep -E "nodes|loop"
```

---

## 常见问题

### Q1: RViz中看不到地图

**可能原因**:
1. Fixed Frame设置错误
2. 话题名称不对
3. QoS设置不匹配

**解决**:
```bash
# 检查话题
ros2 topic list | grep map

# 检查话题类型
ros2 topic info /map

# 在RViz中设置Fixed Frame为 "map"
```

### Q2: 导出的点云很小

**原因**: 机器人移动距离短，建图数据少

**解决**:
- 继续建图，移动更长距离
- 检查数据库大小: `du -h ~/.ros/rtabmap.db`
- 如果数据库<100MB，说明数据量较少

### Q3: PCL Viewer显示错误

**错误**: `Couldn't find file: xxx.pcd`

**解决**:
```bash
# 确认文件存在
ls -l maps/quick_view/cloud_cloud.ply

# 使用绝对路径
pcl_viewer /home/wang/Code/Demo5/maps/quick_view/cloud_cloud.ply
```

### Q4: CloudCompare无法打开PLY文件

**解决**:
```bash
# 检查文件完整性
file maps/quick_view/cloud_cloud.ply

# 转换为PCD格式
pcl_ply2pcd maps/quick_view/cloud_cloud.ply maps/cloud.pcd

# 然后用CloudCompare打开PCD
cloudcompare maps/cloud.pcd
```

---

## 地图文件管理

### 备份地图

```bash
# 备份数据库
cp ~/.ros/rtabmap.db ~/Code/Demo5/maps/backup_$(date +%Y%m%d_%H%M%S).db

# 导出完整数据
mkdir -p maps/archive_$(date +%Y%m%d)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/archive_$(date +%Y%m%d)/cloud.pcd \
  --3d --voxel 0.05
```

### 比较不同时间的地图

```bash
# 使用CloudCompare比较两个地图
cloudcompare maps/backup1_cloud.pcd maps/backup2_cloud.pcd
# 在CloudCompare中: Tools -> Registration -> Fine Registration (ICP)
```

### 压缩导出文件

```bash
# 压缩点云文件
tar -czvf maps/cloud_archive.tar.gz maps/quick_view/

# 解压
tar -xzvf maps/cloud_archive.tar.gz
```

---

## 快速参考命令

```bash
# 查看实时地图（最简单）
./scripts/view_map.sh

# 导出快速预览
./src/t_robot_slam/scripts/export_rtabmap.py ~/.ros/rtabmap.db \
  -o maps/preview.pcd --3d --voxel 0.1

# 导出高质量
./src/t_robot_slam/scripts/export_rtabmap.py ~/.ros/rtabmap.db \
  -o maps/hq.pcd --3d --voxel 0.02

# 查看点云
pcl_viewer maps/preview_cloud.ply

# 查看数据库信息
rtabmap-info ~/.ros/rtabmap.db | head -100

# 数据库可视化工具
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

---

## 相关文档

- 地图导出详细指南: `docs/MAP_EXPORT_GUIDE.md`
- 建图指南: `docs/MAPPING_GUIDE.md`
- 快速开始: `docs/QUICK_START.md`

---

**最后更新**: 2025-10-16
**当前地图**: `~/.ros/rtabmap.db` (22 MB)
