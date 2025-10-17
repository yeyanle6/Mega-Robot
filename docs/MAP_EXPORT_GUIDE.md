# RTAB-Map 地图导出指南

## 概述

本指南说明如何从RTAB-Map数据库导出地图数据，包括3D点云和机器人轨迹。

---

## 导出工具

脚本位置: `src/t_robot_slam/scripts/export_rtabmap.py`

### 支持的导出格式

✅ **3D点云**: PCD或PLY格式，从LiDAR扫描数据生成
✅ **机器人轨迹**: TUM, KITTI或G2O格式
⚠️ **2D占用栅格地图**: 需要使用ROS话题实时保存（见下方说明）

---

##使用方法

### 1. 导出3D点云 (推荐)

```bash
# 导出PCD格式 (默认)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/my_map/cloud.pcd \
  --3d \
  --voxel 0.05

# 导出PLY格式 (带法向量)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/my_map/cloud.ply \
  --3d \
  --format ply \
  --voxel 0.01

# 限制导出范围
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/cloud.pcd \
  --3d \
  --voxel 0.02 \
  --max-range 10.0
```

**参数说明**:
- `--voxel`: 体素下采样大小（米），越小越精细但文件越大
  - 0.01m = 1cm (高精度，文件大)
  - 0.05m = 5cm (中等精度，推荐)
  - 0.1m = 10cm (低精度，文件小)
- `--max-range`: 最大点距离（米），0=无限制
- `--format`: 输出格式 (`pcd` 或 `ply`)

### 2. 导出机器人轨迹

```bash
# TUM格式 (默认，用于评估SLAM)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/trajectory.txt \
  --poses

# G2O格式 (用于图优化)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/poses.g2o \
  --poses \
  --format g2o

# KITTI格式
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/poses.txt \
  --poses \
  --format kitti
```

**输出格式**:
- **TUM**: `timestamp x y z qx qy qz qw` (四元数)
- **KITTI**: 3x4变换矩阵
- **G2O**: 图优化格式

### 3. 导出2D占用栅格地图（实时方式）

由于`rtabmap-export`不直接支持从数据库导出2D栅格地图，需要在RTAB-Map运行时从ROS话题保存：

#### 方法A: 使用map_saver (需要安装Nav2)

```bash
# 1. 安装map_saver工具
sudo apt install ros-humble-nav2-map-server

# 2. 确保RTAB-Map正在运行并发布/map话题
ros2 topic hz /map  # 确认地图在发布

# 3. 保存地图
ros2 run nav2_map_server map_saver_cli -f maps/2d_map

# 输出文件:
# - maps/2d_map.pgm (灰度图像)
# - maps/2d_map.yaml (元数据：分辨率、原点等)
```

#### 方法B: 订阅/map话题手动转换

如果没有Nav2，可以订阅`/map`话题并手动保存：

```bash
ros2 topic echo /map --once > map_data.txt
# 然后编写脚本将OccupancyGrid消息转换为PGM格式
```

---

## 典型工作流程

### 场景1: 完成建图后导出所有数据

```bash
# 1. 停止RTAB-Map节点
pkill -f rtabmap

# 2. 创建导出目录
mkdir -p maps/office_20251016

# 3. 导出3D点云 (高质量)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/office_20251016/cloud_hq.ply \
  --3d \
  --format ply \
  --voxel 0.01

# 4. 导出3D点云 (低质量，用于预览)
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/office_20251016/cloud_preview.pcd \
  --3d \
  --voxel 0.1

# 5. 导出轨迹
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/office_20251016/trajectory.txt \
  --poses

# 6. 重新启动RTAB-Map保存2D地图
ros2 launch t_robot_slam mapping.launch.py \
  launch_bringup:=false \
  delete_db:=false

# 等待地图发布后（1-2分钟）
ros2 run nav2_map_server map_saver_cli \
  -f maps/office_20251016/2d_map
```

### 场景2: 快速导出当前建图进度

```bash
# 边建图边导出（不中断RTAB-Map）
./src/t_robot_slam/scripts/export_rtabmap.py \
  ~/.ros/rtabmap.db \
  -o maps/current/preview.pcd \
  --3d \
  --voxel 0.1
```

注意: 在RTAB-Map运行时导出可能会有数据库锁定问题，建议停止RTAB-Map后导出。

---

## 查看导出的点云

### 使用PCL工具

```bash
# 安装PCL工具
sudo apt install pcl-tools

# 查看PCD文件
pcl_viewer maps/cloud.pcd

# 查看PLY文件
pcl_viewer maps/cloud.ply

# 转换PCD到PLY
pcl_pcd2ply maps/cloud.pcd maps/cloud.ply
```

### 使用RViz 2

```bash
# 创建一个简单的launch文件来加载点云
# 或者使用rviz2直接加载PLY文件
rviz2
# File -> Open Config -> 添加 PointCloud2 display
# 加载文件: maps/cloud.pcd
```

### 使用CloudCompare (图形化工具)

```bash
# 安装CloudCompare
sudo apt install cloudcompare

# 打开点云
cloudcompare maps/cloud.ply
```

---

## 文件大小估算

基于10分钟建图测试（约300个节点）：

| 体素大小 | 文件大小 (PCD) | 文件大小 (PLY) | 点数 | 用途 |
|---------|---------------|---------------|------|------|
| 0.01m | 50-100 MB | 60-120 MB | 500万-1000万 | 高精度测量 |
| 0.02m | 15-30 MB | 20-40 MB | 100万-200万 | 精细建模 |
| 0.05m | 2-5 MB | 3-8 MB | 20万-50万 | 标准建图 ✅ |
| 0.1m | 0.5-1 MB | 0.8-1.5 MB | 5万-10万 | 快速预览 |

**推荐**:
- 正式归档: `--voxel 0.02` (精细)
- 日常使用: `--voxel 0.05` (标准)
- 快速检查: `--voxel 0.1` (预览)

---

## 故障排查

### 问题1: 导出的点云为空

**症状**:
```
Create and assemble the clouds... done (0.001s, 0 points).
Export failed! The cloud is empty.
```

**原因**:
- 机器人没有移动，数据库中只有极少的点云数据
- 或者RTAB-Map使用RGB-D模式而非LiDAR模式

**解决**:
- 确保机器人已经移动并建图
- 检查数据库大小: `ls -lh ~/.ros/rtabmap.db` (应该 > 100 MB)
- 确认导出脚本使用了 `--scan` 选项（已内置）

### 问题2: 导出文件路径错误

**症状**:
```
Saving /home/wang/.ros//home/wang/Code/Demo5/maps/...
```

**原因**: rtabmap-export的路径处理bug

**解决**: 已在脚本中修复，使用`--output_dir`选项

### 问题3: 2D地图无法导出

**症状**: rtabmap-export没有2D地图导出选项

**解决**: 使用实时方式：
1. 确保RTAB-Map在运行
2. 安装nav2_map_server
3. 使用map_saver_cli保存/map话题

### 问题4: 数据库锁定

**症状**:
```
Error: database is locked
```

**原因**: RTAB-Map节点正在访问数据库

**解决**:
```bash
# 停止所有RTAB-Map节点
pkill -f rtabmap

# 或者等待几秒后重试
```

---

## 导出脚本输出示例

### 成功导出3D点云

```
Database: /home/wang/.ros/rtabmap.db
Size: 48.13 MB

[Export 3D Point Cloud]
  Output file: /home/wang/Code/Demo5/maps/test/cloud.pcd
  Format: PCD
  Voxel size: 0.05m
  ✓ Point cloud exported successfully
    - /home/wang/Code/Demo5/maps/test/cloud_cloud.ply
    - Size: 0.14 MB
```

### 成功导出轨迹

```
Database: /home/wang/.ros/rtabmap.db
Size: 64.00 MB

[Export Trajectory]
  Output file: /home/wang/Code/Demo5/maps/test/trajectory
  Format: TUM
  ✓ Trajectory exported successfully
    - /home/wang/Code/Demo5/maps/test/trajectory_poses.txt
```

---

## 进阶技巧

### 1. 批量导出不同精度的点云

```bash
#!/bin/bash
mkdir -p maps/multi_res

for voxel in 0.01 0.02 0.05 0.1; do
  echo "Exporting voxel size: ${voxel}m"
  ./src/t_robot_slam/scripts/export_rtabmap.py \
    ~/.ros/rtabmap.db \
    -o maps/multi_res/cloud_v${voxel}.pcd \
    --3d \
    --voxel $voxel
done
```

### 2. 限制导出区域

rtabmap-export不支持区域过滤，但可以导出后用PCL工具裁剪：

```bash
# 使用PCL工具裁剪点云
pcl_passthrough_filter \
  maps/cloud.pcd \
  maps/cloud_cropped.pcd \
  -field z -min 0.0 -max 2.0
```

### 3. 合并多次建图的点云

如果有多个数据库文件，可以分别导出后用PCL合并：

```bash
# 分别导出
./export_rtabmap.py ~/.ros/map1.db -o maps/cloud1.pcd --3d
./export_rtabmap.py ~/.ros/map2.db -o maps/cloud2.pcd --3d

# 使用PCL合并
pcl_concatenate_points_pcd \
  maps/cloud1.pcd \
  maps/cloud2.pcd \
  -o maps/combined.pcd
```

---

## 相关命令

### 查看数据库统计信息

```bash
rtabmap-info ~/.ros/rtabmap.db
```

### 数据库恢复

```bash
rtabmap-recovery ~/.ros/rtabmap_corrupted.db
```

### 数据库重新优化

```bash
rtabmap-reprocess \
  --input ~/.ros/rtabmap.db \
  --output ~/.ros/rtabmap_optimized.db
```

---

## 参考资料

- [RTAB-Map导出文档](https://github.com/introlab/rtabmap/wiki/Tools)
- [TUM轨迹格式说明](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
- [PCL工具文档](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html)

---

**最后更新**: 2025-10-16
