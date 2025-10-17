# TF冲突修复指南

**状态**: ✅ **已修复** (2025-10-17)
**验证**: 雷达数据不再闪烁，TF树结构正确

## 问题描述

雷达数据在RViz中90度旋转闪烁。

## 根本原因

**两个TF发布源冲突**：

1. `sensors.launch.py` - 手动发布静态TF:
   ```
   base_link → mid360_base → mid360_lidar (无倾角)
   ```

2. `robot_state_publisher` (从URDF) - 发布完整TF树:
   ```
   base_link → top_plate_link → mid360_base → mid360_lidar (有30°倾角)
   ```

两个源同时发布`mid360_base`和`mid360_lidar`，导致TF树冲突和闪烁。

## 解决方案

### ✅ 方案1: 删除sensors.launch.py中的静态TF（已实施）

编辑 `src/t_robot_bringup/launch/sensors.launch.py`，注释掉或删除静态TF发布器：

```python
# 注释掉这三个节点：
# static_tf_mid360_base
# static_tf_mid360_lidar
# static_tf_mid360_imu

return LaunchDescription([
    use_sim_time_arg,
    use_mid360_arg,
    mid360_launch,
    lidar_relay,
    # static_tf_mid360_base,      # 删除或注释
    # static_tf_mid360_lidar,     # 删除或注释
    # static_tf_mid360_imu,       # 删除或注释
])
```

**原因**: robot_state_publisher已经从URDF正确发布了所有TF，包括30°倾角。

**实施日期**: 2025-10-17
**修改文件**: `src/t_robot_bringup/launch/sensors.launch.py:110-121`

### 方案2: 修改静态TF包含30°倾角（未采用）

如果你想保留静态TF方式，需要添加30°倾角：

```python
static_tf_mid360_base = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_mid360_base',
    output='log',
    arguments=[
        '0.12', '0', '0.45',  # x y z (从URDF计算)
        '0', '0.2588', '0', '0.9659',  # 30度pitch的四元数
        'base_link',
        'mid360_base'
    ],
    parameters=[{'use_sim_time': use_sim_time}]
)
```

**但是不推荐**，因为：
- 需要手动同步URDF和launch文件
- 容易出错
- URDF已经有完整的机器人模型

## ✅ 修复验证结果

### TF树结构（修复后）

正确的TF树路径：
```
map → odom → base_footprint → base_link → top_plate_link → mid360_base → mid360_lidar
```

**验证命令**:
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 测试结果

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| TF路径 | ❌ 2条冲突 | ✅ 1条唯一路径 |
| 雷达显示 | ❌ 闪烁/90°旋转 | ✅ 正常稳定 |
| RTAB-Map | ❌ 未测试 | ✅ 正常运行 20-30ms |
| 点云频率 | ❌ 未测试 | ✅ 10Hz 稳定 |

**测试日期**: 2025-10-17
**测试环境**: Jetson AGX Orin, ROS 2 Humble

## 验证TF树

正确的TF树应该是：

```
map
 └─ odom
     └─ base_footprint
         └─ base_link
             ├─ left_wheel
             ├─ right_wheel
             └─ top_plate_link
                 └─ mid360_base (有30°倾角)
                     ├─ mid360_lidar
                     └─ mid360_imu
```

## 检查命令

```bash
# 查看TF树
ros2 run tf2_tools view_frames
evince frames.pdf

# 查看特定TF变换
ros2 run tf2_ros tf2_echo base_link mid360_lidar

# 应该看到30°倾角 (pitch ~0.5236弧度)
```

## 注意事项

1. **不要同时发布相同的TF**: 一个frame只能有一个parent
2. **URDF是权威源**: 机器人结构应该在URDF中定义
3. **livox_frame问题**: 如果Livox驱动也发布`livox_frame`，需要将其映射到`mid360_lidar`

## 相关文件

- `src/t_robot_bringup/launch/sensors.launch.py` - 传感器启动文件
- `src/megarover3_ros2/megarover_description/urdf/mega3.xacro` - 机器人URDF
- `src/megarover3_ros2/megarover_description/urdf/calibration_offsets.xacro` - 30°倾角定义
- `src/megarover3_ros2/megarover_description/urdf/sensors/mid360.xacro` - MID360传感器定义
