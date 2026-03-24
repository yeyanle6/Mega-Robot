# 导航修复检查清单
**快速参考** | 2026-01-30

---

## 🔍 诊断步骤

### 1. 检查 map_server
```bash
# 节点是否存在
ros2 node list | grep map_server

# 生命周期状态
ros2 lifecycle get /map_server

# /map 话题是否存在
ros2 topic echo /map --once
```

**期望结果**:
- ✅ map_server 节点存在
- ✅ 状态为 `active [3]`
- ✅ /map 话题有数据

---

### 2. 检查 localizer
```bash
# 节点是否运行
ros2 node list | grep localizer

# 检查 map → odom TF
ros2 run tf2_ros tf2_echo map odom

# 检查 relocalize 服务
ros2 service list | grep relocalize
```

**期望结果**:
- ✅ localizer_node 存在
- ❌ map → odom TF 不存在（未初始化）
- ✅ /localizer/relocalize 服务存在

---

### 3. 检查当前 TF 树
```bash
# 查看完整 TF 树
ros2 run tf2_tools view_frames

# 检查 odom → base_link
ros2 run tf2_ros tf2_echo odom base_link
```

**期望结果**:
- ✅ odom → base_link 正常（FastLIO2 发布）
- ❌ map 坐标系不存在

---

## 🔧 修复步骤

### 选项 A: 激活 map_server（如果未激活）
```bash
# 1. 配置
ros2 lifecycle set /map_server configure

# 2. 激活
ros2 lifecycle set /map_server activate

# 3. 验证
ros2 topic echo /map --once
```

---

### 选项 B: 盲初始化 localizer（原点假设）
```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize "{
  pcd_path: '/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/1632.pcd',
  x: 0.0,
  y: 0.0,
  z: 0.0,
  yaw: 0.0,
  pitch: 0.0,
  roll: 0.0
}"
```

**期望输出**:
```
response:
  success: True
  message: 'Relocalization successful'
```

---

### 选项 C: 建图模式确认位置后初始化
```bash
# 1. 在控制面板停止导航，启动 SLAM
# 2. 在 RViz2 查看机器人位置，记录坐标
# 3. 停止 SLAM，重启导航
# 4. 使用记录的坐标调用 relocalize（同选项 B）
```

---

## ✅ 验证步骤

### 1. 验证 map 坐标系
```bash
# 应该能看到持续更新的变换
ros2 run tf2_ros tf2_echo map base_link
```

**期望输出**:
```
At time ...
- Translation: [x, y, z]
- Rotation: ...
```

---

### 2. 验证 /map 话题
```bash
ros2 topic hz /map
```

**期望输出**:
```
average rate: 0.2 (或其他低频率，地图通常不常更新)
```

---

### 3. 验证 Nav2 错误消失
```bash
# 查看控制面板日志，应该不再有:
# - "Invalid frame ID 'map'" 错误
# - "Message Filter dropping message" 警告
```

---

### 4. 测试导航功能
```
1. 打开 RViz2
2. 点击 "2D Nav Goal" 设置目标点
3. 观察机器人是否规划路径
4. 观察全局代价地图是否显示
```

---

## 🚨 常见问题

### Q1: relocalize 返回 success: False
**原因**: 初始位置猜错，离实际位置太远

**解决**:
- 使用建图模式确认实际位置（选项 C）
- 或在 RViz2 中多尝试几个位置

---

### Q2: map_server 节点不存在
**原因**: 导航模式未正确启动

**解决**:
1. 停止当前进程
2. 重新在控制面板启动 "导航 (PGO地图)"
3. 确保选择了正确的地图文件

---

### Q3: /map 话题存在但 RViz2 看不到地图
**原因**: RViz2 配置问题或坐标系问题

**解决**:
1. 检查 RViz2 的 Fixed Frame 设置（应为 `map`）
2. 检查 Map 显示插件的 Topic 设置（应为 `/map`）
3. 重启 RViz2

---

### Q4: 初始化后位置明显错误
**解决**:
1. 在 RViz2 中点击 "2D Pose Estimate"
2. 在地图上点击正确位置
3. 拖动设置正确朝向
4. localizer 会自动重新定位

---

## 📋 完整流程总结

```
1. 启动导航模式（控制面板）
   ↓
2. 等待所有节点启动
   ↓
3. 检查 map_server 状态
   ├─ 未激活 → 手动激活
   └─ 已激活 → 继续
   ↓
4. 初始化 localizer
   ├─ 已知位置 → 直接 relocalize
   ├─ 不确定 → 盲初始化（原点）
   └─ 完全不知道 → 建图模式确认
   ↓
5. 验证 TF 和话题
   ↓
6. 测试导航功能
   ↓
7. 完成 ✅
```

---

**提示**: 保存此文件以备后续参考。每次启动导航模式都需要初始化 localizer（除非改进 launch 文件自动初始化）。
