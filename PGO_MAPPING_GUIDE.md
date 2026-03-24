# FastLIO2 + PGO 建图与导航指南

## 🎯 系统架构

### 方案 A：PGO 优化建图（推荐）✨

**建图流程：**
```
MID360 激光雷达 → FAST-LIO2 → PGO (回环检测+优化)
                                  ↓
                          保存优化后的3D点云地图 (.pcd)
                                  +
                            保存2D栅格地图 (.yaml/.pgm)
```

**导航流程：**
```
加载3D点云地图 → Localizer (scan-to-map定位) → 高精度定位
        +
加载2D栅格地图 → Nav2 → 全局路径规划
```

**优势：**
- ✅ 回环检测，消除累积误差
- ✅ 地图优化，更高质量
- ✅ 稳定定位，基于预建地图
- ✅ 可重复使用，适合固定环境

---

## 📋 使用方法

### 1. 启动控制面板

```bash
~/megarover3_panel
```

控制面板现在包含以下组件：

- 🤖 底盘 (Micro-ROS)
- 📡 MID360 激光雷达
- 📷 D455 前方相机
- 🗺️ **SLAM (建图-简单)** - 原始SLAM，无PGO
- 🗺️ **SLAM (建图-PGO优化)** - 新增！带回环检测
- 🧭 导航 (简单) - 基于简单SLAM的导航
- 🧭 **导航 (PGO地图)** - 新增！基于优化地图的高精度导航
- ⌨️ 键盘遥控

---

### 2. 建图流程（PGO 模式）

#### 步骤 1：启动底盘和激光雷达

在控制面板中依次点击：
1. **底盘 (Micro-ROS)** - 启动按钮
2. **MID360 激光雷达** - 启动按钮
3. **D455 前方相机** - 启动按钮（可选）

#### 步骤 2：启动 PGO 建图

点击 **SLAM (建图-PGO优化)** - 启动按钮

系统会启动：
- ✅ FAST-LIO2 - 激光雷达惯性里程计
- ✅ PGO 节点 - 回环检测和优化
- ✅ Patchwork++ - 地面分割
- ✅ OctoMap - 2D地图生成
- ✅ RViz - 可视化

#### 步骤 3：移动机器人建图

使用键盘遥控或手动移动机器人：

```bash
# 可选：启动键盘遥控
点击控制面板中的"键盘遥控"按钮
```

**建图建议：**
- 🔄 **经过同一地点多次** - 触发回环检测
- 🐢 **缓慢移动** - 提高地图质量
- 📏 **覆盖完整区域** - 包含所有需要导航的地方
- ⏱️ **建图时间** - 至少 5-10 分钟，确保回环检测生效

#### 步骤 4：检查回环检测

在 RViz 中查看：
- **绿色路径** - FAST-LIO2 原始轨迹
- **红色路径** - PGO 优化后的轨迹
- 如果看到红色路径与绿色路径对齐，说明回环检测成功！

#### 步骤 5：保存地图

##### 5.1 保存点云地图（3D PCD）

在终端中运行：

```bash
cd ~/Code/Demo8
source install/setup.bash
cd src/megarover3_ros2/megarover3_navigation/maps

# 保存PGO优化后的地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$(pwd)/my_pgo_map', save_patches: true}"
```

这会生成：
- `my_pgo_map.pcd` - 优化后的全局点云地图
- `my_pgo_map_patches/` - 地图块（用于HBA优化，可选）

##### 5.2 保存 2D 栅格地图

```bash
# 使用自定义保存脚本
python3 ~/Code/Demo8/save_map_now.py my_pgo_map
```

这会生成：
- `my_pgo_map.yaml` - 地图配置
- `my_pgo_map.pgm` - 地图图像

**最终文件：**
```
maps/
├── my_pgo_map.pcd      ← FastLIO2/Localizer 使用
├── my_pgo_map.yaml     ← Nav2 使用
├── my_pgo_map.pgm      ← Nav2 使用
└── my_pgo_map_patches/ ← HBA 优化使用（可选）
```

---

### 3. 导航流程（PGO 地图）

#### 步骤 1：停止建图模式

点击 **SLAM (建图-PGO优化)** - 停止按钮

#### 步骤 2：启动底盘和激光雷达

确保以下组件正在运行：
1. ✅ 底盘 (Micro-ROS)
2. ✅ MID360 激光雷达
3. ✅ D455 前方相机（可选）

#### 步骤 3：启动 PGO 导航

点击 **导航 (PGO地图)** - 启动按钮

系统会弹出两个文件选择对话框：

1. **选择2D地图文件** - 选择 `my_pgo_map.yaml`
2. **选择点云地图文件** - 选择 `my_pgo_map.pcd`

#### 步骤 4：等待重定位

系统会自动进行重定位：

```bash
# 如果需要手动设置初始位置，使用服务：
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$(pwd)/maps/my_pgo_map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

#### 步骤 5：在 RViz 中设置导航目标

1. 在 RViz 工具栏中选择 **"2D Goal Pose"**
2. 在地图上点击并拖动设置目标点和方向
3. 机器人会自动规划路径并导航到目标点

---

## 🛠️ 命令行使用（高级）

### 建图模式

```bash
cd ~/Code/Demo8
source install/setup.bash

# 启动底盘
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4 &

# 启动激光雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py &

# 启动 PGO 建图
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam
```

### 保存地图

```bash
# 保存点云地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/my_map', save_patches: true}"

# 保存2D地图
cd ~/Code/Demo8
python3 save_map_now.py maps/my_map
```

### 导航模式

```bash
cd ~/Code/Demo8
source install/setup.bash

# 启动底盘
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4 &

# 启动激光雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py &

# 启动 PGO 导航
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py \
  mode:=nav \
  map:=/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/my_map.yaml \
  pcd_map:=/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/maps/my_map.pcd
```

---

## 📊 对比：简单 SLAM vs PGO 优化

| 特性 | 简单 SLAM | PGO 优化 |
|------|----------|---------|
| 回环检测 | ❌ 无 | ✅ 有 |
| 累积误差 | ⚠️ 存在 | ✅ 优化消除 |
| 地图质量 | 中等 | 高 |
| 定位方式 | 实时SLAM | 基于地图定位 |
| 定位精度 | 中等 | 高 |
| 适用场景 | 探索、动态环境 | 已知环境、重复导航 |
| 地图保存 | 仅2D地图 | 2D+3D地图 |

---

## 🔧 故障排除

### 问题1：PGO 节点启动失败

**检查：**
```bash
ros2 node list | grep pgo
```

**解决：**
- 确保 `pgo` 包已编译
- 检查配置文件路径

### 问题2：回环检测不触发

**原因：**
- 建图时间太短（< 5分钟）
- 没有经过同一地点多次
- 移动太快，点云质量差

**解决：**
- 增加建图时间
- 多次经过关键区域
- 降低移动速度

### 问题3：Localizer 重定位失败

**检查：**
```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

**解决：**
- 手动设置初始位置（使用 `/localizer/relocalize` 服务）
- 确保点云地图文件正确加载
- 检查机器人当前位置是否在地图范围内

### 问题4：导航时定位漂移

**原因：**
- 环境发生变化（动态障碍物）
- 点云地图质量不佳

**解决：**
- 重新建图，确保回环检测成功
- 在静态环境中建图
- 检查激光雷达数据质量

---

## 📁 文件说明

### maps/ 目录结构

```
maps/
├── my_pgo_map.pcd          # 3D点云地图（Localizer使用）
├── my_pgo_map.yaml         # 2D地图配置（Nav2使用）
├── my_pgo_map.pgm          # 2D地图图像（Nav2使用）
├── my_pgo_map_patches/     # 地图块（HBA优化使用）
│   ├── patch_0000.pcd
│   ├── patch_0001.pcd
│   └── ...
└── notes                   # 地图说明文件
```

### 地图文件大小参考

- **PCD 点云地图**：通常 5-50 MB（取决于环境大小）
- **2D 栅格地图**：通常 100-500 KB
- **地图块**：每块约 1-5 MB

---

## 💡 最佳实践

### 建图建议

1. **环境准备**
   - 🔆 光线充足（D455相机需要）
   - 🚫 移除动态障碍物
   - 📏 规划建图路径

2. **建图过程**
   - 🐢 缓慢移动（< 0.5 m/s）
   - 🔄 多次经过关键区域（至少2-3次）
   - ⏱️ 建图时间 > 10分钟
   - 👀 实时监控 RViz

3. **质量检查**
   - ✅ 检查回环检测是否触发
   - ✅ 检查地图是否闭合
   - ✅ 检查点云密度是否均匀

### 导航建议

1. **初次启动**
   - 🎯 手动设置初始位置（使用 relocalize 服务）
   - ⏱️ 等待定位收敛（约 10-30 秒）

2. **日常使用**
   - 📍 在地图已知位置启动
   - 🔄 定期检查定位精度
   - 🚫 避免在地图外部区域导航

---

## 🎉 总结

**您现在拥有一个完整的 PGO 优化建图和导航系统！**

**关键命令速查：**

```bash
# 启动控制面板
~/megarover3_panel

# 保存PGO地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$(pwd)/my_map', save_patches: true}"

# 保存2D地图
python3 ~/Code/Demo8/save_map_now.py my_map

# 手动重定位
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '$(pwd)/my_map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

祝您建图和导航顺利！🚀
