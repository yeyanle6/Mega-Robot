# 脚本工具说明

本目录包含Mega-Robot项目的核心脚本工具。

---

## 📋 脚本列表

### 🚀 启动脚本

#### `start_navigation.sh`
**用途**: 交互式启动导航系统（推荐使用）

**功能**:
- 自动检测传感器硬件
- 交互式选择传感器模式（Fusion/LiDAR Only/RGB-D Only）
- 启动对应的SLAM系统

**使用**:
```bash
./scripts/start_navigation.sh
```

---

### 🧪 测试脚本

#### `test_new_lidar_slam.sh`
**用途**: 测试重构后的LiDAR SLAM系统

**功能**:
- 验证rtabmap_lidar_slam.launch.py
- 检查配置文件完整性
- Python语法验证

**使用**:
```bash
./scripts/test_new_lidar_slam.sh
```

#### `test_navigation.sh`
**用途**: 测试Nav2导航功能

**功能**:
- 测试路径规划
- 测试避障
- 测试目标点导航

**使用**:
```bash
./scripts/test_navigation.sh
```

#### `test_build.sh`
**用途**: 测试编译系统

**功能**:
- 清理build目录
- 编译所有包
- 验证编译结果

**使用**:
```bash
./scripts/test_build.sh
```

---

### 🛠️ 工具脚本

#### `cleanup_all_nodes.sh`
**用途**: 清理所有ROS2和传感器节点

**功能**:
- 停止所有ROS2节点
- 清理Livox驱动进程
- 清理RealSense驱动进程
- 清理RTAB-Map进程

**使用**:
```bash
./scripts/cleanup_all_nodes.sh
# 或显示详细信息
./scripts/cleanup_all_nodes.sh true
```

#### `install_dependencies.sh`
**用途**: 安装项目依赖

**功能**:
- 安装ROS2依赖
- 安装传感器驱动依赖
- 安装编译工具

**使用**:
```bash
./scripts/install_dependencies.sh
```

---

## 📝 使用建议

### 首次使用
```bash
# 1. 安装依赖
./scripts/install_dependencies.sh

# 2. 测试编译
./scripts/test_build.sh

# 3. 启动系统
./scripts/start_navigation.sh
```

### 日常使用
```bash
# 启动导航系统
./scripts/start_navigation.sh

# 如果遇到问题，先清理节点
./scripts/cleanup_all_nodes.sh
```

### 开发测试
```bash
# 测试SLAM
./scripts/test_new_lidar_slam.sh

# 测试导航
./scripts/test_navigation.sh

# 测试编译
./scripts/test_build.sh
```

---

## 🗑️ 已删除的历史脚本

以下脚本已在2025-10-27删除（架构重构后不再需要）：

- `diagnose_pointcloud.sh` - 点云问题已解决
- `fix_rtabmap_pointcloud.sh` - 点云问题已解决
- `test_rtabmap_tf_fix.sh` - TF问题已解决
- `test_slam_mapping.sh` - 被test_new_lidar_slam.sh替代
- `test_mid360_orientations.sh` - 配置已确定
- `verify_mid360_tilt.sh` - 配置已写入URDF
- `visualize_mid360_tilt.sh` - 不再需要
- `test_odometry_fusion.sh` - 功能已移除
- `launch_navigation.sh` - 与start_navigation.sh重复
- `verify_all_fixes.sh` - 修复已完成
- `test_basic_functions.sh` - 功能已整合

---

**维护者**: Claude Code
**最后更新**: 2025-10-27
