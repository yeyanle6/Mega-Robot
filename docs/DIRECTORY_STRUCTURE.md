# 目录结构详解

本文档详细说明 Mega-Robot 项目的目录结构和各文件的作用。

## 📂 根目录结构

```
Demo5/
├── src/                    # ROS 2 软件包源代码
├── docs/                   # 项目文档
├── scripts/                # 自动化测试与工具脚本
├── build/                  # 编译输出目录 [.gitignore]
├── install/                # 安装目录 [.gitignore]
├── log/                    # ROS 2 日志 [.gitignore]
├── README.md               # 项目主文档
├── .gitignore              # Git 忽略规则
└── LICENSE                 # 许可证文件
```

---

## 📦 src/ - ROS 2 软件包

### 自研包

#### `t_robot_bringup/` - 机器人启动配置包
**功能**: 提供机器人硬件驱动、传感器启动和状态估计的统一入口。

```
t_robot_bringup/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS 2 包清单
├── README.md               # 包说明文档
├── launch/                 # Launch 文件（分层架构）
│   ├── bringup.launch.py           # 主入口（调用下面三个）
│   ├── base.launch.py              # 底盘驱动（Megarover3 + pub_odom）
│   ├── sensors.launch.py           # 传感器驱动（MID360 + IMU relay）
│   └── state_estimation.launch.py  # 状态估计（EKF + 静态 TF）
├── params/                 # 参数文件
│   ├── ekf.yaml                    # robot_localization EKF 配置
│   └── livox_driver.yaml           # Livox MID360 驱动参数
├── config/                 # 配置文件
│   ├── static_tf.yaml              # 静态 TF 声明
│   ├── filters.yaml                # 点云滤波器参数
│   └── twist_mux.yaml              # 速度多路复用（可选）
├── scripts/                # Python 工具脚本
│   ├── imu_relay.py                # IMU QoS 转换节点
│   └── time_sync_monitor.py        # 时间同步监控
└── rviz/                   # RViz 配置
    └── mid360_calibration.rviz     # 传感器校准视图
```

**关键文件说明**:
- `launch/bringup.launch.py`: 启动完整系统，参数 `include_base`, `include_sensors`, `include_ekf` 可独立控制各模块
- `params/ekf.yaml`: EKF 配置中 `publish_tf: false` 是关键设置，避免 TF 发布冲突
- `scripts/imu_relay.py`: 将 Livox `/livox/imu` (BEST_EFFORT) 转换为 `/mid360/imu` (BEST_EFFORT) 以适配 robot_localization

---

#### `t_robot_slam/` - SLAM 与导航包
**功能**: 集成 RTAB-Map 3D SLAM、点云预处理和 Nav2 导航。

```
t_robot_slam/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS 2 包清单
├── README.md               # 包说明文档
├── src/                    # C++ 源代码
│   ├── pointcloud_preprocessor.cpp        # 点云预处理节点
│   ├── map_odom_tf_publisher.cpp          # TF 发布工具（备用）
│   └── map_odom_tf_publisher_v2.cpp       # TF 发布工具 v2（备用）
├── launch/                 # Launch 文件
│   ├── mapping.launch.py                  # RTAB-Map 建图模式
│   ├── localization.launch.py             # RTAB-Map 定位模式
│   ├── navigation.launch.py               # Nav2 导航
│   ├── preprocessing.launch.py            # 点云预处理独立启动
│   ├── complete_navigation.launch.py      # 完整导航（包含 bringup）
│   ├── simple_navigation.launch.py        # 简化导航
│   ├── pointcloud_to_laserscan.launch.py  # 2D 激光转换（备用）
│   └── [其他实验性 launch 文件]
├── params/                 # 参数文件
│   └── rtabmap.yaml                       # RTAB-Map 核心配置（60+ 参数）
├── config/                 # 配置文件
│   ├── preprocessing_params.yaml          # 点云预处理参数
│   └── navigation/                        # Nav2 配置目录
│       └── nav2_params.yaml               # Nav2 参数（Costmap + 规划器）
├── scripts/                # Python 工具脚本
│   ├── mapping_recorder.py                # 建图录制工具
│   ├── map_manager.py                     # 地图管理脚本
│   ├── slam_monitor.py                    # SLAM 性能监控
│   ├── rtabmap_monitor.py                 # RTAB-Map 专用监控
│   ├── pcd_to_pgm.py                      # 点云转 2D 地图
│   ├── mid360_map_converter.py            # MID360 地图转换
│   ├── advanced_pcd_converter.py          # 高级点云转换
│   └── [其他转换工具]
└── rviz/                   # RViz 配置
    ├── mapping.rviz                       # 建图模式可视化（建议保存）
    ├── navigation.rviz                    # 导航模式可视化
    ├── map_fixed.rviz                     # 固定地图视图
    └── simple_map.rviz                    # 简化地图视图
```

**关键文件说明**:
- `src/pointcloud_preprocessor.cpp`: 实现 3D 半径裁剪、体素下采样、地面分割和 TF 变换
  - 输入: `/livox/lidar`
  - 输出: `/mid360/points_filtered`, `/cloud/obstacles`, `/cloud/ground`
- `params/rtabmap.yaml`:
  - `Reg/Strategy: 1` - 纯 ICP 配准
  - `approx_sync: true` - 近似时间同步（关键设置）
  - `Grid/CellSize: 0.05` - 5cm 栅格地图分辨率
- `scripts/mapping_recorder.py`: 录制 rosbag 并监控建图进度

---

### 第三方包

#### `megarover3_ros2/` - Megarover3 底盘驱动
**来源**: [VerteXobotics/megarover_ros2](https://github.com/vstoneofficial/megarover_ros2)

```
megarover3_ros2/
├── megarover3/                     # 元包
├── megarover3_bringup/             # 启动配置
│   ├── launch/
│   │   ├── robot.launch.py                 # 底盘主启动
│   │   ├── robot_base_only.launch.py       # 仅底盘驱动
│   │   └── [其他 launch 文件]
│   ├── params/
│   └── src/                                # C++ 驱动节点
└── megarover_description/          # URDF 机器人描述
    └── urdf/
        └── mega3.xacro                     # Megarover3 URDF 模型
```

**使用方式**: 在 `t_robot_bringup/launch/base.launch.py` 中通过 `IncludeLaunchDescription` 调用。

---

#### `livox_ros_driver2/` - Livox MID360 驱动
**来源**: [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

```
livox_ros_driver2/
├── src/                            # C++ 驱动源码
├── launch_ROS2/
│   ├── msg_MID360_launch.py                # MID360 标准启动
│   └── rviz_MID360_launch.py               # 带 RViz 的启动
├── config/
│   └── MID360_config.json                  # MID360 配置文件
└── package.xml
```

**话题输出**:
- `/livox/lidar` - 点云数据 (sensor_msgs/PointCloud2)
- `/livox/imu` - IMU 数据 (sensor_msgs/Imu)

---

#### `realsense-ros/` - Intel RealSense 驱动（可选）
**来源**: [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)

```
realsense-ros/
├── realsense2_camera/              # 相机驱动
├── realsense2_description/         # URDF 描述
└── realsense2_camera_msgs/         # 自定义消息
```

**状态**: 已预留，暂未集成到 bringup 流程。

---

#### `vs_rover_options_description/` - 配件 URDF 描述
**来源**: [VerteXobotics](https://github.com/vstoneofficial/)

```
vs_rover_options_description/
├── urdf/                           # 配件 URDF（保险杠、支架等）
└── meshes/                         # 3D 网格文件
```

---

## 📚 docs/ - 项目文档

```
docs/
├── README_TEST.md                  # 完整测试手册
├── project_plan.md                 # 项目规划与里程碑
├── AGENTS.md                       # 开发者规范
└── DIRECTORY_STRUCTURE.md          # 本文件
```

**文件说明**:
- `README_TEST.md`: 包含硬件配置、网络设置、实测流程和故障排查
- `project_plan.md`: 详细的阶段目标、待办事项和技术决策记录
- `AGENTS.md`: 代码规范、构建命令、Git 提交规范

---

## 🛠️ scripts/ - 自动化脚本

```
scripts/
├── pre_test_check.sh               # 环境自检（17 项检查）
├── test_bringup.sh                 # Bringup 自动化测试
├── test_mapping.sh                 # 建图自动化测试
├── test_navigation.sh              # 导航自动化测试
├── reset_ros_env.sh                # ROS 环境重置
└── fix_issues.sh                   # 常见问题修复脚本
```

**使用流程**:
1. `./scripts/pre_test_check.sh` - 确认环境 OK
2. `./scripts/test_bringup.sh` - 启动底盘和传感器
3. `./scripts/test_mapping.sh` - 启动建图
4. `./scripts/test_navigation.sh` - 启动导航

**特性**:
- 自动检查依赖和硬件连接
- 后台启动 micro-ROS Agent
- Ctrl+C 自动清理所有节点
- 日志输出到 `/tmp/*.log`

---

## 🗂️ 编译与安装目录（.gitignore）

### `build/` - 编译中间文件
- CMake 生成的中间文件
- 编译产物（.o, .so 等）
- 大小: ~51MB

### `install/` - 安装目录
- 编译后的可执行文件
- Python 模块和 Launch 文件的安装副本
- 大小: ~16MB

### `log/` - ROS 2 运行日志
- 节点标准输出/错误日志
- 按时间戳组织的日志目录
- 大小: ~5.1MB

**注意**: 这三个目录都在 `.gitignore` 中，不会提交到 Git。

---

## 📄 配置文件

### `.gitignore`
忽略规则包括:
- ROS 2 构建产物 (`build/`, `install/`, `log/`)
- Python 缓存 (`__pycache__/`, `*.pyc`)
- IDE 配置 (`.vscode/`, `.idea/`)
- 临时文件 (`*.log`, `*.tmp`, `frames*.pdf`)
- 测试输出 (`/tmp/rtabmap_export/`, `*.bag`)

### `LICENSE`
Apache License 2.0 - 开源许可证

---

## 🔑 关键路径速查

| 用途 | 路径 |
|------|------|
| 主 Launch | `src/t_robot_bringup/launch/bringup.launch.py` |
| EKF 配置 | `src/t_robot_bringup/params/ekf.yaml` |
| RTAB-Map 配置 | `src/t_robot_slam/params/rtabmap.yaml` |
| 点云预处理源码 | `src/t_robot_slam/src/pointcloud_preprocessor.cpp` |
| 建图 Launch | `src/t_robot_slam/launch/mapping.launch.py` |
| Nav2 配置 | `src/t_robot_slam/config/navigation/nav2_params.yaml` |
| 测试脚本 | `scripts/test_*.sh` |
| 环境检查 | `scripts/pre_test_check.sh` |
| 完整测试手册 | `docs/README_TEST.md` |
| 项目规划 | `docs/project_plan.md` |

---

## 📊 文件统计

```bash
# 统计代码行数（排除第三方包）
find src/t_robot_* -name "*.cpp" -o -name "*.py" -o -name "*.yaml" | xargs wc -l

# 查看目录大小
du -sh src/*

# 统计 launch 文件数量
find src -name "*.launch.py" | wc -l
```

---

## 🔄 更新记录

- **2025-10-16**: 创建本文档，整理项目结构
- **2025-10-10**: 完成 RTAB-Map 静止测试
- **2025-10-08**: 完成 Bringup 系统验证
- **2025-10-07**: TF 树架构修复
- **2025-10-06**: 初始项目结构搭建

---

*本文档随项目开发持续更新*
