# T-Robot SLAM 实测手册

本手册整合了原有的 QUICKSTART、READY_TO_TEST、TEST_PROCEDURE、NETWORK_CONFIG 等文档内容，用于指导 Megarover3 + MID360 平台的现场 Bringup / Mapping / Navigation 测试。

## 1. 快速启动总览

```bash
cd ~/Code/Demo5

# 0. 确认 micro-ROS Agent 运行（串口桥，详见 §2.2）
./pre_test_check.sh            # 环境自检，应全部 PASS
./test_bringup.sh              # Bringup + 传感器 + EKF
# 新终端
./test_mapping.sh              # 点云预处理 + RTAB-Map
# 再开终端（可选）
./test_navigation.sh           # 地图导出 + Nav2
```

运行期间脚本会保持前台并在 Ctrl+C 后自动清理相关节点、重启 `ros2 daemon`，无需额外“环境重置”命令。

## 2. 测试前准备

### 2.1 硬件与网络

| 模块 | 配置 | 状态 |
|------|------|------|
| MID360 LiDAR | IP `192.168.1.3`, MAC `e4:7a:2c:a9:e7:52` | ✓ 已验证可达 |
| 主机有线 (eno1) | IP `192.168.1.5/24` | ✓ 与 MID360 同网段 |
| 主机 WiFi (wlP1p1s0) | IP `192.168.50.15/23` | ✓ 仅用于外网 |
| Megarover3 底盘 | `/dev/ttyUSB0` micro-ROS 串口 | ✓ 需上电后连接 |

常用网络检查：
```bash
ping -c 3 192.168.1.3
ip addr show eno1 | grep 192.168.1
ethtool eno1 | grep "Link detected"
```

Livox 驱动参数已写入 `src/t_robot_bringup/params/livox_driver.yaml`（主机 IP 192.168.1.5，MID360 IP 192.168.1.3）。

### 2.2 micro-ROS Agent（底盘串口桥）

- Megarover3 底盘通过 micro-ROS Agent 将 `/dev/ttyUSB0` 上的 `/rover_odo` 数据桥接到 ROS 2。
- `test_bringup.sh` 会尝试自动启动 `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4`，并记录日志 `/tmp/micro_ros_agent.log`。
- 如脚本提示未找到 micro-ROS workspace，可手动执行：
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/wang/uros_ws/install/setup.bash
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v4
  ```
- Bringup 阶段需确保 Agent 持续运行，`/rover_odo → /odom → /odometry/filtered` 才会有数据。

### 2.3 `pre_test_check.sh` 自检

脚本会检查 ROS 环境、依赖包、网络、串口、脚本可执行权限等。目前全部 17 项均通过：
```
Passed: 17
Failed: 0
```
如有 FAIL，可参考 §5 故障排查。

## 3. 实测流程

### 3.1 Bringup（传感器 + EKF，约 10–15 分钟）

```bash
./test_bringup.sh
```
脚本会：
1. 确认/启动 micro-ROS Agent。
2. 运行 `t_robot_bringup/bringup.launch.py`。
3. 检查关键节点与话题：`/mid360/lidar`、`/mid360/imu`、`/odom`、`/odometry/filtered`、TF 树。
4. 生成 `frames.pdf`（TF 结构）并输出频率统计。

保持窗口，观察 `/rover_odo` 数值；底盘静止时速度为 0，移动后应传递到 `/odom` 和 `/odometry/filtered`。按 Ctrl+C 结束，脚本会清理节点并重启 ROS 2 daemon。

### 3.2 Mapping（点云预处理 + RTAB-Map，约 20–30 分钟）

```bash
./test_mapping.sh
```
脚本会在确认 Bringup 运行后，额外启动：
- `t_robot_slam/pointcloud_preprocessor`（包含 3D 半径裁剪、TF 变换、障碍/地面输出）。
- `rtabmap_ros/rtabmap`。

待初始化 15 秒后，可在终端看到 `/mid360/points_filtered`、`/cloud/obstacles`、`/rtabmap` 相关话题。缓慢驱动底盘（建议线速度 < 0.2 m/s），实时监控：
```bash
ros2 topic echo /rtabmap/info --once       # 查看节点/回环信息
ros2 run t_robot_slam slam_monitor.py      # 性能监控
ros2 run t_robot_slam mapping_recorder.py  # 可录制 rosbag
```
Ctrl+C 退出后，脚本会自动停止相关节点并刷新 daemon。

### 3.3 Navigation（地图导出 + Nav2，约 15–20 分钟）

```bash
./test_navigation.sh
```
流程：
1. 检查 `~/.ros/rtabmap.db`，并导出地图至 `/tmp/rtabmap_export/map.pgm` & `.yaml`。
2. 若 Bringup 未运行，脚本会临时启动一次。
3. 启动 Nav2 （使用 `t_robot_slam/config/navigation/nav2_params.yaml`）并等待节点就绪。
4. 提示在 RViz 中设定初始位姿、发送导航目标或使用 CLI：
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}}'
   ```

观察 `/local_costmap`、`/global_costmap`、`/navigate_to_pose` 等话题。退出同样使用 Ctrl+C，脚本会清理 Nav2 与临时 Bringup。

## 4. 监控与日志

- Bringup 日志：`/tmp/bringup_test.log`
- Mapping 日志：`/tmp/mapping_test.log`
- Navigation 日志：`/tmp/nav2_test.log`
- micro-ROS Agent 日志：`/tmp/micro_ros_agent.log`
- 导出地图：`/tmp/rtabmap_export/`
- RTAB-Map 数据库：`~/.ros/rtabmap.db`
- 录制数据：`/tmp/slam_recordings/`

常用监控命令：
```bash
ros2 topic hz /mid360/lidar                 # 原始点云频率 (~10Hz)
ros2 topic hz /mid360/points_filtered       # 过滤后点云频率
ros2 topic hz /odometry/filtered            # EKF 输出 (~50Hz)
ros2 topic echo /rtabmap/info | grep loop   # 回环检测
ros2 run tf2_tools view_frames              # 生成 TF 树
```

## 5. 故障排查与修复

### 5.1 常见问题

| 问题 | 现象 | 解决方案 |
|------|------|----------|
| rtabmap_ros 缺失 | 自检 FAIL，`ros2 pkg list` 找不到 | `sudo apt install ros-humble-rtabmap-ros` |
| MID360 无数据 | `/mid360/lidar` 没有消息 | 检查网线、IP、`ping 192.168.1.3`，重启传感器 |
| EKF 无输出 | `/odometry/filtered` 无数据 | 确认 micro-ROS Agent 运行、`/rover_odo` 有速度输入 |
| QoS 不匹配 | EKF 订阅失败 | 已通过 `imu_relay.py` 修复，确保脚本在装置上运行 |
| 节点重复 | `/odom` 多 publisher | 已在 `base.launch.py` 清理，只保留 megarover driver include |

更多网络排查命令请参考 §2.1 中的示例。

### 5.2 重要修复记录

- 话题命名统一：所有配置使用 `/mid360/*`（驱动话题通过 relay 适配）。
- IMU QoS 修复：`imu_relay.py` 将 `/livox/imu` 转换为可靠的 `/mid360/imu`（可选镜像 `/imu/data`）。
- 底盘重复节点移除：`base.launch.py` 只 include 官方 bringup。
- 点云预处理增强：`pointcloud_preprocessor.cpp` 实现 3D 半径裁剪 + TF 变换。
- Nav2 costmap 使用 `/cloud/obstacles` 的 3D 点云层。

## 6. 当前系统状态概览（2025-10-08）

- 软件配置已全部完成并通过 Bringup 自检。
- `/rover_odo → /odom → /odometry/filtered` 数据链在底盘静止时保持 0，等待实际移动验证。
- MID360 点云、IMU、预处理、RTAB-Map 运行正常。
- 待完成：现场建图后的地图导出与 Nav2 多点任务验证（参见 `project_plan.md` 当前待办）。

## 7. 参考资料

- `project_plan.md` —— 阶段目标、里程碑与待办。
- `pre_test_check.sh` —— 环境巡检脚本。
- `test_bringup.sh` / `test_mapping.sh` / `test_navigation.sh` —— 自动化测试脚本。

如需追溯更早的修复详情，可查看 Git 历史或 `BRINGUP_TEST_REPORT.md`（已合并入本手册相关章节）。
