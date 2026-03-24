# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble workspace for the **MegaRover3** differential-drive robot. The system uses FAST-LIO2 (LiDAR-Inertial Odometry) with a Livox MID-360 LiDAR and Intel D455 depth camera for SLAM and autonomous navigation via Nav2. Runs on a Jetson (Ubuntu 22.04).

## Build Commands

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build entire workspace
cd /home/ros/Code/Demo8
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select megarover3_navigation

# Build a package and its dependencies
colcon build --symlink-install --packages-up-to megarover3_navigation

# Source the workspace after building
source install/setup.bash

# Run linter tests for a package
colcon test --packages-select megarover3_navigation
colcon test-result --verbose
```

## Launch Commands

The **single recommended entry point** for all field operation is `fastlio2_pgo_navigation.launch.py`. Other launch files (`fastlio2_navigation.launch.py`, `fastlio2_localization.launch.py`) are legacy and not recommended.

```bash
# SLAM mode (mapping with PGO loop closure)
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py mode:=slam

# Navigation mode (with pre-built map)
ros2 launch megarover3_navigation fastlio2_pgo_navigation.launch.py \
  mode:=nav pcd_map:=/path/to/map.pcd map:=/path/to/map.yaml

# If LiDAR is already running from another terminal/control panel
# add: start_lidar:=false

# A/B test with alternative Nav2 params
# add: nav2_params_file:=/path/to/alternative_params.yaml
```

The chassis micro-ROS agent must be started separately first:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200 -v4
```

## GUI Applications

```bash
# Control panel (component launcher + log viewer)
~/megarover3_panel
# or: python3 src/megarover_control_panel_qt/scripts/control_panel.py

# Navigation test workbench (waypoint patrol + trajectory recording)
python3 src/megarover_control_panel_qt/scripts/nav_test_workbench.py
```

## Architecture

### Source Packages (src/)

**Custom packages:**
- `megarover3_ros2/megarover3_navigation` — Main navigation package: launch files, Nav2 configs, Python processing nodes
- `megarover3_ros2/megarover_description` — URDF/Xacro robot model, sensor mounts, TF definitions
- `megarover3_ros2/megarover3_bringup` — Robot bringup (chassis + base sensor startup)
- `megarover_control_panel_qt` — Qt5 control panel + navigation test workbench
- `map_repair_qt` — Qt5 2D occupancy map editor

**Integrated third-party packages:**
- `FASTLIO2_ROS2/` — FAST-LIO2 LiDAR-inertial odometry (fastlio2, pgo, localizer, hba, interface)
- `livox_ros_driver2` — Livox MID-360 driver
- `patchwork-plusplus` — Ground segmentation (C++)
- `spatio_temporal_voxel_layer` — STVL costmap plugin for Nav2
- `octomap_server2-master` — 3D OctoMap server
- `uros/` + `micro_ros_setup` — Micro-ROS agent for chassis MCU communication

### Point Cloud Processing Pipeline

```
MID-360 → FAST-LIO2 → /body_cloud (lio_base frame)
  → Patchwork++ → /patchworkpp/nonground (base_link frame)
    → pointcloud_relay.py (self-filter + merge) → /nonground_filtered

D455 → /camera/d455_front/depth/color/points
  → d455_nearfield_obstacle_filter.py → /d455_front_obstacles (base_footprint frame)

Both feeds → STVL costmap layers (local + global)
Merged cloud → OctoMap → 3D visualization
```

### Coordinate Frame Convention (CRITICAL)

- **base_link = lio_base**: +X = RIGHT, +Y = FORWARD, +Z = UP (non-standard)
- **base_footprint**: +X = FORWARD (ROS standard), rotated 90° yaw from lio_base
- TF chain: `map → odom → lio_base → base_footprint → base_link → sensors`
- FAST-LIO2 `body_cloud` frame_id = `lio_base`; Patchwork++ output frame_id = `base_link`

### Key Python Nodes (megarover3_navigation/scripts/)

| Node | Purpose |
|------|---------|
| `pointcloud_relay.py` | Fuses LiDAR + D455, self-filters robot body points, timestamps relay |
| `d455_nearfield_obstacle_filter.py` | Distance-bin ground baseline method for near-field obstacles |
| `nav_initializer.py` | Loads PCD maps, publishes initial pose, calls localizer relocalize |
| `bumper_safety_monitor.py` | Emergency stop on bumper contact, retreat logic |
| `calibrate_d455.py` | D455 extrinsic auto-calibration (ground/wall/full/verify modes) |

### Control Panel Architecture (megarover_control_panel_qt/)

```
scripts/control_panel.py          — Entry point
megarover_control_panel_qt/
  main_window.py                  — Main GUI window
  process_manager.py              — Spawns/kills ROS2 processes
  ros2_monitor.py                 — Real-time node/topic monitoring
  config/panel_config.yaml        — Component definitions (commands, dependencies, health checks)
  nav_workbench/
    workbench_window.py           — Nav test workbench GUI
    test_engine.py                — Headless ROS2 test runner (QThread)
    map_widget.py                 — Map + 4-layer trajectory overlay
```

### Key Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| `fastlio2_nav2_params.yaml` | megarover3_navigation/config/ | Default Nav2 costmap/planner/controller params |
| `fastlio2_nav2_params_mppi.yaml` | same | MPPI controller variant for A/B testing |
| `lio_megarover.yaml` | FASTLIO2_ROS2/fastlio2/config/ | FAST-LIO2 config (lidar range, extrinsics) |
| `calibration_offsets.xacro` | megarover_description/urdf/ | Sensor mount positions (D455, MID360) |
| `panel_config.yaml` | megarover_control_panel_qt/config/ | Control panel component definitions |
| `mega3.xacro` | megarover_description/urdf/ | Main robot URDF model |

### Maps

Stored in `maps/` as paired files:
- `.pcd` — 3D point cloud (used by FAST-LIO2 localizer)
- `.pgm` + `.yaml` — 2D occupancy grid (used by Nav2 map_server)

## Platform Notes

- D455 on Jetson requires `LD_PRELOAD=/usr/local/lib/librealsense2.so` (see `start_d455_front.sh`)
- Chassis micro-ROS baudrate is **115200** (not 921600)
- Use `--qos-reliability best_effort` when echoing sensor topics
- Patchwork++ `min_range` set to 0.5m in all launch files
- STVL voxel decay: local=5-8s, global=30s
