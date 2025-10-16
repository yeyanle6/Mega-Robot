# t_robot_bringup

Bringup utilities for the Megarover3 + MID360 platform. This package aggregates launch files and parameters that start the base robot, sensor drivers, TF tree, and state estimation nodes required by RTAB-Map / Nav2.

## Objectives
- Provide a single entry-point launch (`bringup.launch.py`) that starts chassis drivers, MID360 driver, TF publisher, and `robot_localization`.
- Offer modular includes for sensors-only / base-only startup.
- Store calibrated transform and filter parameters (e.g., `params/ekf.yaml`, `config/static_transforms.yaml`).
- Supply monitoring helpers such as the existing `time_sync_monitor.py`.

## Current Status
- Legacy FAST-LIO links removed from dependencies.
- Launch/config directories are placeholders and need population.
- Time sync monitor script preserved; further diagnostics TBD.

## Planned Directory Layout
```
t_robot_bringup/
├── launch/
│   ├── bringup.launch.py            # Full system (base + sensors + EKF)
│   ├── sensors.launch.py            # MID360 + auxiliary sensors
│   ├── base.launch.py               # Megarover3 base only
│   └── diagnostics.launch.py        # Optional monitoring tools
├── params/
│   ├── ekf.yaml                     # robot_localization settings
│   └── livox_driver.yaml            # MID360 driver ROS params
├── config/
│   ├── static_tf.yaml               # Static TF declarations if needed
│   ├── filters.yaml                 # Point cloud filter parameters
│   └── twist_mux.yaml               # Optional velocity mux configuration
└── scripts/
    └── time_sync_monitor.py
```

## Immediate Tasks
1. Collect calibration measurements (sensor offsets, IMU orientation) and encode them in URDF or TF configuration.
2. Author `bringup.launch.py` that composes:
   - `megarover3_bringup` nodes for base control/odometry.
   - `livox_ros_driver2` with proper frame IDs.
   - `robot_state_publisher` loading `megarover_description/urdf/mega3.xacro`.
   - `robot_localization` EKF producing `/odom`.
3. Add diagnostics (TF monitor, time sync) to assist in field debugging.
4. Document required environment variables / launch arguments.

## Notes
- Ensure the launch files align with TF names expected by RTAB-Map (`map -> odom -> base_link -> mid360_lidar`).
- When adding parameters, prefer ROS 2 YAML style (`/**:` namespace) for clarity.
- Any future sensors (e.g., RealSense) should be added via optional includes to keep the default bringup lightweight.

---
*Work in progress – RTAB-Map focused bringup*
