# t_robot_slam

RTAB-Map SLAM and mapping utilities for the T-Robot platform equipped with the Megarover3 chassis and Livox MID360 LiDAR. This package is being restructured from an earlier FAST-LIO workflow; only the utilities that remain relevant are kept while new RTAB-Map launch files and parameters are added.

## Scope
- Launch files for RTAB-Map mapping/localization (to be implemented)
- Point cloud preprocessing helpers for MID360 (voxel filtering, ground removal)
- Map management scripts for saving/loading RTAB-Map databases and 2D occupancy grids
- RViz configurations for monitoring SLAM status

## Status
| Component | State |
|-----------|-------|
| Legacy FAST-LIO artifacts | ♻️ Isolated under legacy docs | 
| RTAB-Map mapping launch | ⬜ Planned |
| RTAB-Map localization launch | ⬜ Planned |
| Parameter set (`params/rtabmap.yaml`) | ⬜ Planned |
| Map management scripts | ⬜ Needs review |
| Nav2 integration bridge | ⬜ Planned |

## Planned Layout (subject to change)
```
t_robot_slam/
├── launch/
│   ├── mapping.launch.py        # RTAB-Map mapping mode
│   ├── localization.launch.py   # Localization using existing map
│   └── rtabmap_with_nav2.launch.py
├── params/
│   ├── rtabmap.yaml             # Core RTAB-Map parameters
│   └── filters.yaml             # Preprocessing filter configuration
├── scripts/
│   ├── mapping_recorder.py
│   ├── map_manager.py
│   └── ...
└── rviz/
    └── rtabmap_monitor.rviz
```

## Next Steps
1. Port or create point cloud preprocessing nodes for MID360 (voxel grid, height filter).
2. Write `mapping.launch.py` that launches RTAB-Map together with the required filters.
3. Provide localization launch that loads a saved database and publishes `/map`.
4. Update scripts to operate on RTAB-Map outputs (e.g., export occupancy grid).
5. Integrate Nav2 by generating 2D map and ensuring consistent TF frames.

## Notes
- Historical FAST-LIO launch files now live in `launch/*` with explicit deprecation
  notices. They are kept only as reference until the RTAB-Map equivalent is ready.
- The package depends on `t_robot_bringup` for bringup of sensors and TF; ensure the bringup stack provides `/odom` and `/tf` before launching SLAM.
- MID360 driver must publish `sensor_msgs/msg/PointCloud2` at a stable rate. Adjust down-sampling to keep the Jetson workload manageable.

---
*Work in progress – RTAB-Map integration coming next.*
