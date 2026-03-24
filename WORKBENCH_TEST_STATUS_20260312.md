## Navigation Workbench Real-Robot Regression

Date: March 12, 2026

## Scope

This record captures the first stable real-robot regression run using the new Qt navigation workbench.

Relevant components:

- robot main system via [fastlio2_pgo_navigation.launch.py](/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/launch/fastlio2_pgo_navigation.launch.py)
- workbench entrypoint [nav_test_workbench.py](/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/scripts/nav_test_workbench.py)
- latest map [map.yaml](/home/ros/Code/Demo8/maps/new_map_20260310_0918/map.yaml)

## Workbench Validation Status

The following workbench functions were verified on the live robot:

- continuous TF-based robot pose monitoring
- manual map load
- workbench-side `2D Pose Estimate`
- waypoint selection and multi-cycle patrol execution
- real-time path and trajectory recording
- automatic JSON export after batch completion

The workbench remained alive after test completion and continued pose polling successfully.

## Test Configuration

- waypoints: `2`
- patrol order: `1 -> 2`
- cycles: `5`
- total runs: `10`

Output file:

- [nav_test_20260312_120833.json](/home/ros/Code/Demo8/install/megarover3_navigation/lib/megarover3_navigation/test_results/nav_test_20260312_120833.json)

## Summary

- total runs: `10`
- succeeded: `10`
- failed: `0`
- canceled: `0`
- total distance: `79.50 m`
- total time: `268.48 s`
- average deviation: `0.0866 m`
- total recoveries: `0`
- total aborts: `0`
- total stalls: `0`

This batch was clean:

- no recovery behavior was recorded
- no aborts were recorded
- no stall events were recorded

## Per-Run Statistics

- run01: `27.6 s`, actual `7.83 m`, theory `8.35 m`, avg dev `0.115 m`, max dev `0.379 m`
- run02: `29.0 s`, actual `8.34 m`, theory `8.43 m`, avg dev `0.078 m`, max dev `0.191 m`
- run03: `26.0 s`, actual `7.81 m`, theory `8.27 m`, avg dev `0.058 m`, max dev `0.234 m`
- run04: `26.9 s`, actual `7.98 m`, theory `8.39 m`, avg dev `0.060 m`, max dev `0.149 m`
- run05: `26.0 s`, actual `7.86 m`, theory `8.20 m`, avg dev `0.086 m`, max dev `0.212 m`
- run06: `26.9 s`, actual `8.02 m`, theory `8.47 m`, avg dev `0.130 m`, max dev `0.304 m`
- run07: `26.0 s`, actual `7.80 m`, theory `8.20 m`, avg dev `0.066 m`, max dev `0.209 m`
- run08: `27.3 s`, actual `8.05 m`, theory `8.40 m`, avg dev `0.071 m`, max dev `0.193 m`
- run09: `25.9 s`, actual `7.84 m`, theory `8.22 m`, avg dev `0.084 m`, max dev `0.251 m`
- run10: `26.8 s`, actual `7.98 m`, theory `8.47 m`, avg dev `0.118 m`, max dev `0.270 m`

## Aggregate Interpretation

Across the 10 runs:

- duration ranged from `25.88 s` to `29.00 s`
- actual path length ranged from `7.80 m` to `8.34 m`
- theoretical path length ranged from `8.20 m` to `8.47 m`
- average lateral deviation ranged from `0.058 m` to `0.130 m`
- maximum deviation ranged from `0.149 m` to `0.379 m`

The robot consistently followed the planned route with low average deviation and without triggering recovery behavior.

The actual path remained slightly shorter than the theoretical path in all runs, which is acceptable for the current system and consistent with normal path smoothing and tracking behavior.

## Current Assessment

This batch establishes the first reliable baseline for the new navigation workbench on the real robot.

It supports the following conclusions:

- the workbench is now capable of producing usable real-robot regression data
- TF polling, trajectory capture, and automatic export are functioning
- the robot system and workbench can now be used for repeated patrol-style validation

## Next Recommended Use

Use this batch as the baseline before controller A/B testing, especially:

- `Smac + DWB`
- `Smac + MPPI`

The same workbench flow and JSON export format can now be reused for controlled comparisons.
