## MPPI Status as of March 11, 2026

This document records the current state of the `Smac + MPPI` trial after the first real-robot A/B switch from the existing `Smac + DWB` setup.

## Current Config State

A separate MPPI Nav2 parameter file has been added:

- [fastlio2_nav2_params_mppi.yaml](/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/config/fastlio2_nav2_params_mppi.yaml)

This was created as an A/B configuration and does not overwrite the existing DWB-based main configuration.

The MPPI trial keeps these parts unchanged:

- Smac Hybrid-A* global planner
- local/global costmap structure
- D455 geometric near-field obstacle filter
- YOLO observation layer (currently not fed into costmap)

Only the local controller was changed:

- From `dwb_core::DWBLocalPlanner`
- To `nav2_mppi_controller::MPPIController`

The runtime launch confirmed that the controller actually loaded as:

- `nav2_mppi_controller::MPPIController`

## First MPPI Runtime Result

The first short real-robot smoke test used:

- [two_point_nav_test.py](/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/scripts/two_point_nav_test.py)
- `--cycles 2 --no-require-localized`

Result summary:

- `run_01_goal_a`: `succeeded`
- `run_02_goal_b`: `canceled`

The relevant test outputs are:

- [two_point_nav_20260311_184918_default_p1_p2.log](/home/ros/nav_regression_logs/two_point_nav_20260311_184918_default_p1_p2.log)
- [two_point_nav_20260311_184918_default_p1_p2.jsonl](/home/ros/nav_regression_logs/two_point_nav_20260311_184918_default_p1_p2.jsonl)

The script diagnosis for that run was:

- `diagnosis=bumper_safety_intervention`
- `confidence=high`

## What Happened on run_02_goal_b

The failure was not primarily caused by planner collapse or loss of command output.

Observed behavior:

- The robot initially moved forward normally.
- Later, the robot yaw rotated to roughly `1.53~1.57 rad` (close to 90 degrees).
- After that, the command became sustained reverse motion:
  - `cmd.linear_x = -0.05`
  - `cmd.angular_z = 0.0`
- This reverse command propagated to:
  - `/rover_twist`
  - `/rover_odo`

So this was not a fake reverse command. The robot actually reversed.

At the end of the run:

- `bumper_trigger: 7`
- `bumper_safety_active: 1`
- outcome: `canceled`

This means the run ended because the physical bumper safety chain intervened after reverse motion contacted an obstacle.

## Main Interpretation

The current MPPI trial shows that:

1. MPPI is correctly integrated and operational.
2. MPPI did not immediately fail with the same DWB-style controller deadlock signature.
3. However, the current MPPI settings allow reverse solutions.
4. In the tested environment, that reverse behavior is not yet safe enough.

The current MPPI configuration explicitly allows this:

- `vx_min: -0.05`
- `motion_model: DiffDrive`
- `PathAngleCritic` runtime log indicated reversing is allowed
- `PreferForwardCritic` only biases forward motion, it does not forbid reverse motion

Therefore, the current failure mode is best described as:

- `MPPI + reverse motion allowed + insufficient rear safety margin`

not as a generic MPPI integration failure.

## Current Assessment

The present MPPI status should be considered:

- technically integrated
- behaviorally promising
- not yet safe for direct replacement of the DWB setup

This is because the first meaningful failure under MPPI was a real reverse-motion bumper intervention, not a purely software-side local planner stall.

## Recommended Next Step

The next step should be to constrain reverse behavior before larger MPPI regression runs.

Recommended first adjustment:

- set `vx_min: 0.0`

Alternative if some reverse behavior must remain available:

- keep `vx_min < 0`
- but significantly strengthen forward preference and add stronger rear-safety handling

At the current stage, the most practical direction is:

- first test MPPI in a forward-only or near-forward-only form
- then rerun the same `goal_b` segment
- only after that run longer multi-cycle comparisons against DWB
