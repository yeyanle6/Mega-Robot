## Project Overview as of March 11, 2026

This project is no longer in the "can SLAM run" phase. The main navigation chain has already been brought up on the real robot and the project is now in the stabilization, safety, and regression-testing phase.

The current system backbone is:

- Livox MID-360
- FAST-LIO2
- PGO
- localizer
- Nav2
- micro-ROS chassis interface

The D455 is not the primary localization source. Its role is front near-field supplementary perception, especially for low obstacles that may not be represented reliably enough by the main LiDAR pipeline alone.

## Current Architecture

The current recommended navigation entrypoint is:

- [fastlio2_pgo_navigation.launch.py](/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/launch/fastlio2_pgo_navigation.launch.py)

The currently used latest map is:

- [map.yaml](/home/ros/Code/Demo8/maps/new_map_20260310_0918/map.yaml)
- [map.pcd](/home/ros/Code/Demo8/maps/new_map_20260310_0918/map.pcd)

The system should be understood as follows:

- Livox + FAST-LIO2 provide the main localization and main obstacle structure
- localizer provides map-based relocalization
- Nav2 handles planning and control
- D455 provides front near-field low-obstacle supplementation

## Planning and Control Status

The project has already started moving away from the older `NavFn + DWB` setup.

Current direction:

- Global planner: `Smac Hybrid-A*`
- Local controller: still `DWB` for now

This is an important project transition point.

`Smac Hybrid-A*` is a good fit for the robot because:

- it is better suited to narrow indoor environments
- it handles heading-sensitive path generation better than `NavFn`
- it is more appropriate for a differential-drive platform with an asymmetric footprint

However, local control has not been fully stabilized yet because `DWB` is still in place. The system can complete navigation runs, but near-goal behavior and local-plan degradation have not been fully eliminated.

## D455 Integration Status

The D455 work is one of the most important current developments in the project.

The earlier problem was not simply that the D455 "could not see." The deeper issue was that the D455 data path into the local costmap was not correctly structured:

- ground separation was too crude
- low obstacles and floor points were difficult to distinguish
- in no-obstacle situations, the filter could stop publishing entirely
- this caused local costmap sensor timeout behavior

To address that, the project has moved away from treating the D455 as a raw point-cloud supplement and toward a dedicated near-field obstacle extraction stage.

The key node is:

- [d455_nearfield_obstacle_filter.py](/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/scripts/d455_nearfield_obstacle_filter.py)

Its purpose is to convert raw D455 point clouds into:

- `/d455_front_obstacles`

This topic is intended to contain only near-field, locally meaningful obstacle points for the robot body, instead of forwarding coarse-filtered raw depth points directly into the local costmap.

This reflects an important architectural shift:

- from "stack more sensor data directly into Nav2"
- toward "embodied near-field perception for robot traversability"

## Current Test Status

The system has already demonstrated that the main navigation loop can work on the real robot.

The latest dedicated workbench-based real-robot regression record is:

- [WORKBENCH_TEST_STATUS_20260312.md](/home/ros/Code/Demo8/WORKBENCH_TEST_STATUS_20260312.md)

Using the automated two-point regression script:

- [two_point_nav_test.py](/home/ros/Code/Demo8/src/megarover3_ros2/megarover3_navigation/scripts/two_point_nav_test.py)

recent testing showed:

- a `3`-dispatch run completed with `3/3 succeeded`
- the command chain `plan -> cmd_vel -> rover_twist -> rover_odo` was working
- the chassis was actually executing commands

This means the project is no longer blocked at the basic integration layer.

However, the system should not yet be described as stable.

Observed residual issues include:

- one test leg (`goal_a`) remains much slower than the other
- mild `DWB` degradation signals still appear in logs
- near-goal `local_plan_length -> 0.0` behavior still shows up
- a later long-run attempt resulted in a real collision during repeated autonomous navigation

So the current state is:

- the robot can navigate
- the stack is not yet robust enough for unattended repeated long-run regression

## Main Remaining Risks

The major remaining risks are:

1. Local control stability

`DWB` still shows signs of degradation in narrow or near-goal situations. The system can succeed, but this part has not fully converged.

2. D455 near-field reliability

The D455 path has improved from "broken" to "basically usable," but its near-field obstacle extraction still needs continued real-world validation, especially in collision-prone scenes.

3. Real-robot safety under repeated runs

A collision already occurred during a longer repeated test sequence. That means the project should currently prioritize:

- short targeted reproductions
- single-variable debugging
- controlled regression

instead of immediately scaling up long unattended test loops.

## Overall Project Assessment

In one sentence:

This is a real-robot indoor navigation project whose main SLAM and navigation backbone is already operational, whose global planning has already advanced to `Smac Hybrid-A*`, and whose D455 integration is currently evolving into a dedicated near-field embodied perception module. The current engineering focus is turning "it can run" into "it is stable, safe, and repeatable."
