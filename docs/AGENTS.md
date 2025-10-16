# Repository Guidelines

## Project Structure & Module Organization
- `src/` hosts ROS 2 packages: `t_robot_bringup` (launch/config), `t_robot_slam` (SLAM nodes), and vendor stacks (`megarover3_ros2`, `livox_ros_driver2`, `realsense-ros`, `vs_rover_options_description`). Leave vendor code untouched.
- `build/`, `install/`, `log/` are colcon artifacts regenerated as needed; remove them between clean builds.
- Root scripts (`pre_test_check.sh`, `test_bringup.sh`, `test_mapping.sh`, `test_navigation.sh`, `reset_ros_env.sh`) plus RViz configs handle automation so packages stay lean.

## Build, Test, and Development Commands
- `source /opt/ros/humble/setup.bash && source install/setup.bash` before building; run `./reset_ros_env.sh` if the shell drifts.
- `colcon build --symlink-install --packages-up-to t_robot_bringup t_robot_slam` compiles nodes while keeping files editable in-place.
- `colcon test` runs ament lint/unit suites; narrow the scope with `--packages-select <pkg>` when iterating.
- `./pre_test_check.sh` gates hardware runs; run `./test_*.sh` for bringup, mapping, and navigation smoke tests (requires MID360 and Megarover online).

## Coding Style & Naming Conventions
- C++ (`src/t_robot_slam/src`) uses Allman braces, two-space indentation, and `snake_case` names; rely on the default `ament_lint_auto` formatters triggered by `colcon test`.
- Python nodes (`src/t_robot_bringup/scripts`) follow PEP 8 with 4-space indents, module docstrings, and descriptive `Node` names. Keep launch and YAML keys aligned with declared parameters.
- File naming: place launch/YAML configs in each package’s `launch/` and `params/` directories; new nodes should live under `src/<pkg>/src` or `scripts/` following existing patterns.

## Testing Guidelines
- Run `colcon test` after feature work, keep `BUILD_TESTING` enabled, and register new gtests or launch tests under `test/` via `CMakeLists.txt`.
- Hardware scripts must log to `/tmp` and clean up processes like `test_bringup.sh`; mirror that pattern for diagnostics.
- Capture bag files for regressions but keep large data out of the repo—reference storage paths in issues instead.

## Commit & Pull Request Guidelines
- Use Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`). Body should capture the reason for the change plus the commands used for verification.
- PRs should list impacted launch/config files, link tracking issues, and attach RViz screenshots or log snippets when behavior changes. Tag bringup and SLAM reviewers for shared parameter updates.
- Document hardware configuration deltas inside the PR and mirror persistent updates in `project_plan.md`.

## Configuration & Safety Notes
- Run `./pre_test_check.sh` on every new vehicle or Jetson image to confirm network, serial, and package availability before deployment.
- Keep QoS/topic names consistent with `t_robot_bringup` defaults; if you override them, update the relevant YAML and notify mapping/navigation owners.
- Stage experimental launch files under `src/t_robot_bringup/launch/experiments/` and strip device-specific values before merging.
