# Agent Bootstrap

This workspace is the active RoboMaster sentry simulation workspace.

## First Reads

Read these before changing behavior:

```text
docs/index.md
docs/agent_bootstrap.md
docs/architecture/runtime_flows.md
docs/reference/interfaces.md
docs/reference/parameters.md
docs/reference/active_legacy.md
```

For operation/debugging:

```text
docs/runbooks/bringup.md
docs/runbooks/debug_planning.md
docs/runbooks/debug_control.md
docs/validation/planning_validation.md
docs/validation/control_validation.md
```

## Active Runtime

The active simulation path is:

```text
pb_rm_simulation/Gazebo + rm_nav_bringup FAST-LIO2
-> /cloud_registered + /Odometry
-> sentry_mapping/traversability_mapper
-> sentry_perception/esdf2d
-> sentry_planner/sim_planner
-> sentry_controller/traj_tracker
-> /cmd_vel_chassis
```

`pure_pursuit` and ROG-Map based mapping are legacy/reference paths unless a user explicitly asks to revive them.

## Repository Boundaries

- `src/pb_rm_simulation` is a lightweight local vendor subset derived from the Polar Bear RoboMaster simulation stack. It is not an embedded Git repo in this checkout.
- `src/sentry_mapping` and `src/sentry_perception` are local packages in this workspace, not symlinks.
- `rm_sentry_real_ws` is legacy/reference. Do not delete or mutate it while working in this workspace unless the user explicitly asks.

## Edit Rules

- Do not commit or rely on `build/`, `install/`, `log/`, rosbag outputs, debug CSV files, or generated frame graphs.
- Do not introduce machine-specific absolute paths in launch/config/source files.
- Runtime assets required by launch files should live in ROS package share directories so `get_package_share_directory` or `FindPackageShare` can resolve them.
- Prefer small, validated changes. After code/config changes, run at least a targeted `colcon build --symlink-install --packages-select ...` and search for newly introduced absolute paths.

## Common Commands

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  livox_ros_driver2 ros2_livox_simulation imu_complementary_filter fast_lio \
  pb_rm_simulation rm_nav_bringup \
  sentry_mapping sentry_perception sentry_planner sentry_controller
source install/setup.bash
```

```bash
rg -n --hidden -S "/home/|/Users/" . --glob '!build/**' --glob '!install/**' --glob '!docs/**'
```
