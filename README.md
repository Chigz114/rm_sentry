# RM Sentry Simulation Workspace

ROS 2 Humble workspace for the RoboMaster sentry navigation simulation.

The current active chain is:

```text
Gazebo + FAST-LIO2
-> /cloud_registered + /Odometry
-> sentry_mapping/traversability_mapper
-> sentry_perception/esdf2d
-> sentry_planner JPS + MINCO
-> sentry_controller/traj_tracker
-> /cmd_vel_chassis
```

## Repository Layout

```text
src/
  sentry_controller/      # traj_tracker and auxiliary control nodes
  sentry_planner/         # JPS, MINCO smoothing, planner launch
  sentry_mapping/         # height-gated traversability mapper
  sentry_perception/      # ESDF and relocalization nodes
  pb_rm_simulation/       # lightweight Polar Bear simulation vendor subset
docs/                     # current truth, modules, reference, runbooks, testbook, history, evidence
tools/                    # generation utilities
scripts/                  # reserved for ad-hoc local scripts; currently empty
```

`build/`, `install/`, `log/`, rosbag outputs, and debug CSV/graph dumps are intentionally ignored.

## Dependency Boundary

`src/pb_rm_simulation` is a lightweight local vendor subset derived from the Polar Bear RoboMaster simulation stack. It keeps the `pb` naming for attribution, but the upstream `.git`, unused Nav2/TEB/Point-LIO/ICP branches, old maps, and large demo assets have been removed.

`src/sentry_mapping` and `src/sentry_perception` are now real package directories in this workspace. They were copied from the older real-workspace reference so that this simulation workspace does not depend on absolute symlinks.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For the currently edited packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  livox_ros_driver2 ros2_livox_simulation imu_complementary_filter fast_lio \
  pb_rm_simulation rm_nav_bringup \
  sentry_mapping sentry_perception sentry_planner sentry_controller
```

## Run

Use the runbook as the source of truth:

```text
docs/runbooks/bringup.md
```

The short form of the staged simulation flow is:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RM3V3 lio:=fastlio mode:=mapping lio_rviz:=False nav_rviz:=False
ros2 launch rm_nav_bringup sim_perception.launch.py perception_rviz:=false
ros2 launch sentry_planner sim_planner.launch.py
```

## Documentation Entry

Start at:

```text
docs/index.md
```

Useful next pages:

- `docs/current/project_state.md`
- `docs/current/agent_bootstrap.md`
- `docs/current/architecture.md`
- `docs/current/active_pipeline.md`
- `docs/current/dataflow_and_frames.md`
- `docs/current/active_legacy.md`
- `docs/architecture/runtime_flows.md`
- `docs/modules/README.md`
- `docs/reference/interfaces.md`
- `docs/reference/parameters.md`
- `docs/runbooks/bringup.md`
- `docs/testbook/localization_validation.md`
- `docs/testbook/mapping_validation.md`
- `docs/testbook/planning_validation.md`
- `docs/testbook/control_validation.md`
- `docs/testbook/system_validation.md`
- `docs/archive/deprecated_implementations.md`

## Portability Rules

- Do not add machine-specific absolute paths to launch/config/source files.
- Put simulation assets that are needed at runtime inside an installed ROS package share directory.
- Keep large recordings out of Git unless they are deliberately tracked with Git LFS or a release artifact workflow.
- Treat `rm_sentry_real_ws` as legacy/reference until real-robot bringup is intentionally migrated.
