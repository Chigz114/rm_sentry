# Current Architecture

This is the current architecture overview for `rm_sentry_sim_ws`.

For detailed runtime commands, use `docs/runbooks/bringup.md`. For detailed topic/frame tables, use `docs/current/dataflow_and_frames.md` and `docs/reference/interfaces.md`.

## System Boundary

This workspace is the simulation-first RoboMaster sentry navigation stack.

In scope:

- RM3V3 Gazebo simulation.
- Simulated MID360/Livox lidar and IMU path.
- FAST-LIO2 state-estimation outputs used by perception.
- Height-gated 2.5D traversability mapping.
- 2D ESDF visualization/reference.
- JPS topology planning.
- MINCO timed trajectory generation.
- `traj_tracker` velocity tracking.

Out of scope for the active sim pipeline:

- Nav2 upstream-style navigation.
- ROG-Map as the active sim mapper.
- Pure pursuit or old geometric trackers as active control.
- Real-robot bringup.
- Direct MPC control.

## Container View

| Container | Main Packages | Role |
|---|---|---|
| Simulation vendor subset | `pb_rm_simulation`, `rm_nav_bringup`, Livox sim, FAST-LIO2 | provides Gazebo robot/world, lidar/IMU, LIO outputs |
| Mapping/perception | `sentry_mapping`, `sentry_perception` | relocalization, traversability costmap, ESDF |
| Planning | `sentry_planner` | inflation, JPS, MINCO, trajectory samples |
| Control | `sentry_controller` | `traj_tracker` and `keyboard_teleop` manual smoke-test tool |
| Documentation | `docs/` | current truth, modules, reference, runbooks, testbook, history, evidence |

## Module Contracts

Each active runtime module has a fixed-format module card in `docs/modules/`.

The active cards are:

- `fastlio2.md`
- `relocalization.md`
- `traversability_map.md`
- `esdf2d.md`
- `costmap_inflation.md`
- `jps.md`
- `minco.md`
- `traj_tracker.md`
- `gazebo_chassis.md`

## Architecture Rules

- Active launch files are the first source for runtime topology.
- Launch overrides beat code defaults for effective parameters.
- Stage 3 uses `/odom` in simulation by design.
- Height-gated traversability is the active sim mapping path.
- Removed ROG and older controller paths are documented in archive/history, not default debugging targets.
