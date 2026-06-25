# Project State

This file is the current-truth control plane for `rm_sentry_sim_ws`. If it disagrees with launch files, yaml, source code, or a running ROS graph, the executable source is authoritative and this file should be updated.

## Project Goal

Build and validate a simulation-first RoboMaster sentry navigation stack that can later be migrated to a real robot with clear module contracts, launch boundaries, and verification evidence.

## Active Pipeline

```text
Gazebo + simulated MID360 + FAST-LIO2
  -> /cloud_registered + /Odometry
  -> relocalization_node: map -> lidar_odom
  -> traversability_mapper: height-gated 2.5D costmap
  -> esdf2d_node: ESDF visualization/reference
  -> costmap_inflator
  -> jps_node
  -> minco_planner_node
  -> traj_tracker
  -> /cmd_vel_chassis
```

Stage 3 intentionally uses `/odom` as the simulation pose source for JPS, MINCO, and `traj_tracker`. This avoids FAST-LIO simulation drift contaminating controller tests while Stage 2 still uses FAST-LIO output for perception.

## Deprecated Or Non-Default Pipelines

| Pipeline or Component | Current Status |
|---|---|
| Nav2 upstream-style pipeline from the original simulation stack | removed from the local vendor subset |
| ROG-Map runtime in `sim_ws` | removed; current mapping is `traversability_mapper` |
| `perception_mapper_node` | removed old ROG wrapper |
| `goal_controller` | removed old simple debug controller |
| `path_tracker` | removed old geometric/Frenet tracker baseline |
| `pure_pursuit` and fixed `traj_publisher` | removed |
| direct MPC controller | future option, not the current implementation |

Do not re-enable a deprecated path unless the task explicitly asks for historical reconstruction or migration work.

## Current Bottleneck

The current known risk area is high-speed navigation near sharp corners and walls. The active strategy is not to solve this only by increasing JPS inflation. JPS provides topology; MINCO improves clearance and timing; `traj_tracker` follows the timed trajectory while respecting measured control dt and acceleration limits.

## Critical Assumptions

| Assumption | Why It Matters | First Check |
|---|---|---|
| Stage 2 costmap is in `map` and aligns with the simulated field | planning safety depends on correct obstacle geometry | `ros2 topic echo /perception/costmap_2d --once` and RViz fixed frame `map` |
| `map -> lidar_odom` exists and is stable | traversability mapping uses global alignment | `ros2 topic echo /relocalization/status --once` |
| Stage 3 pose source is `/odom` in sim | JPS/MINCO/tracker coordinates must match current simplification | `docs/architecture/runtime_flows.md` and `sim_planner.launch.py` |
| MINCO final check passes on nominal routes | controller results are not meaningful on unexpected fallback trajectories | `/tmp/planner.log` |
| `traj_tracker` uses measured `control_dt` | acceleration limiting and overshoot depend on real callback timing | `/tmp/traj_tracker_debug.csv` |
| Gazebo/chassis plugin update rate is high enough | low odom/control responsiveness can look like controller failure | `/odom` hz and xacro/plugin settings |

## Runtime Entrypoints

| Stage | Launch |
|---|---|
| Stage 1: Gazebo + FAST-LIO2 | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RM3V3 lio:=fastlio mode:=mapping lio_rviz:=False nav_rviz:=False` |
| Stage 2: perception | `ros2 launch rm_nav_bringup sim_perception.launch.py perception_rviz:=false` |
| Stage 3: planning + control | `ros2 launch sentry_planner sim_planner.launch.py` |

## Code Entrypoints

| Area | File |
|---|---|
| Stage 1 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` |
| Stage 2 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` |
| Stage 3 launch | `src/sentry_planner/launch/sim_planner.launch.py` |
| FAST-LIO sim config | `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml` |
| Relocalization | `src/sentry_perception/sentry_perception/relocalization_node.py` |
| Traversability mapper | `src/sentry_mapping/src/traversability_mapper_node.cpp` |
| ESDF 2D | `src/sentry_perception/sentry_perception/esdf2d_node.py` |
| JPS | `src/sentry_planner/sentry_planner/jps_node.py` |
| MINCO node | `src/sentry_planner/sentry_planner/minco_planner_node.py` |
| MINCO solver | `src/sentry_planner/sentry_planner/minco_solver_2d.py` |
| Current tracker | `src/sentry_controller/sentry_controller/traj_tracker.py` |

## Must-Read Docs

Read these first when re-entering the project:

1. `docs/current/project_state.md`
2. `docs/current/architecture.md`
3. `docs/current/active_pipeline.md`
4. `docs/current/dataflow_and_frames.md`
5. `docs/current/agent_bootstrap.md`
6. `docs/architecture/runtime_flows.md`
7. `docs/reference/interfaces.md`
8. `docs/reference/parameters.md`
9. `docs/current/active_legacy.md`
10. `docs/runbooks/bringup.md`

Use `docs/height_gated_traversability_plan.md` as a historical work log, not as the first source for current runtime truth.

## Current Verification Status

The testbook templates exist in `docs/testbook/`. Evidence packets are not yet backfilled. New runs should be recorded under `docs/evidence/` using the templates there.
