# Active And Legacy Components

This document is the central active/legacy map for `rm_sentry_sim_ws`. Use it before selecting files to modify.

The goal is to prevent old but valid-looking nodes from being mistaken for the current runtime chain.

## Current Active Runtime Chain

```text
Stage 1:
  Gazebo + simulated MID360 + FAST-LIO2

Stage 2:
  relocalization_node
  traversability_mapper
  esdf2d_node

Stage 3:
  costmap_inflator
  jps_node
  minco_planner_node
  traj_tracker
```

## Active Entrypoints

| Area | Active File | Why |
|---|---|---|
| Stage 1 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` | starts simulation, robot, FAST-LIO branch |
| Stage 2 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` | starts relocalization, height-gated mapping, ESDF |
| Stage 3 launch | `src/sentry_planner/launch/sim_planner.launch.py` | starts active planner and controller chain |
| FAST-LIO config | `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml` | current sim LIO config |
| Relocalization | `src/sentry_perception/sentry_perception/relocalization_node.py` | publishes `map -> lidar_odom` |
| Mapping | `src/sentry_mapping/src/traversability_mapper_node.cpp` | current costmap source |
| 2D ESDF | `src/sentry_perception/sentry_perception/esdf2d_node.py` | costmap-to-ESDF visualization/reference |
| JPS | `src/sentry_planner/sentry_planner/jps_node.py` | current grid path planner |
| MINCO node | `src/sentry_planner/sentry_planner/minco_planner_node.py` | current trajectory optimizer wrapper |
| MINCO solver | `src/sentry_planner/sentry_planner/minco_solver_2d.py` | current polynomial solver/cost implementation |
| Controller | `src/sentry_controller/sentry_controller/traj_tracker.py` | current active tracker |

## Active Topics

| Stage | Topic | Status |
|---|---|---|
| Stage 1 | `/cloud_registered` | active FAST-LIO point cloud output |
| Stage 1 | `/Odometry` | active FAST-LIO odometry for mapping |
| Stage 1/3 | `/odom` | active sim/chassis odometry for Stage 3 pose |
| Stage 2 | `/relocalization/status` | active relocalization health output |
| Stage 2 | `/perception/costmap_2d` | active planning costmap |
| Stage 2 | `/perception/esdf_2d` | active ESDF visualization output |
| Stage 3 | `/planner/costmap_inflated` | active JPS input |
| Stage 3 | `/planner/path` | active JPS path output |
| Stage 3 | `/planner/traj_samples` | active controller trajectory input |
| Stage 3 | `/cmd_vel_chassis` | active chassis command output |

## Registered But Not Active In Current Stage 3

These tools remain useful, but the current `sim_planner.launch.py` does not launch them.

| Component | File | Current Role |
|---|---|---|
| `keyboard_teleop` | `src/sentry_controller/sentry_controller/keyboard_teleop.py` | manual smoke-test tool |
| `esdf_grad_viz` | `src/sentry_planner/sentry_planner/esdf_grad_viz.py` | optional ESDF gradient visualization/debug node |

Do not edit optional debug tools first when debugging current overshoot, MINCO fallback, or wall-risk behavior unless the task explicitly asks for manual testing or visualization support.

Removed old controller files:

- `src/sentry_controller/sentry_controller/path_tracker.py`
- `src/sentry_controller/sentry_controller/goal_controller.py`
- `src/sentry_controller/sentry_controller/pure_pursuit.py`
- `src/sentry_controller/sentry_controller/traj_publisher.py`

## Removed ROG-Map Runtime

The current simulation chain uses `traversability_mapper`, not ROG-Map.

Archive note: see `docs/archive/deprecated_implementations.md` for the deprecated implementation inventory.

| Removed Item | Previous Role |
|---|---|
| `src/sentry_mapping/src/perception_mapper_node.cpp` | old ROG wrapper |
| `src/sentry_mapping/config/rog_map_sentry.yaml` | old ROG mapping config |
| `src/sentry_mapping/launch/mapping.launch.py` | old ROG mapping launch |
| `src/sentry_mapping/launch/replay_mapping.launch.py` | old ROG replay launch |
| `src/sentry_mapping/rviz/mapping.rviz` | old ROG RViz config |
| `src/rog_map` symlink | old local ROG dependency |
| `rm_nav_bringup/config/simulation/rog_map_sim.yaml` | old sim ROG config |

Use the archive/history docs only if a task explicitly asks to reconstruct or compare the old ROG path. The sibling `rm_sentry_real_ws` checkout remains separate legacy/reference context and is not part of current sim Stage 3.

## Removed Upstream Side Branches

The local `pb_rm_simulation` vendor subset no longer contains the upstream-style Nav2, SLAM Toolbox, Point-LIO, ICP, segmentation, pointcloud-to-laserscan, TEB, costmap-converter, or fake-velocity-transform branches. Current `bringup_sim.launch.py` starts only the Stage 1 simulation and FAST-LIO path needed by the custom Stage 2/3 chain.

Current standard bringup uses:

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RM3V3 lio:=fastlio mode:=mapping lio_rviz:=False nav_rviz:=False
```

## Historical Documents

| File | Current Interpretation |
|---|---|
| `docs/height_gated_traversability_plan.md` | detailed history and experiment log; not first source for current facts |
| `notes/progress.md` | removed old progress notes; use docs history only if needed |
| `docs/file_inventory.md` | inventory snapshot; useful, but should not override active launch/code |
| `docs/history/` | structured history skeleton for future confirmed records |
| `docs/evidence/` | structured evidence packets for future real runs |

## Safe Selection Rules

- If modifying current runtime topology, start from launch files.
- If modifying current parameters, check launch overrides before code defaults.
- If debugging current control, start with `traj_tracker`.
- If debugging current planning, start with `jps_node`, `minco_planner_node`, and `minco_solver_2d`.
- If debugging current mapping, start with `traversability_mapper`.
- If a legacy file looks relevant, confirm it is launched before editing it.
