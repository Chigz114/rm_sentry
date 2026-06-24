# Agent Bootstrap

This file is a routing surface for coding agents working in `rm_sentry_sim_ws`. It is not a complete project encyclopedia.

## First Reads

Read these before modifying navigation, perception, planning, or control:

1. [`docs/index.md`](index.md)
2. [`docs/architecture/runtime_flows.md`](architecture/runtime_flows.md)
3. [`docs/reference/interfaces.md`](reference/interfaces.md)
4. [`docs/reference/parameters.md`](reference/parameters.md)
5. [`docs/reference/active_legacy.md`](reference/active_legacy.md)
6. [`docs/decisions/navigation_decisions.md`](decisions/navigation_decisions.md)
7. [`docs/runbooks/bringup.md`](runbooks/bringup.md) if you need to launch or stop the stack

Use [`docs/height_gated_traversability_plan.md`](height_gated_traversability_plan.md) as historical context, not as the first source for current active runtime behavior.

## Current Active Chain

```text
Stage 1:
  rm_nav_bringup bringup_sim.launch.py
  Gazebo + simulated MID360 + FAST-LIO2

Stage 2:
  rm_nav_bringup sim_perception.launch.py
  relocalization_node + traversability_mapper + esdf2d_node

Stage 3:
  sentry_planner sim_planner.launch.py
  costmap_inflator + jps_node + minco_planner_node + traj_tracker
```

The current controller is `traj_tracker`, not `goal_controller` or `path_tracker`. The earlier `pure_pursuit` and fixed test `traj_publisher` nodes have been removed from this workspace.

The current sim perception map is height-gated traversability, not ROG-Map. The sim_ws `src/rog_map` symlink has been removed; ROG-related files are retained only as optional/legacy references.

## Main Entrypoints

| Area | File |
|---|---|
| Stage 1 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` |
| Stage 2 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` |
| Stage 3 launch | `src/sentry_planner/launch/sim_planner.launch.py` |
| FAST-LIO sim config | `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml` |
| Height-gated mapper | `src/sentry_mapping/src/traversability_mapper_node.cpp` |
| Relocalization | `src/sentry_perception/sentry_perception/relocalization_node.py` |
| 2D ESDF | `src/sentry_perception/sentry_perception/esdf2d_node.py` |
| JPS | `src/sentry_planner/sentry_planner/jps_node.py` |
| MINCO ROS node | `src/sentry_planner/sentry_planner/minco_planner_node.py` |
| MINCO solver | `src/sentry_planner/sentry_planner/minco_solver_2d.py` |
| Current tracker | `src/sentry_controller/sentry_controller/traj_tracker.py` |

## Legacy Or Baseline Traps

| File or Node | Status |
|---|---|
| `sentry_controller/path_tracker.py` | previous geometric/Frenet tracker baseline, not active in current launch |
| `sentry_controller/goal_controller.py` | simple early controller, not active |
| `sentry_mapping/src/perception_mapper_node.cpp` | optional ROG-Map wrapper from the real-data lineage; skipped in sim_ws when `rog_map` is absent |
| `notes/progress.md` | removed old progress notes; check historical docs only if needed |

## Change Rules

- Before editing parameters, identify whether the effective value comes from launch, yaml, or code default.
- Before changing Stage 3 behavior, read `sim_planner.launch.py`; it overrides many node defaults.
- Before changing mapping behavior, read `sim_perception.launch.py`; it sets the current `traversability_mapper` and `esdf2d_node` parameters.
- If a change affects topics, frames, launch order, or active/legacy status, update the docs in this entry layer.
- Do not delete remaining old controllers or ROG-lineage files without an explicit cleanup task; some are still useful for comparison or rollback.

## Validation Shortcuts

Use these before deeper debugging:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep -E "relocalization|traversability|esdf|inflator|jps|minco|tracker"
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic hz /perception/costmap_2d
ros2 topic hz /planner/costmap_inflated
ros2 topic echo /relocalization/status --once
```

For tracker overshoot or speed issues, inspect:

```bash
/tmp/traj_tracker_debug.csv
/tmp/planner.log
```

Look for `control_dt`, `nearest_d`, command magnitude, `MINCO final check`, `wp_shift`, `max_v`, `max_a`, and fallback messages.

## Debug Routing

| Symptom | Read First |
|---|---|
| Agent might be editing the wrong node | [`docs/reference/active_legacy.md`](reference/active_legacy.md) |
| JPS no path, MINCO fallback, wall-hugging path | [`docs/runbooks/debug_planning.md`](runbooks/debug_planning.md) |
| Overshoot, drift, slow response, speed too low | [`docs/runbooks/debug_control.md`](runbooks/debug_control.md) |
| Need to start or stop the stack | [`docs/runbooks/bringup.md`](runbooks/bringup.md) |
| Need to judge planning changes | [`docs/validation/planning_validation.md`](validation/planning_validation.md) |
| Need to judge control changes | [`docs/validation/control_validation.md`](validation/control_validation.md) |
| Topic/frame mismatch | [`docs/reference/interfaces.md`](reference/interfaces.md) |
| Parameter uncertainty | [`docs/reference/parameters.md`](reference/parameters.md) |
