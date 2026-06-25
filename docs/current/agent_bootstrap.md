# Agent Bootstrap

This file is a routing surface for coding agents working in `rm_sentry_sim_ws`. It is not a complete project encyclopedia.

## First Reads

Read these before modifying navigation, perception, planning, or control:

1. [`docs/index.md`](../index.md)
2. [`docs/current/project_state.md`](project_state.md)
3. [`docs/current/architecture.md`](architecture.md)
4. [`docs/current/active_pipeline.md`](active_pipeline.md)
5. [`docs/current/dataflow_and_frames.md`](dataflow_and_frames.md)
6. [`docs/architecture/runtime_flows.md`](../architecture/runtime_flows.md)
7. [`docs/modules/README.md`](../modules/README.md)
8. [`docs/reference/interfaces.md`](../reference/interfaces.md)
9. [`docs/reference/parameters.md`](../reference/parameters.md)
10. [`docs/current/active_legacy.md`](active_legacy.md)
11. [`docs/history/decisions/navigation_decisions.md`](../history/decisions/navigation_decisions.md)
12. [`docs/runbooks/bringup.md`](../runbooks/bringup.md) if you need to launch or stop the stack

Use [`docs/height_gated_traversability_plan.md`](../height_gated_traversability_plan.md) as historical context, not as the first source for current active runtime behavior.

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

The current controller is `traj_tracker`. The older `goal_controller`, `path_tracker`, `pure_pursuit`, and fixed test `traj_publisher` nodes have been removed from this workspace. `keyboard_teleop` remains as a manual smoke-test tool.

The current sim perception map is height-gated traversability, not ROG-Map. The old ROG wrapper/config/launch/RViz files and the sim_ws `src/rog_map` symlink have been removed; only archive/history documentation remains for reconstruction context.

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
| `sentry_mapping/src/perception_mapper_node.cpp` | removed old ROG-Map wrapper |
| `notes/progress.md` | removed old progress notes; check historical docs only if needed |

## Change Rules

- Before editing parameters, identify whether the effective value comes from launch, yaml, or code default.
- Before changing Stage 3 behavior, read `sim_planner.launch.py`; it overrides many node defaults.
- Before changing mapping behavior, read `sim_perception.launch.py`; it sets the current `traversability_mapper` and `esdf2d_node` parameters.
- If a change affects topics, frames, launch order, or active/legacy status, update the docs in this entry layer.
- Do not look for old ROG mapping files in `sim_ws`; use `docs/archive/` and `docs/history/` if a task asks to reconstruct that lineage.

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
| Agent might be editing the wrong node | [`docs/current/active_legacy.md`](active_legacy.md) |
| JPS no path, MINCO fallback, wall-hugging path | [`docs/runbooks/debug_planning.md`](../runbooks/debug_planning.md) |
| Overshoot, drift, slow response, speed too low | [`docs/runbooks/debug_control.md`](../runbooks/debug_control.md) |
| Need to start or stop the stack | [`docs/runbooks/bringup.md`](../runbooks/bringup.md) |
| Need to judge localization changes | [`docs/testbook/localization_validation.md`](../testbook/localization_validation.md) |
| Need to judge mapping changes | [`docs/testbook/mapping_validation.md`](../testbook/mapping_validation.md) |
| Need to judge planning changes | [`docs/testbook/planning_validation.md`](../testbook/planning_validation.md) |
| Need to judge control changes | [`docs/testbook/control_validation.md`](../testbook/control_validation.md) |
| Need to judge full-system behavior | [`docs/testbook/system_validation.md`](../testbook/system_validation.md) |
| Topic/frame mismatch | [`docs/reference/interfaces.md`](../reference/interfaces.md) |
| Parameter uncertainty | [`docs/reference/parameters.md`](../reference/parameters.md) |
| Deprecated path uncertainty | [`docs/archive/deprecated_implementations.md`](../archive/deprecated_implementations.md) |
