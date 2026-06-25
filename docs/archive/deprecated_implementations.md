# Deprecated Implementations

This file lists implementation paths that are not part of the active simulation pipeline.

## ROG-Map Runtime

Status: removed from current `sim_ws` runtime.

Active replacement:

- `src/sentry_mapping/src/traversability_mapper_node.cpp`
- `docs/modules/traversability_map.md`

Removed from current sim workspace:

| Removed Item | Notes |
|---|---|
| `src/sentry_mapping/src/perception_mapper_node.cpp` | removed old ROG-Map wrapper from the real-data lineage |
| `src/sentry_mapping/config/rog_map_sentry.yaml` | removed old ROG config |
| `src/sentry_mapping/launch/mapping.launch.py` | removed ROG-oriented mapping launch |
| `src/sentry_mapping/launch/replay_mapping.launch.py` | removed ROG-oriented replay launch |
| `src/sentry_mapping/rviz/mapping.rviz` | removed ROG-oriented RViz config |
| `src/rog_map` symlink | removed; current sim workspace does not build against ROG-Map by default |
| `rm_nav_bringup/config/simulation/rog_map_sim.yaml` | removed; current sim perception launch does not use it |

Do not look for ROG files to fix current sim behavior unless the task explicitly asks to reconstruct or compare the ROG path.

## Old Perception Obstacle/Cluster Pipeline

Status: removed from current `sim_ws` runtime.

Active replacement:

- `src/sentry_mapping/src/traversability_mapper_node.cpp`
- `src/sentry_perception/sentry_perception/relocalization_node.py`
- `src/sentry_perception/sentry_perception/esdf2d_node.py`
- `docs/modules/traversability_map.md`
- `docs/modules/relocalization.md`
- `docs/modules/esdf2d.md`

Removed:

| Removed Item | Notes |
|---|---|
| `src/sentry_perception/sentry_perception/obstacle_detector_node.py` | old ROI/voxel/ground-threshold point-cloud obstacle filter |
| `src/sentry_perception/sentry_perception/cluster_detector_node.py` | old Euclidean clustering and marker publisher |
| `src/sentry_perception/launch/perception.launch.py` | launched the removed obstacle/cluster pipeline |
| `src/sentry_perception/launch/replay_perception.launch.py` | old bag replay perception launch |
| `src/sentry_perception/config/obstacle_detector.yaml` | parameters for removed obstacle detector |
| `src/sentry_perception/config/cluster_detector.yaml` | parameters for removed cluster detector |
| `src/sentry_perception/rviz/perception.rviz` | RViz config for removed obstacle markers |
| `src/sentry_perception/README.md` | old README describing the removed detector pipeline |

The current `sentry_perception` package intentionally registers only `relocalization` and `esdf2d`.

## Old Controllers

Status: removed from current `sim_ws` runtime.

Active replacement:

- `src/sentry_controller/sentry_controller/traj_tracker.py`
- `docs/modules/traj_tracker.md`

Still present:

| File | Role |
|---|---|
| `src/sentry_controller/sentry_controller/keyboard_teleop.py` | manual smoke-test tool |

Removed:

| Removed Item | Notes |
|---|---|
| `src/sentry_controller/sentry_controller/path_tracker.py` | removed previous geometric/Frenet baseline |
| `src/sentry_controller/sentry_controller/goal_controller.py` | removed simple goal-to-point debug controller |
| `src/sentry_controller/sentry_controller/pure_pursuit.py` | removed old controller implementation |
| `src/sentry_controller/sentry_controller/traj_publisher.py` | removed fixed test publisher |

Do not look for old controllers when debugging current overshoot or wall-risk behavior; start with `traj_tracker`.
