# 2D ESDF

## Purpose

Convert the active 2D occupancy grid into an ESDF-like point cloud for visualization and distance-field reference.

## Runtime Status

Active in Stage 2.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/perception/costmap_2d` | `nav_msgs/OccupancyGrid` | `map` | mapper-dependent | traversability_mapper | occupancy grid input |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/perception/esdf_2d` | `sensor_msgs/PointCloud2` | costmap frame | costmap callback | RViz/humans | intensity is distance plus offset |
| `/perception/esdf_2d_demo` | `sensor_msgs/PointCloud2` | shifted copy | costmap callback | RViz demo | enabled in current launch |

## Internal Mechanism

The node subscribes to the occupancy grid, computes distance to obstacles in 2D, clamps visualization distance, and publishes a point cloud where intensity encodes distance.

## State

The node mostly recomputes from each incoming costmap. It stores parameters and publishers/subscribers.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `treat_unknown_as_obstacle` | `False` | `sim_perception.launch.py` | unknown cells become more conservative if enabled | unknown remains free-like |
| `max_distance_m` | `5.0` | `sim_perception.launch.py` | larger visualization range | stronger clamp |
| `demo_copy_enable` | `True` | `sim_perception.launch.py` | publishes shifted copy | no demo copy |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| no ESDF cloud | no costmap or node missing | `ros2 topic hz /perception/esdf_2d` |
| ESDF frame wrong | costmap frame wrong upstream | inspect costmap header |
| MINCO behavior does not match ESDF visualization | MINCO uses occupancy grid/internal distance, not this cloud directly | check `minco_planner_node` inputs |

## Code Map

| Role | File or Function |
|---|---|
| Stage 2 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` |
| Node implementation | `src/sentry_perception/sentry_perception/esdf2d_node.py` |

## Validation Hooks

Use `docs/testbook/mapping_validation.md`.

Quick checks:

```bash
ros2 topic hz /perception/esdf_2d
ros2 topic echo /perception/esdf_2d --once
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- Whether future MINCO/MPC should consume a shared ESDF service or keep internal occupancy-derived distance is not decided here.
