# JPS

## Purpose

Find a grid path from the current robot pose to the requested goal on the inflated planning costmap.

## Runtime Status

Active in Stage 3.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/planner/costmap_inflated` | `nav_msgs/OccupancyGrid` | costmap frame | costmap-dependent | costmap_inflator | search grid |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz/goal frame | event-driven | RViz 2D Goal Pose or CLI | planning target |
| `/odom` | `nav_msgs/Odometry` | sim odom | odom-dependent | Gazebo/chassis | current Stage 3 pose source |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/planner/path` | `nav_msgs/Path` | planning grid frame | goal-triggered | MINCO, RViz | JPS waypoint path |
| `/planner/jps_viz` | `visualization_msgs/MarkerArray` | planning grid frame | goal-triggered | RViz | visualization markers |

## Internal Mechanism

JPS performs grid search on the inflated occupancy grid. In current sim launch it uses raw `/odom` position directly instead of transforming FAST-LIO odometry.

## State

- Latest costmap.
- Latest odometry pose.
- Goal pose until replanning.
- Search limits and goal tolerance.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `max_iter` | `200000` | `sim_planner.launch.py` | larger search budget | faster failure, possible missed path |
| `goal_tolerance_m` | `0.15` | `sim_planner.launch.py` | easier goal acceptance | stricter goal reach |
| `use_raw_odom` | `True` | `sim_planner.launch.py` | bypasses TF in sim | would use transform path if disabled |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| no `/planner/path` after goal | no costmap, bad goal frame, blocked grid, or search failure | `ros2 topic echo /planner/costmap_inflated --once` |
| path starts from wrong place | odom/frame mismatch | `/odom` and RViz fixed frame |
| path too close to wall | expected with thin inflation in some cases | MINCO clearance and wall-risk validation |

## Code Map

| Role | File or Function |
|---|---|
| Stage 3 launch | `src/sentry_planner/launch/sim_planner.launch.py` |
| Node implementation | `src/sentry_planner/sentry_planner/jps_node.py` |

## Validation Hooks

Use `docs/testbook/planning_validation.md` and `docs/runbooks/debug_planning.md`.

Quick checks:

```bash
ros2 topic echo /planner/path --once
ros2 topic echo /planner/jps_viz --once
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- Real-robot operation should revisit whether Stage 3 still uses raw `/odom` or a mapped pose source.
