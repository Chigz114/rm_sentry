# MINCO

## Purpose

Convert the JPS waypoint path into a smoother timed trajectory with soft clearance, soft reference, dynamic penalties, and bounded waypoint movement.

## Runtime Status

Active in Stage 3.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/planner/path` | `nav_msgs/Path` | planning frame | goal-triggered | `jps_node` | topology/reference path |
| `/perception/costmap_2d` | `nav_msgs/OccupancyGrid` | `map` | mapper-dependent | traversability_mapper | distance/collision source |
| `/odom` | `nav_msgs/Odometry` | sim odom | odom-dependent | Gazebo/chassis | current pose source |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/planner/traj_samples` | `std_msgs/Float64MultiArray` | planning frame data | on plan | `traj_tracker` | rows `[t,x,y,vx,vy,ax,ay,yaw]` |
| `/planner/path_vis` | `nav_msgs/Path` | planning frame | on plan | RViz | dense path visualization |
| `/planner/minco_traj` | `visualization_msgs/MarkerArray` | planning frame | on plan | RViz | trajectory markers |
| `/planner/minco_info` | `visualization_msgs/Marker` | planning frame | on plan | RViz | info/debug marker |

## Internal Mechanism

The planner post-processes JPS waypoints, allocates segment durations, solves a 2D MINCO-style polynomial trajectory optimization, evaluates collision/dynamic constraints, publishes timed samples, and falls back when final checks fail.

## State

- Latest path, odometry, and costmap.
- Internal occupancy-derived distance representation.
- Optimizer configuration and last planned trajectory.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `v_max` | `6.0` | `sim_planner.launch.py` | permits faster trajectory | caps trajectory speed lower |
| `a_max` | `16.0` | `sim_planner.launch.py` | permits sharper acceleration | more conservative feasibility |
| `v_alloc` | `3.0` | `sim_planner.launch.py` | shorter initial durations | longer initial durations |
| `w_time` | `100.0` | `sim_planner.launch.py` | more time reduction pressure | smoother/slower tendency |
| `w_obs` | `3000.0` | `sim_planner.launch.py` | stronger soft clearance | weaker soft clearance |
| `w_collision` | `10000.0` | `sim_planner.launch.py` | stronger hard-clearance avoidance | weaker hard-clearance avoidance |
| `d_soft` | `0.50` | `sim_planner.launch.py` | clearance cost starts farther out | clearance cost starts closer |
| `d_hard` | `0.25` | `sim_planner.launch.py` | stricter final clearance | less strict final clearance |
| `w_ref` | `20.0` | `sim_planner.launch.py` | stays closer to JPS waypoints | allows more waypoint movement |
| `waypoint_bound_m` | `1.00` | `sim_planner.launch.py` | more local movement possible | less local movement possible |
| `t_min` | `1.0` | `sim_planner.launch.py` | longer minimum segments | faster but riskier segments |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| `MINCO final check FAILED` | clearance/dynamic check failed | `/tmp/planner.log` |
| fallback path looks polyline-like | optimizer output rejected or fallback active | grep fallback/final check |
| high `a_viol` | timing too aggressive or constraints mismatched | `max_a`, `a_viol`, `t_min` |
| wall risk at 90 degree corner | clearance/reference/bound balance insufficient or tracker error margin too large | `min_d`, `wp_shift`, control validation |

## Code Map

| Role | File or Function |
|---|---|
| Stage 3 launch | `src/sentry_planner/launch/sim_planner.launch.py` |
| ROS wrapper | `src/sentry_planner/sentry_planner/minco_planner_node.py` |
| Solver | `src/sentry_planner/sentry_planner/minco_solver_2d.py` |
| Path post-processing | `src/sentry_planner/sentry_planner/path_postprocess.py` |
| Internal distance helper | `src/sentry_planner/sentry_planner/esdf_map_2d.py` |

## Validation Hooks

Use `docs/testbook/planning_validation.md` and `docs/runbooks/debug_planning.md`.

Quick checks:

```bash
grep -E "MINCO|final check|fallback|min_d|wp_shift|T=|max_v|max_a|a_viol" /tmp/planner.log 2>/dev/null | tail -n 120
ros2 topic echo /planner/traj_samples --once
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- Future speed work should be based on evidence; do not reapply `t_min=0.5` as a blind speed fix.
