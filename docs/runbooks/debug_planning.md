# Debug Runbook: Planning, JPS, MINCO, Fallback, And Wall Risk

Use this when JPS fails, paths hug walls, MINCO falls back, trajectories look polyline-like, 90 degree corners are risky, or speed changes make planning worse.

Current active planning chain:

```text
/perception/costmap_2d
  -> costmap_inflator
  -> /planner/costmap_inflated
  -> jps_node
  -> /planner/path
  -> minco_planner_node
  -> /planner/path_vis + /planner/traj_samples + /planner/minco_traj
```

## First Checks

Confirm active planning nodes:

```bash
ros2 node list | grep -E "costmap_inflator|jps_node|minco_planner_node"
```

Confirm core topics:

```bash
ros2 topic hz /perception/costmap_2d
ros2 topic hz /planner/costmap_inflated
ros2 topic echo /planner/path --once
ros2 topic echo /planner/traj_samples --once
```

Inspect planner logs:

```bash
grep -E "JPS|MINCO|final check|fallback|wp_shift|max_v|max_a|a_viol|min_d" /tmp/planner.log 2>/dev/null | tail -n 100
```

## JPS Failure Or No Path

Likely causes:

- goal is inside occupied or inflated space;
- costmap frame or origin is wrong;
- costmap is stale or empty;
- inflation is too large for the passage;
- clicked RViz goal is in the wrong fixed frame.

Check:

```bash
ros2 topic echo /perception/costmap_2d --once
ros2 topic echo /planner/costmap_inflated --once
ros2 topic echo /goal_pose --once
```

RViz expectations:

- Fixed Frame should be `map`.
- `/perception/costmap_2d` should align with walls/platforms.
- `/planner/costmap_inflated` should be visibly more conservative than the raw costmap, but not seal valid corridors.

Known current value:

```text
inflation_radius_m = 0.30
```

Do not increase inflation as the first response to real trajectory wall risk. Increased JPS inflation can make feasible passages falsely infeasible.

## MINCO Fallback

MINCO fallback means the optimized trajectory failed a final safety or feasibility check and the node fell back to a JPS-like path.

Log signatures:

```text
MINCO final check FAILED
falling back to JPS path
JPS fallback min_d=...
a_viol=...
```

Check:

```bash
grep -E "MINCO final check|fallback|min_d|a_viol|max_a|max_v|T=" /tmp/planner.log 2>/dev/null | tail -n 80
```

Interpretation:

| Signal | Meaning |
|---|---|
| `min_d < d_hard` | optimized path too close to obstacle |
| high `max_a` | timing too aggressive or corner too sharp |
| `a_viol > 0` | dynamic limits violated in sampled checks |
| fallback path visually angular | controller may receive a less smooth path |

Immediate next action:

- If fallback happens, debug MINCO feasibility before tuning controller gains.

## Path Looks Polyline-Like Or Cuts 90 Degree Corners

Likely causes:

- MINCO fell back to JPS;
- segment timing is too short;
- waypoint constraints are too tight or too loose;
- obstacle/clearance weights are not enough to move path inward;
- JPS waypoint geometry is too close to wall and MINCO does not have enough room to round the corner.

Check planner log:

```bash
grep -E "MINCO:|wp_shift|final check|T=|max_v|max_a|a_viol" /tmp/planner.log 2>/dev/null | tail -n 80
```

Known failed speed experiment:

```text
t_min: 1.0 -> 0.5
```

Observed failure:

- trajectory duration shortened and speed increased;
- path became visually less smooth/more polyline-like;
- final check could fail with `min_d < d_hard`;
- fallback path had wall-contact risk.

Current decision:

```text
t_min = 1.0
```

Future speed increases should use duration repair, curvature-aware time allocation, or stronger feasibility handling instead of simply lowering `t_min`.

## Wall-Hugging Or 90 Degree Corner Risk

Separate two cases:

### Case A: Planned Path Itself Is Too Close

Check RViz:

- `/planner/path` is the JPS path.
- `/planner/path_vis` is the MINCO dense path.
- `/planner/minco_traj` is the MINCO marker output.

If JPS is wall-adjacent but MINCO moves inward, the current design is working.

If MINCO stays wall-adjacent, check:

| Parameter | Current Value | Effect |
|---|---:|---|
| `w_ref` | `20.0` | pulls optimized waypoints toward original JPS |
| `waypoint_bound_m` | `1.00` | caps waypoint movement from JPS |
| `d_soft` | `0.50` | starts soft clearance pressure |
| `d_hard` | `0.25` | hard final clearance threshold |
| `w_obs` | `3000.0` | soft obstacle cost |
| `w_collision` | `10000.0` | hard obstacle cost |

Use `wp_shift=max/avg`:

- near zero: MINCO is not moving away from JPS enough;
- often near `1.00m`: waypoint bound is saturated and topology/safety must be reviewed before increasing it.

### Case B: Planned Path Is Safe But Real Motion Risks Collision

Do not solve this only in JPS. Check controller tracking:

- `/tmp/traj_tracker_debug.csv`;
- `control_dt`;
- tracking error near the corner;
- command saturation;
- offtrack speed limiting.

Use [`debug_control.md`](debug_control.md) for the controller branch.

## Current MINCO Parameters

Effective source:

- `src/sentry_planner/launch/sim_planner.launch.py`

High-risk current values:

| Parameter | Current Value | Meaning |
|---|---:|---|
| `v_max` | `6.0` | velocity soft limit |
| `a_max` | `16.0` | acceleration soft limit |
| `v_alloc` | `3.0` | initial time allocation speed |
| `w_time` | `100.0` | pressure to shorten time |
| `w_obs` | `3000.0` | soft clearance pressure |
| `w_collision` | `10000.0` | hard clearance pressure |
| `d_soft` | `0.50` | desired clearance onset |
| `d_hard` | `0.25` | final hard clearance threshold |
| `w_ref` | `20.0` | soft reference to JPS waypoints |
| `waypoint_bound_m` | `1.00` | local movement bound |
| `w_dyn` | `500.0` | dynamic feasibility penalty |
| `min_spacing` | `0.8` | waypoint spacing lower bound |
| `max_spacing` | `1.5` | waypoint spacing upper bound |
| `sample_dt` | `0.05` | trajectory sample interval |
| `max_iter` | `150` | optimizer iteration cap |
| `t_min` | `1.0` | minimum segment time |

## Safe Tuning Order

For wall-adjacent but valid JPS:

1. Confirm MINCO did not fallback.
2. Inspect `wp_shift=max/avg`.
3. Inspect `min_d`, `max_a`, and `a_viol`.
4. Consider reducing `w_ref` slightly or increasing clearance pressure only if MINCO is not moving enough.
5. Consider `waypoint_bound_m` only if shifts are not saturated and topology remains clear.
6. Re-run the same route and compare planner log plus controller log.

For speed:

1. Inspect trajectory duration `T` and reference `max_v`.
2. Confirm no fallback.
3. Confirm `max_a` and `a_viol` are acceptable.
4. Avoid lowering `t_min` without feasibility repair.
5. If speed remains too low, investigate time allocation and curvature-aware duration rather than controller-only gains.

## Minimal Validation After A Planning Change

Record these from the same test route:

| Metric | Source |
|---|---|
| JPS success/failure | planner log |
| `MINCO final check OK/FAILED` | planner log |
| `min_d` | planner log |
| `wp_shift=max/avg` | planner log |
| trajectory duration `T` | planner log |
| `max_v`, `max_a`, `a_viol` | planner log |
| visual distance from 90 degree corners | RViz/Gazebo |
| tracking error after planning change | `/tmp/traj_tracker_debug.csv` |

If a planning change improves visual smoothness but increases fallback or tracking error, treat it as not validated.

