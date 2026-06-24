# Planning Validation

Use this document to decide whether a planning change is acceptable. It is not a tuning guide; use `docs/runbooks/debug_planning.md` for debugging.

## Scope

This validation covers:

```text
/perception/costmap_2d
  -> costmap_inflator
  -> jps_node
  -> minco_planner_node
  -> /planner/path_vis
  -> /planner/traj_samples
```

Controller tracking is covered separately in `control_validation.md`.

## Preconditions

Before judging planning:

```bash
ros2 topic hz /perception/costmap_2d
ros2 topic hz /planner/costmap_inflated
ros2 node list | grep -E "costmap_inflator|jps_node|minco_planner_node"
```

RViz should show:

- `/perception/costmap_2d` aligned with the field;
- `/planner/path` after a goal;
- `/planner/path_vis` or `/planner/minco_traj` after MINCO.

## Required Test Routes

Use at least these route types after a planning change:

| Route | Purpose |
|---|---|
| Long S curve | exposes speed/timing, curvature, and fallback issues |
| 90 degree corner near wall | exposes wall-hugging and corner-cut risk |
| Narrow but valid passage | checks that JPS inflation did not make feasible space infeasible |
| Open-space goal | verifies normal path generation without obstacles dominating |

Use the same start state and goal positions when comparing before/after changes.

## Required Log Capture

Save the relevant planner log lines:

```bash
grep -E "JPS|MINCO|final check|fallback|min_d|wp_shift|T=|max_v|max_a|a_viol" /tmp/planner.log 2>/dev/null | tail -n 120
```

Also capture topic existence:

```bash
ros2 topic echo /planner/path --once
ros2 topic echo /planner/traj_samples --once
```

## Acceptance Criteria

| Criterion | Pass Condition |
|---|---|
| JPS path exists | `/planner/path` publishes after a valid goal |
| MINCO output exists | `/planner/traj_samples` publishes after JPS |
| Final check | planner log shows `MINCO final check OK` for nominal routes |
| No fallback on nominal routes | no `falling back to JPS path` on normal S/corner/open-space tests |
| Clearance | `min_d >= d_hard` in final check |
| Dynamics | `a_viol=0` is preferred; any nonzero value must be justified |
| Smoothness | RViz trajectory should not collapse into a JPS-like polyline unless fallback is expected |
| Topology | trajectory must stay in the same intended passage, not cut through obstacles |
| Wall risk | 90 degree corner route must visibly keep enough margin for tracker error |

Current hard clearance parameter:

```text
d_hard = 0.25
```

## Metrics To Record

| Metric | Source |
|---|---|
| JPS success/failure | planner log |
| waypoint count | planner log |
| `wp_shift=max/avg` | planner log |
| trajectory duration `T` | planner log |
| `max_v` | planner log |
| `max_a` | planner log |
| `a_viol` | planner log |
| `min_d` | planner log |
| fallback count | planner log |
| visual corner clearance | RViz/Gazebo observation |

## Known Failure Signatures

### Fallback

```text
MINCO final check FAILED
falling back to JPS path
```

Result: do not judge controller performance from this route until planning fallback is understood.

### Over-Aggressive Segment Timing

Known failed experiment:

```text
t_min = 0.5
```

Observed symptoms:

- faster trajectory;
- less smooth, more polyline-like path;
- high `max_a`;
- nonzero `a_viol`;
- final-check failure or wall-risk fallback.

Current validated default:

```text
t_min = 1.0
```

### Waypoint Bound Saturation

If `wp_shift=max` is repeatedly near `waypoint_bound_m`, then MINCO is asking for more movement than the current local bound allows. Do not automatically increase the bound; first check whether topology and clearance remain valid.

## Pass/Fail Template

```text
Planning validation:
  date:
  route:
  git/build state:
  goal:
  JPS success:
  MINCO final check:
  fallback:
  min_d:
  T:
  max_v:
  max_a:
  a_viol:
  wp_shift max/avg:
  RViz wall-risk observation:
  verdict: pass / fail / inconclusive
```

