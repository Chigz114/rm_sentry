# Control Validation

Use this document to decide whether a control or speed change is acceptable. It is not a tuning guide; use `docs/runbooks/debug_control.md` for debugging.

## Scope

This validation covers:

```text
/planner/traj_samples
  -> traj_tracker
  -> /cmd_vel_chassis
  -> Gazebo chassis response
  -> /odom
```

Planning validity must be checked first. Do not validate controller behavior on a route where MINCO fell back unexpectedly.

## Preconditions

Confirm active controller and trajectory input:

```bash
ros2 node list | grep traj_tracker
ros2 topic info /planner/traj_samples -v
ros2 topic info /cmd_vel_chassis -v
ros2 topic hz /odom
ls -lh /tmp/traj_tracker_debug.csv
```

Confirm planning did not fail:

```bash
grep -E "MINCO final check|fallback|a_viol|max_a" /tmp/planner.log 2>/dev/null | tail -n 80
```

## Required Test Routes

| Route | Purpose |
|---|---|
| Long S curve | exposes overshoot, timing, and speed saturation |
| 90 degree corner | exposes lateral tracking and braking |
| Open straight path | exposes speed ceiling and acceleration response |
| Stop-at-goal route | checks final deceleration and goal behavior |

Use the same route before and after any control change.

## Required Log Capture

Tracker CSV:

```bash
cp /tmp/traj_tracker_debug.csv /tmp/traj_tracker_debug_$(date +%Y%m%d_%H%M%S).csv
tail -n 20 /tmp/traj_tracker_debug.csv
```

Topic rates:

```bash
ros2 topic hz /odom
ros2 topic hz /planner/traj_samples
```

Planner health:

```bash
grep -E "MINCO final check|fallback|T=|max_v|max_a|a_viol" /tmp/planner.log 2>/dev/null | tail -n 80
```

## Acceptance Criteria

| Criterion | Pass Condition |
|---|---|
| Active tracker | `traj_tracker` is running |
| Command output | `/cmd_vel_chassis` has a subscriber and receives commands |
| Control dt | `control_dt` is finite and near actual callback interval |
| Odom rate | `/odom` rate is stable enough for the test |
| Overshoot | peak tracking error is not worse than baseline for same route |
| Recovery shape | if off-track, robot returns without large secondary oscillation |
| Speed | achieved/reference speed matches the intent of the test |
| Saturation | command caps are understood, not accidental |
| Goal stop | robot stops within `goal_tol` without persistent command output |
| Safety | real path remains clear of walls in high-risk turns |

Current controller values of interest:

```text
v_max = 6.0
acc_lim = 12.0
max_feedback_speed = 2.0
offtrack_v_max = 2.0
goal_tol = 0.18
```

## Metrics To Record

| Metric | Source |
|---|---|
| peak tracking error | `/tmp/traj_tracker_debug.csv` |
| mean tracking error | `/tmp/traj_tracker_debug.csv` |
| peak reference speed | `/tmp/traj_tracker_debug.csv` or `/planner/traj_samples` |
| peak command speed | `/tmp/traj_tracker_debug.csv` or `/cmd_vel_chassis` |
| `control_dt` min/mean/max | `/tmp/traj_tracker_debug.csv` |
| offtrack duration | `/tmp/traj_tracker_debug.csv` |
| goal stop behavior | RViz/Gazebo and command output |
| visible wall margin | RViz/Gazebo |

## Known Failure Signatures

### Fixed-Dt Acceleration Limit Regression

Symptom:

- configured control rate looks high;
- actual callback is slower;
- robot overshoots because acceleration changes are effectively too small.

Required check:

```bash
tail -n 200 /tmp/traj_tracker_debug.csv
```

`traj_tracker` should use measured `control_dt`, not only `1/rate_hz`.

### Planning-Limited Speed

Symptom:

- tracker `v_max` is high;
- robot still moves slowly;
- reference velocity in `/planner/traj_samples` is low.

Likely cause:

- MINCO duration/timing, not controller cap.

Check planning logs before changing controller gains.

### Offtrack Speed Cap Dominates

Symptom:

- robot slows sharply after leaving trajectory;
- it returns to the path before continuing.

Interpretation:

- offtrack protection is active. This may be correct. Judge whether peak error and wall margin are acceptable.

## Pass/Fail Template

```text
Control validation:
  date:
  route:
  git/build state:
  planner final check:
  fallback:
  odom rate:
  traj sample rate:
  control_dt min/mean/max:
  peak tracking error:
  mean tracking error:
  peak reference speed:
  peak command speed:
  offtrack duration:
  wall-risk observation:
  goal stop behavior:
  verdict: pass / fail / inconclusive
```

