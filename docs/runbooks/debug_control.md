# Debug Runbook: Control, Overshoot, Speed, And Chassis Response

Use this when the robot overshoots, drifts, moves slower than expected, oscillates, or appears to ignore trajectory timing.

Current active controller:

```text
/planner/traj_samples -> traj_tracker -> /cmd_vel_chassis -> Gazebo mecanum/chassis plugin
```

Do not start with `pure_pursuit`, `path_tracker`, or `goal_controller`; they are not the current Stage 3 controller.

## First Checks

Confirm the active controller exists:

```bash
ros2 node list | grep traj_tracker
ros2 topic info /planner/traj_samples -v
ros2 topic info /cmd_vel_chassis -v
```

Confirm Stage 3 is receiving pose and trajectory:

```bash
ros2 topic hz /odom
ros2 topic echo /planner/traj_samples --once
```

Inspect current tracker diagnostics:

```bash
ls -lh /tmp/traj_tracker_debug.csv
tail -n 20 /tmp/traj_tracker_debug.csv
```

Important fields to look for:

| Field Or Signal | What It Tells You |
|---|---|
| `control_dt` | actual controller interval used for acceleration limiting |
| `nearest_d` or tracking error | geometric tracking error |
| reference velocity magnitude | whether MINCO is actually asking for higher speed |
| command magnitude | whether tracker is saturating or under-commanding |
| acceleration limiting signs | whether commands cannot change quickly enough |

## Overshoot Triage

### Case A: Large Overshoot With Low Command Responsiveness

Likely causes:

- actual callback interval is much larger than expected;
- `control_dt` is wrong or stale;
- chassis odom/plugin publish rate is too low;
- acceleration limit is effectively too low.

Check:

```bash
ros2 topic hz /odom
grep -m 5 -n "control_dt" /tmp/traj_tracker_debug.csv 2>/dev/null || head -n 1 /tmp/traj_tracker_debug.csv
tail -n 50 /tmp/traj_tracker_debug.csv
```

Expected reasoning:

- The configured `rate_hz=30.0` is a desired timer rate, not proof of actual control frequency.
- `traj_tracker` must use measured `control_dt`. A previous major overshoot root cause was using fixed `1/30s` while callbacks were closer to about `0.1s`, which made acceleration changes too weak.

Do not fix this by simply lowering global speed before confirming `control_dt`, `/odom` frequency, and command saturation.

### Case B: Robot Leaves Path, Then Returns To Path

Likely causes:

- feedback is working, but trajectory timing or acceleration is still too aggressive;
- offtrack speed limiting is active;
- MINCO curve is feasible geometrically but too fast locally;
- real/sim actuation delay or update rate is limiting correction.

Check:

```bash
tail -n 200 /tmp/traj_tracker_debug.csv
grep -E "TrajTracker|control_dt|offtrack|goal" /tmp/planner.log 2>/dev/null | tail -n 50
```

Look for:

- tracking error peaks near sharp turns;
- command speed clipped by `offtrack_v_max`;
- feedback velocity capped by `max_feedback_speed`;
- reference speed stays high entering a sharp bend.

Next actions:

- If reference speed is too high near a turn, debug planning/timing first.
- If reference is reasonable but command cannot follow, debug controller/chassis response.

### Case C: Speed Is Lower Than Expected

Likely causes:

- MINCO trajectory duration is long, so `/planner/traj_samples` reference speed is low;
- `v_alloc` or `t_min` is limiting duration;
- tracker caps speed through `v_max`, `offtrack_v_max`, or feedback saturation;
- robot is often off-track, triggering lower speed behavior.

Check planner logs first:

```bash
grep -E "MINCO:|max_v|max_a|T=|a_viol|wp_shift|final check" /tmp/planner.log 2>/dev/null | tail -n 50
```

Then check tracker diagnostics:

```bash
tail -n 200 /tmp/traj_tracker_debug.csv
```

Known current speed constraint:

- `t_min=1.0` prevents very short segment durations when JPS/MINCO creates many segments.
- `t_min=0.5` was tried and reverted because it made paths more polyline-like, caused final-check/fallback risk, and increased wall-contact risk.

Do not reapply `t_min=0.5` as a simple speed fix without adding duration repair or stronger feasibility validation.

## Current Controller Parameters

Effective source:

- `src/sentry_planner/launch/sim_planner.launch.py`
- `src/sentry_controller/sentry_controller/traj_tracker.py`

High-risk current values:

| Parameter | Current Value | Debug Meaning |
|---|---:|---|
| `v_max` | `6.0` | command speed cap |
| `acc_lim` | `12.0` | command acceleration cap using actual `control_dt` |
| `kp_pos` | `2.0` | position feedback gain |
| `kd_vel` | `0.8` | velocity damping |
| `lookahead_time` | `0.25` | future reference time |
| `max_feedback_speed` | `2.0` | feedback correction speed cap |
| `offtrack_v_max` | `2.0` | speed cap when far from trajectory |
| `debug_csv_path` | `/tmp/traj_tracker_debug.csv` | primary controller log |

## Chassis And Gazebo Checks

The current command topic is:

```text
/cmd_vel_chassis
```

Check that something subscribes:

```bash
ros2 topic info /cmd_vel_chassis -v
```

Check odom frequency:

```bash
ros2 topic hz /odom
```

Simulation integration files of interest:

- `src/pb_rm_simulation/src/rm_nav_bringup/urdf/sentry_robot_sim.xacro`
- `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/urdf/simulation_waking_robot.xacro`

The mecanum plugin publish rate was raised to `50`. Xacro/plugin changes require a full Gazebo restart to take effect.

## When To Consider MPC

Do not jump to MPC before answering:

1. Is `/planner/traj_samples` locally feasible?
2. Is MINCO passing final clearance and dynamic checks?
3. Is `control_dt` sane?
4. Is `/odom` frequency sane?
5. Is the command saturating due to `v_max`, `acc_lim`, `offtrack_v_max`, or feedback caps?

MPC becomes a better next step if timed tracking is healthy, the chassis response is healthy, and overshoot remains dominated by model-prediction/latency effects rather than trajectory infeasibility or update-rate limits.

## Minimal Validation After A Control Change

Run the same route before and after the change. Record:

| Metric | Source |
|---|---|
| peak tracking error | `/tmp/traj_tracker_debug.csv` |
| mean tracking error | `/tmp/traj_tracker_debug.csv` |
| max command speed | `/tmp/traj_tracker_debug.csv` or `/cmd_vel_chassis` |
| max reference speed | `/planner/traj_samples` or planner log |
| `control_dt` range | `/tmp/traj_tracker_debug.csv` |
| visible wall contact risk | RViz/Gazebo |

If only visual smoothness improved but tracking error or wall risk worsened, keep debugging.

