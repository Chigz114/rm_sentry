# Traj Tracker

## Purpose

Track the timed MINCO trajectory and publish chassis velocity commands.

## Runtime Status

Active in Stage 3.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/planner/traj_samples` | `std_msgs/Float64MultiArray` | planning frame data | on plan | MINCO | rows `[t,x,y,vx,vy,ax,ay,yaw]` |
| `/odom` | `nav_msgs/Odometry` | sim odom | odom-dependent | Gazebo/chassis | current Stage 3 pose source |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/cmd_vel_chassis` | `geometry_msgs/Twist` | chassis command | timer-driven | Gazebo mecanum/chassis plugin | velocity command |
| `/tmp/traj_tracker_debug.csv` | CSV file | n/a | timer-driven when enabled | humans/agents | tracking diagnostics |

## Internal Mechanism

The tracker samples the timed trajectory with lookahead, computes feedforward plus position/velocity feedback, applies speed and acceleration limits, caps off-track speed, and writes debug diagnostics. Acceleration limiting uses measured control dt rather than only the configured timer period.

## State

- Current trajectory sample buffer.
- Latest odometry.
- Last command and last control timestamp.
- Progress anchor/reference state.
- Debug CSV writer when enabled.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `v_max` | `6.0` | `sim_planner.launch.py` | higher command speed cap | lower command speed cap |
| `acc_lim` | `12.0` | `sim_planner.launch.py` | faster command changes | slower acceleration/braking |
| `kp_pos` | `2.0` | `sim_planner.launch.py` | stronger position correction | weaker correction |
| `kd_vel` | `0.8` | `sim_planner.launch.py` | stronger velocity damping | less damping |
| `rate_hz` | `30.0` | `sim_planner.launch.py` | desired timer rate | lower desired timer rate |
| `lookahead_time` | `0.25` | `sim_planner.launch.py` | farther reference lookahead | closer reference |
| `max_feedback_speed` | `2.0` | `sim_planner.launch.py` | larger feedback correction speed | smaller correction speed |
| `offtrack_v_max` | `2.0` | `sim_planner.launch.py` | higher speed while off-track | more conservative recovery |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| large overshoot with slow correction | acceleration limit, odom rate, or dt issue | `control_dt` and command columns in debug CSV |
| robot leaves path then returns before continuing | offtrack protection active | `nearest_d`, offtrack duration |
| speed lower than expected | planning reference speed may be low | compare reference speed and command speed |
| no command | missing trajectory, missing odom, or no subscriber | topic info for `/planner/traj_samples`, `/odom`, `/cmd_vel_chassis` |

## Code Map

| Role | File or Function |
|---|---|
| Stage 3 launch | `src/sentry_planner/launch/sim_planner.launch.py` |
| Tracker implementation | `src/sentry_controller/sentry_controller/traj_tracker.py` |

## Validation Hooks

Use `docs/testbook/control_validation.md` and `docs/runbooks/debug_control.md`.

Quick checks:

```bash
ros2 topic info /planner/traj_samples -v
ros2 topic info /cmd_vel_chassis -v
tail -n 20 /tmp/traj_tracker_debug.csv
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- MPC remains a future option if timed trajectory tracking plus trajectory shaping cannot meet speed/safety targets.
