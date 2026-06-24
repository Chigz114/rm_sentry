# Navigation Decisions

This document records durable decisions for the current sim navigation stack. It is not a complete experiment log; see `docs/height_gated_traversability_plan.md` for detailed history.

## Decision: Replace ROG-Map Runtime With Height-Gated Traversability In Simulation

Status: active.

Context:

- The simulated MID360/LiDAR mounting and low platform geometry made full 3D ROG-Map raycasting clear some 0.2 m platform obstacles as free.
- The project needed a robust 2.5D planning costmap for the RM3V3 simulation field.

Decision:

- Current sim perception uses `traversability_mapper`, a direct height-gated 2.5D mapper, instead of launching ROG-Map.
- ROG-Map files remain in the workspace as reference/fallback and for real-data lineage.

Consequence:

- Stage 2 now produces `/perception/costmap_2d` directly from `/cloud_registered`, `/Odometry`, a static world prior, and map-frame TF.
- ROG tuning is not the first place to debug current sim planning failures.

## Decision: Use Relocalization To Publish `map -> lidar_odom`

Status: active.

Context:

- Planning and RViz need a stable field/world frame.
- FAST-LIO naturally publishes in a local odometry frame.

Decision:

- `relocalization_node` publishes `map -> lidar_odom`.
- It starts from a birth-zone seed and can refine with 2D ICP against the generated world prior.

Consequence:

- Stage 2 outputs costmap/ESDF in `map`.
- `/relocalization/status` is a required health signal.

## Decision: Keep JPS Inflation Thin And Move Safety Responsibility Downstream

Status: active.

Context:

- Large binary inflation can make narrow but feasible corridors appear blocked.
- Real motion error means a raw JPS path near walls is still risky.

Decision:

- JPS uses relatively thin inflation, currently `inflation_radius_m=0.30`.
- JPS is treated mainly as a topology provider.
- MINCO is responsible for smoothing and clearance improvement using ESDF-related costs and bounded waypoint movement.

Consequence:

- Avoidance safety is split between JPS feasibility and MINCO clearance.
- Do not blindly increase JPS inflation to fix wall contact risk.

## Decision: Use MINCO Soft Reference And Bounded Waypoint Motion

Status: active.

Context:

- JPS often returns wall-adjacent or corner-cutting waypoint sequences.
- Forcing MINCO to pass through every JPS waypoint keeps the path too close to obstacles.
- Allowing unlimited waypoint motion can change topology or cut through invalid regions.

Decision:

- MINCO uses a soft reference cost `w_ref` toward original JPS waypoints.
- MINCO applies local bounds through `waypoint_bound_m`.
- Current values are `w_ref=20.0` and `waypoint_bound_m=1.00`.

Consequence:

- MINCO can round 90 degree corners and move away from walls while preserving the JPS-provided passage.
- Watch `wp_shift=max/avg` in planner logs; frequent saturation means the bound/reference balance may need review.

## Decision: Track Timed MINCO Samples With `traj_tracker`

Status: active.

Context:

- `pure_pursuit` and earlier geometric trackers did not use MINCO's time, velocity, or acceleration information.
- Reactive path following changed overshoot shape but did not fully solve corner overshoot.

Decision:

- MINCO publishes `/planner/traj_samples` rows `[t,x,y,vx,vy,ax,ay,yaw]`.
- `traj_tracker` tracks the timed trajectory and publishes `/cmd_vel_chassis`.

Consequence:

- `path_tracker`, `goal_controller`, and `pure_pursuit` are no longer the active Stage 3 controller.
- Future MPC work should replace only the control law after the timed trajectory interface is validated.

## Decision: Do Not Jump Directly To MPC Yet

Status: current strategy.

Context:

- MPC is useful, but it would add solver, horizon, model, and tuning complexity.
- A major overshoot root cause was found in lower-level timing/acceleration behavior, not only in control structure.

Decision:

- First stabilize timed trajectory tracking and simulation responsiveness.
- Keep MPC as a next-stage option if timed tracking plus trajectory shaping cannot meet performance targets.

Consequence:

- Current debugging should inspect MINCO timing, acceleration feasibility, tracker `control_dt`, command saturation, and odom/update rates before implementing MPC.

## Decision: Use Actual Control Dt For Acceleration Limiting

Status: active.

Context:

- The tracker desired rate can differ from actual callback timing in simulation.
- Using fixed `1/rate_hz` caused effective acceleration limits to be much lower when callbacks were slower, delaying braking and lateral correction.

Decision:

- `traj_tracker` uses measured `control_dt`, clamped to a safe interval, for acceleration limiting.

Consequence:

- `/tmp/traj_tracker_debug.csv` should be checked for `control_dt` when diagnosing overshoot.
- Timer configuration alone does not prove control frequency.

## Failed Attempt: Lowering `t_min` To `0.5` As A Speed Fix

Status: reverted.

Context:

- Higher speed was desired after acceleration and dt fixes reduced overshoot.
- MINCO total duration was constrained by many segments and `t_min=1.0`.

Attempt:

- `t_min` was lowered to `0.5`.

Observed result:

- The robot became faster, but paths looked less smooth and more polyline-like.
- MINCO final checks could fail, causing fallback to a JPS-like path with wall-contact risk.
- Recorded failure signatures included `min_d < d_hard`, high acceleration, and acceleration violations.

Decision:

- Revert to `t_min=1.0`.
- Future speed work should use duration repair, curvature-aware timing, or better trajectory feasibility handling rather than simply lowering `t_min`.

## Current Main Debug Order For Overshoot Or Wall Risk

1. Verify Stage 2 costmap frame and obstacle geometry.
2. Check MINCO logs: `MINCO final check`, `min_d`, `wp_shift`, `max_v`, `max_a`, `a_viol`.
3. Check `/tmp/traj_tracker_debug.csv`: `control_dt`, tracking error, reference speed, command magnitude, saturation.
4. Check Gazebo/odom publish rate and plugin settings.
5. Adjust MINCO/tracker parameters only after identifying whether the issue is geometry, timing, feasibility, or actuation.

