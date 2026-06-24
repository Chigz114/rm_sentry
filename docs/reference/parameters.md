# Parameter Reference

This document lists high-risk effective parameters and where their source of truth lives. It is not a full copy of every yaml or launch file.

If a value here disagrees with launch/yaml/code or a running node, treat the executable source as authoritative and update this document.

## Source Priority

For current runtime behavior:

1. launch-time parameter overrides;
2. yaml files loaded by launch;
3. code defaults declared by the node;
4. historical docs and notes.

## Stage 1: FAST-LIO2

Effective source:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py`
- `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml`

High-value parameters:

| Parameter | Current Value | Source | Why It Matters |
|---|---:|---|---|
| `common.lid_topic` | `/livox/lidar` | FAST-LIO yaml | lidar input |
| `common.imu_topic` | `/imu/data` | FAST-LIO yaml | IMU input |
| `preprocess.lidar_type` | `1` | FAST-LIO yaml | Livox serial lidar mode |
| `preprocess.scan_line` | `4` | FAST-LIO yaml | MID360-style scan configuration |
| `preprocess.blind` | `0.5` | FAST-LIO yaml | ignores near-field points |
| `mapping.fov_degree` | `360.0` | FAST-LIO yaml | full horizontal field |
| `mapping.det_range` | `100.0` | FAST-LIO yaml | max detection range |
| `mapping.extrinsic_est_en` | `false` | FAST-LIO yaml | fixed extrinsic in sim |
| `publish.dense_publish_en` | `true` | FAST-LIO yaml | required for dense traversability input |

## Stage 2: Relocalization

Effective source:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py`
- defaults in `src/sentry_perception/sentry_perception/relocalization_node.py`

High-value launch overrides:

| Parameter | Current Value | Why It Matters |
|---|---:|---|
| `seed_x` | `5.5` | initial map-frame x |
| `seed_y` | `3.5` | initial map-frame y |
| `seed_yaw` | `0.15` | initial yaw rad |
| `world_file` | `pb_rm_simulation/world/RM3V3/rm3v3_sym_v1.world` via package share | ICP/static prior source |
| `accumulate_count` | `30` | scans before ICP refinement |
| `refine_with_icp` | `True` | enables ICP correction |
| `tf_rate_hz` | `50.0` | TF publish rate |
| `cloud_topic` | `/cloud_registered` | ICP source cloud |

Validation signal:

```bash
ros2 topic echo /relocalization/status --once
```

## Stage 2: Height-Gated Traversability

Effective source:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py`
- defaults in `src/sentry_mapping/src/traversability_mapper_node.cpp`

High-value launch overrides:

| Parameter | Current Value | Why It Matters |
|---|---:|---|
| `resolution` | `0.1` | costmap resolution |
| `width_m` | `13.0` | map width |
| `height_m` | `9.0` | map height |
| `x_offset` | `6.0` | map center x |
| `y_offset` | `4.0` | map center y |
| `h_climb` | `0.10` | obstacle height threshold |
| `ground_clamp_lo` | `-0.30` | ground z clamp lower bound |
| `ground_clamp_hi` | `-0.08` | ground z clamp upper bound |
| `ground_init` | `-0.14` | initial ground estimate |
| `n_min_near/mid/far` | `1/1/1` | hit count thresholds by distance |
| `delta_hit` | `1.0` | log-odds hit increment |
| `decay_tau` | `3.0` | temporal decay constant |
| `occ_thresh` | `0.5` | occupancy threshold |
| `rate_hz` | `10.0` | costmap publish loop |
| `frame_id` | `map` | output frame |
| `odom_frame` | `lidar_odom` | transform lookup target |

Couplings:

- `ground_clamp_lo`, `ground_clamp_hi`, and `ground_init` are coupled to the simulated registered-cloud ground height.
- `h_climb` controls which height differences become obstacles; changing it affects low platform detection.
- `decay_tau`, `delta_hit`, and hit thresholds jointly affect persistence and noise.

## Stage 2: 2D ESDF

Effective source:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py`
- defaults in `src/sentry_perception/sentry_perception/esdf2d_node.py`

High-value launch overrides:

| Parameter | Current Value | Why It Matters |
|---|---:|---|
| `costmap_topic` | `/perception/costmap_2d` | input occupancy grid |
| `esdf_topic` | `/perception/esdf_2d` | visualization/output cloud |
| `treat_unknown_as_obstacle` | `False` | unknown space is treated as free for distance expansion |
| `max_distance_m` | `5.0` | ESDF visualization clamp |
| `demo_copy_enable` | `True` | publishes shifted copy for demo view |

## Stage 3: Costmap Inflation + JPS

Effective source:

- `src/sentry_planner/launch/sim_planner.launch.py`
- defaults in `src/sentry_planner/sentry_planner/costmap_inflator.py`
- defaults in `src/sentry_planner/sentry_planner/jps_node.py`

High-value launch overrides:

| Node | Parameter | Current Value | Why It Matters |
|---|---|---:|---|
| costmap_inflator | `inflation_radius_m` | `0.30` | JPS obstacle inflation; keep thin enough for feasible passages |
| jps_node | `costmap_topic` | `/planner/costmap_inflated` | JPS input grid |
| jps_node | `odom_topic` | `/odom` | simulation pose source |
| jps_node | `use_raw_odom` | `True` | bypasses TF for sim pose |
| jps_node | `path_topic` | `/planner/path` | JPS output |
| jps_node | `max_iter` | `200000` | search budget |
| jps_node | `goal_tolerance_m` | `0.15` | goal acceptance tolerance |

Decision note:

- Do not solve real-trajectory wall-risk only by increasing JPS inflation. This can turn narrow feasible passages into false infeasible passages.

## Stage 3: MINCO

Effective source:

- `src/sentry_planner/launch/sim_planner.launch.py`
- defaults in `src/sentry_planner/sentry_planner/minco_planner_node.py`
- solver behavior in `src/sentry_planner/sentry_planner/minco_solver_2d.py`

High-value launch overrides:

| Parameter | Current Value | Why It Matters |
|---|---:|---|
| `v_max` | `6.0` | velocity soft limit |
| `a_max` | `16.0` | acceleration soft limit |
| `v_alloc` | `3.0` | initial duration allocation speed |
| `w_smooth` | `1.0` | smoothness weight |
| `w_time` | `100.0` | time reduction pressure |
| `w_obs` | `3000.0` | soft clearance cost |
| `w_collision` | `10000.0` | hard clearance cost |
| `d_soft` | `0.50` | preferred clearance onset |
| `d_hard` | `0.25` | final hard clearance threshold |
| `w_ref` | `20.0` | soft reference to original JPS waypoints |
| `waypoint_bound_m` | `1.00` | local bound around original JPS waypoint variables |
| `w_dyn` | `500.0` | dynamic limit penalty |
| `min_spacing` | `0.8` | waypoint postprocess lower spacing |
| `max_spacing` | `1.5` | waypoint postprocess upper spacing |
| `sample_dt` | `0.05` | trajectory sampling interval |
| `max_iter` | `150` | optimizer iteration cap |
| `t_min` | `1.0` | minimum segment duration |
| `odom_topic` | `/odom` | simulation pose source |
| `traj_samples_topic` | `/planner/traj_samples` | control-facing output |

Known high-risk change:

- `t_min=0.5` made the robot faster but caused trajectory/fallback behavior that looked less smooth and risked wall contact. Do not reapply it as a simple speed fix without duration repair and validation.

## Stage 3: Traj Tracker

Effective source:

- `src/sentry_planner/launch/sim_planner.launch.py`
- defaults in `src/sentry_controller/sentry_controller/traj_tracker.py`

High-value launch overrides:

| Parameter | Current Value | Why It Matters |
|---|---:|---|
| `v_max` | `6.0` | controller speed cap |
| `acc_lim` | `12.0` | acceleration limit using actual measured control dt |
| `kp_pos` | `2.0` | position feedback gain |
| `kd_vel` | `0.8` | velocity damping |
| `goal_tol` | `0.18` | goal completion tolerance |
| `rate_hz` | `30.0` | desired timer rate, not guaranteed actual rate |
| `odom_topic` | `/odom` | simulation pose source |
| `cmd_vel_topic` | `/cmd_vel_chassis` | command output |
| `traj_topic` | `/planner/traj_samples` | timed trajectory input |
| `lookahead_time` | `0.25` | reference lookahead |
| `max_feedback_speed` | `2.0` | feedback velocity cap |
| `offtrack_v_max` | `2.0` | speed cap when far from trajectory |
| `debug_traj` | `True` | writes debug CSV |
| `debug_csv_path` | `/tmp/traj_tracker_debug.csv` | controller diagnostics |

Critical implementation detail:

- Acceleration limiting must use actual `control_dt`, not only `1/rate_hz`. A previous overshoot problem was dominated by the effective acceleration being too low when callback rate was lower than the configured timer rate.

## Simulation Robot Integration

Effective source:

- `src/pb_rm_simulation/src/rm_nav_bringup/urdf/sentry_robot_sim.xacro`
- `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/urdf/simulation_waking_robot.xacro`

High-value setting:

| Parameter | Current Value | Why It Matters |
|---|---:|---|
| mecanum plugin `publish_rate` | `50` | improves odom/control responsiveness |

Changing xacro/plugin settings requires a full Gazebo restart.
