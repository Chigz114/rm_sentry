# rm_sentry_sim_ws File Inventory

Last updated: 2026-06-23

This document summarizes the workspace structure after the height-gated traversability, JPS/MINCO planning, and `traj_tracker` control work. It is intentionally focused on project-owned files and modified simulation integration points; third-party/upstream packages are grouped instead of listed exhaustively.

## Top-Level Layout

| Path | Role | Notes |
|------|------|-------|
| `docs/` | Project documentation and operations notes | Main design history lives here. |
| `scripts/` | One-off diagnostics | Costmap, density, hit, and z-histogram checks. |
| `src/sentry_planner/` | Project-owned planning package | Costmap inflation, JPS, ESDF, MINCO, stage3 launch. |
| `src/sentry_controller/` | Project-owned control package | Current `traj_tracker` plus older controller baselines/tools. |
| `src/sentry_mapping/` | Project-owned mapping package | Local package directory; no longer an absolute symlink to `real_ws`. |
| `src/sentry_perception/` | Project-owned perception package | Local package directory; no longer an absolute symlink to `real_ws`. |
| `src/pb_rm_simulation/` | Lightweight Polar Bear simulation vendor subset | Gazebo RM3V3 world, robot xacro, FAST-LIO, Livox sim, and IMU filter. |
| `build/`, `install/`, `log/` | Colcon outputs | Generated artifacts; ignored by `.gitignore`. |

## Documentation

| File | Purpose | Keep? |
|------|---------|-------|
| `docs/height_gated_traversability_plan.md` | Main design log and decision record for perception, planning, MINCO, and control. | Keep. |
| `docs/runbooks/bringup.md` | Short start/stop/build/health-check commands for the current simulation chain. | Keep; replaces the older `ops_reference.md`. |
| `docs/runbooks/debug_planning.md` | Operational guide for JPS/MINCO/fallback/wall-risk debugging. | Keep. |
| `docs/runbooks/debug_control.md` | Operational guide for overshoot/speed/chassis response debugging. | Keep. |
| `docs/validation/planning_validation.md` | Pass/fail checks for planning changes. | Keep. |
| `docs/validation/control_validation.md` | Pass/fail checks for control changes. | Keep. |
| `docs/file_inventory.md` | This inventory. | Keep. |
| `SIM_WORKFLOW.md` | Earlier simulation workflow notes. | Deleted after replacement by `runbooks/bringup.md` and current docs. |
| `notes/progress.md` | Older progress notes mentioning ROG/pure-pursuit tests. | Deleted. |

## Planning Package: `src/sentry_planner`

| File | Runtime Role | Notes |
|------|--------------|-------|
| `launch/sim_planner.launch.py` | Current stage3 launch entry. Starts `costmap_inflator`, `jps_node`, `minco_planner_node`, and `traj_tracker`. | Primary launch file for planning/control. |
| `sentry_planner/costmap_inflator.py` | Inflates `/perception/costmap_2d` into `/planner/costmap_inflated`. | Current launch uses `inflation_radius_m=0.30`; do not change casually. |
| `sentry_planner/jps_node.py` | Binary-grid JPS planner. Publishes `/planner/path`. | Current JPS has no clearance cost; it only sees free/occupied cells. |
| `sentry_planner/path_postprocess.py` | Prunes and spaces JPS waypoints; allocates initial segment durations. | Feeds MINCO. |
| `sentry_planner/esdf_map_2d.py` | ESDF distance lookup wrapper. | Used by MINCO clearance costs. |
| `sentry_planner/esdf_grad_viz.py` | ESDF visualization/debug node. | Not launched by default; useful for clearance tuning. |
| `sentry_planner/minco_solver_2d.py` | MINCO polynomial solver and cost evaluation. | Includes smoothness, time, ESDF clearance, dynamics, and JPS soft reference cost. |
| `sentry_planner/minco_planner_node.py` | ROS node wrapping postprocess + MINCO. Publishes `/planner/path_vis`, `/planner/traj_samples`, markers, and info. | Current control-facing trajectory source. |
| `setup.py` | Console-script registration. | Registers all planner executables. |

Current `sim_planner.launch.py` planning parameters of interest:

| Parameter | Current Value | Meaning |
|-----------|---------------|---------|
| `inflation_radius_m` | `0.30` | JPS binary obstacle inflation. |
| `v_max` | `6.0` | MINCO velocity soft limit. |
| `a_max` | `16.0` | MINCO acceleration soft limit. |
| `v_alloc` | `3.0` | Initial time allocation speed. |
| `t_min` | `1.0` | Minimum segment duration; returned from failed `0.5` test. |
| `w_obs` | `3000.0` | ESDF soft clearance cost. |
| `w_collision` | `10000.0` | ESDF hard clearance cost. |
| `d_soft` | `0.50` | Preferred clearance onset. |
| `d_hard` | `0.25` | Final hard clearance threshold. |
| `w_ref` | `20.0` | Soft pull toward original JPS waypoints. |
| `waypoint_bound_m` | `1.00` | Local movement bound around original JPS waypoints. |

## Controller Package: `src/sentry_controller`

| File | Runtime Role | Notes |
|------|--------------|-------|
| `sentry_controller/traj_tracker.py` | Current main controller. Tracks `/planner/traj_samples`, uses real `control_dt`, publishes `/cmd_vel_chassis`. | Active in `sim_planner.launch.py`. |
| `sentry_controller/path_tracker.py` | Previous geometric/Frenet/path tracker baseline. | Not launched by current stage3; useful for regression comparison. |
| `sentry_controller/goal_controller.py` | Early goal-to-point controller. | Not launched by current stage3; useful as simple fallback/test tool. |
| `sentry_controller/keyboard_teleop.py` | Manual velocity command tool. | Useful for smoke testing Gazebo drive path. |
| `setup.py` | Console-script registration. | Still registers all controller tools/baselines. |

Current `traj_tracker` parameters of interest:

| Parameter | Current Value | Meaning |
|-----------|---------------|---------|
| `v_max` | `6.0` | Controller speed cap. |
| `acc_lim` | `12.0` | Controller acceleration cap using real `control_dt`. |
| `max_feedback_speed` | `2.0` | Cap on feedback correction velocity. |
| `offtrack_v_max` | `2.0` | Speed cap when tracking error is large. |
| `debug_csv_path` | `/tmp/traj_tracker_debug.csv` | Per-cycle tracking diagnostics. |

## Modified Simulation Integration Points

| File | Change | Notes |
|------|--------|-------|
| `src/pb_rm_simulation/src/rm_nav_bringup/urdf/sentry_robot_sim.xacro` | `mecanum_controller` odom `<publish_rate>` raised to `50`. | Requires full Gazebo restart to take effect. |
| `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/urdf/simulation_waking_robot.xacro` | Same `<publish_rate>` change. | Kept synchronized with bringup xacro. |
| `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` | Simulation stage1 entry. | Lightweight launch: Gazebo + IMU filter + FAST-LIO. |
| `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` | Simulation perception stage2 entry. | Starts relocalization, traversability, and ESDF. |

## Data and Diagnostics

| Path | Purpose | Keep? |
|------|---------|-------|
| `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/world/RM3V3/rm3v3_sym_v1.world` | Runtime RM3V3 simulation world. | Keep; resolved through `pb_rm_simulation` package share. |
| `src/pb_rm_simulation/src/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv` | Livox MID360 scan pattern. | Keep; required by the Gazebo Livox plugin. |
| `data/bags/` | Optional local rosbag recordings. | Directory is absent by default and ignored by `.gitignore`; track deliberately only if needed. |
| `scripts/diag_costmap.py` | Detailed costmap diagnostics. | Keep or move to `tools/diagnostics/`. |
| `scripts/diag_costmap_quick.py` | Quick costmap diagnostics. | Keep or move to `tools/diagnostics/`. |
| `scripts/diag_density.py` | Point density diagnostics. | Keep or move to `tools/diagnostics/`. |
| `scripts/diag_hits.py` | Hit-count diagnostics. | Keep or move to `tools/diagnostics/`. |
| `scripts/diag_zhist.py` | Height histogram diagnostics. | Keep or move to `tools/diagnostics/`. |

## Entry Dependency Audit

Static search was run excluding `build/`, `install/`, `log/`, and `__pycache__/`.

### Current Stage3 Runtime Entrypoints

These are directly launched by `src/sentry_planner/launch/sim_planner.launch.py`:

| Executable | Package | Status |
|------------|---------|--------|
| `costmap_inflator` | `sentry_planner` | Active. |
| `jps_node` | `sentry_planner` | Active. |
| `minco_planner_node` | `sentry_planner` | Active. |
| `traj_tracker` | `sentry_controller` | Active current controller. |

### Registered But Not Launched By Current Stage3

These remain in `setup.py` console scripts but are not used by current `sim_planner.launch.py`:

| Executable | File | Recommendation |
|------------|------|----------------|
| `path_tracker` | `sentry_controller/path_tracker.py` | Keep as baseline/regression tool. |
| `goal_controller` | `sentry_controller/goal_controller.py` | Keep as simple fallback for now. |
| `keyboard_teleop` | `sentry_controller/keyboard_teleop.py` | Keep for manual smoke tests. |
| `esdf_grad_viz` | `sentry_planner/esdf_grad_viz.py` | Keep for ESDF/MINCO clearance tuning. |

### Stale Documentation References

`docs/runbooks/bringup.md` is the current concise start/stop reference. Historical sections in `height_gated_traversability_plan.md` may still mention `path_tracker` or `pure_pursuit` as past-stage baselines.

## Cleanup Recommendations

Safe generated artifacts to delete when a clean workspace is desired:

- `build/`
- `install/`
- `log/`
- `src/**/__pycache__/`
- `install/**/__pycache__/`
- `*.pyc`

Archive candidates, not immediate deletion:

- old `data/bags/stage0_*` — deleted.
- older narrative docs once `runbooks/bringup.md` and this inventory are updated — `SIM_WORKFLOW.md` and `notes/progress.md` deleted.

Do not delete yet:

- `path_tracker.py` because it is a useful baseline against `traj_tracker`.
- `goal_controller.py` because it is a simple fallback/debug controller.
- `esdf_grad_viz.py` because MINCO clearance tuning is still active.
- either modified xacro, because both can be reached by different simulation launches.
