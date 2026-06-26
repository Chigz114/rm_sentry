# System Overview

本文档是 `rm_sentry_sim_ws` 当前系统事实的唯一入口。任务推进看 `docs/current/plan.md`，启动命令看 `docs/current/bringup.md`。

## Active pipeline

```text
Stage 1: Gazebo + MID360 simulation + FAST-LIO2
  /livox/lidar + /livox/imu
  -> imu_complementary_filter + fast_lio
  -> /cloud_registered + /Odometry + /odom

Stage 2: relocalization + traversability map + ESDF visualization
  /cloud_registered
  -> relocalization_node
  -> TF map -> lidar_odom

  /cloud_registered + /Odometry + TF + Gazebo world prior
  -> traversability_mapper
  -> /perception/costmap_2d
  -> esdf2d_node
  -> /perception/esdf_2d

Stage 3: costmap inflation + JPS + MINCO + trajectory tracking
  /perception/costmap_2d
  -> costmap_inflator
  -> /planner/costmap_inflated
  -> jps_node
  -> /planner/path
  -> minco_planner_node
  -> /planner/traj_samples
  -> traj_tracker
  -> /cmd_vel_chassis
```

## Stage layout

| Stage | 做什么 | launch 入口 | 关键输出 |
|---|---|---|---|
| Stage 1 | 启动 Gazebo、仿真 MID360/IMU、IMU 滤波和 FAST-LIO2 | `rm_nav_bringup/launch/bringup_sim.launch.py` | `/cloud_registered`, `/Odometry`, `/odom` |
| Stage 2 | 发布 `map -> lidar_odom`，生成 height-gated 2.5D costmap 和 ESDF 可视化 | `rm_nav_bringup/launch/sim_perception.launch.py` | `/relocalization/status`, `/perception/costmap_2d`, `/perception/esdf_2d` |
| Stage 3 | 膨胀 costmap、搜索路径、生成 MINCO 轨迹并跟踪 | `sentry_planner/launch/sim_planner.launch.py` | `/planner/path`, `/planner/traj_samples`, `/cmd_vel_chassis` |

## Active modules

| Function | Active implementation | Main file | Module doc |
|---|---|---|---|
| Gazebo + FAST-LIO2 | `bringup_sim.launch.py`, `fastlio_mapping` | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` | `docs/modules/fastlio2.md` |
| Relocalization | `relocalization_node` | `src/sentry_perception/sentry_perception/relocalization_node.py` | `docs/modules/relocalization.md` |
| Traversability map | `traversability_mapper` | `src/sentry_mapping/src/traversability_mapper_node.cpp` | `docs/modules/traversability_map.md` |
| 2D ESDF visualization | `esdf2d_node` | `src/sentry_perception/sentry_perception/esdf2d_node.py` | `docs/modules/esdf2d.md` |
| Costmap inflation | `costmap_inflator` | `src/sentry_planner/sentry_planner/costmap_inflator.py` | `docs/modules/costmap_inflation.md` |
| JPS search | `jps_node` | `src/sentry_planner/sentry_planner/jps_node.py` | `docs/modules/jps.md` |
| MINCO trajectory | `minco_planner_node`, `minco_solver_2d.py` | `src/sentry_planner/sentry_planner/minco_planner_node.py` | `docs/modules/minco.md` |
| Trajectory tracking | `traj_tracker` | `src/sentry_controller/sentry_controller/traj_tracker.py` | `docs/modules/traj_tracker.md` |
| Gazebo chassis | `mecanum_controller` plugin | `src/pb_rm_simulation/src/rm_nav_bringup/urdf/sentry_robot_sim.xacro` | `docs/modules/gazebo_chassis.md` |

## Core topics and frames

| Topic / frame | Role |
|---|---|
| `/cloud_registered` | FAST-LIO2 配准点云，供重定位和可通行性建图使用 |
| `/Odometry` | FAST-LIO2 odometry，供可通行性建图使用 |
| `/odom` | Gazebo/底盘 odometry，当前 Stage 3 的仿真位姿源 |
| `/relocalization/status` | 重定位状态和 ICP/seed 健康信号 |
| `/perception/costmap_2d` | 当前规划 costmap，`frame_id=map` |
| `/perception/esdf_2d` | ESDF 可视化点云；MINCO 当前主要使用内部 `EsdfMap2D` |
| `/planner/costmap_inflated` | JPS 输入栅格 |
| `/goal_pose` | RViz 或 CLI 发布的目标 |
| `/planner/path` | JPS 输出路径 |
| `/planner/traj_samples` | MINCO 输出的时间参数化轨迹，行格式为 `[t,x,y,vx,vy,ax,ay,yaw]` |
| `/cmd_vel_chassis` | 发给 Gazebo mecanum plugin 的底盘速度命令 |
| `map -> lidar_odom -> base_link` | 当前感知侧概念 TF 链 |

## Runtime assumptions

- Stage 3 当前故意使用 `/odom` 作为 JPS、MINCO 和 `traj_tracker` 的仿真位姿源，避免 FAST-LIO 仿真漂移污染控制测试。
- Stage 2 的 `/perception/costmap_2d` 应在 `map` frame 中，且需要与 Gazebo RM3V3 世界对齐。
- `traversability_mapper` 会读取 Gazebo world 文件生成静态先验，并与动态点云证据合并。
- JPS 只负责拓扑可行性；贴墙风险不应只靠增大 `inflation_radius_m` 解决。
- MINCO、`traj_tracker` 和 Gazebo 底盘实际能力必须保持动力学一致，否则过冲调参会失去归因。
- Deprecated pipelines 不应默认恢复。

## Deprecated summary

| Component | Status | Replacement / note |
|---|---|---|
| Nav2 upstream pipeline | deprecated | 当前使用 JPS + MINCO，见 `docs/decisions/ADR-001-nav2-to-jps-minco.md` |
| 3D ROG runtime | deprecated | 当前使用 height-gated 2.5D traversability map，见 `docs/decisions/ADR-002-drop-3d-rog.md` |
| old static/dynamic split without world prior | deprecated | 当前 world prior 并入 traversability map，见 `docs/decisions/ADR-003-static-prior-map.md` |
| `pure_pursuit`, `path_tracker`, `goal_controller`, fixed `traj_publisher` | removed / non-default | 当前使用 `traj_tracker`，见 `docs/decisions/ADR-004-traj-tracker.md` |
| direct MPC controller | future option | 当前不属于 active pipeline |

## Simulation integration points

- `bringup_sim.launch.py`：轻量 Stage 1，只启动 Gazebo、IMU 滤波、FAST-LIO2 和可选 FAST-LIO RViz。
- `sim_perception.launch.py`：Stage 2，启动 `relocalization_node`、`traversability_mapper`、`esdf2d_node`。
- `sim_planner.launch.py`：Stage 3，启动 `costmap_inflator`、`jps_node`、`minco_planner_node`、`traj_tracker`。
- `sentry_robot_sim.xacro` 和 `simulation_waking_robot.xacro`：mecanum plugin 订阅 `cmd_vel_chassis`、发布 `/odom`，当前 `publish_rate=50`。

## Where to read next

- 当前任务：`docs/current/plan.md`
- 启动/停止/健康检查：`docs/current/bringup.md`
- 模块理解：`docs/modules/*.md`
- 方案原因：`docs/decisions/*.md`
- Agent 工作规则：`AGENTS.md`
