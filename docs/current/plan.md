# Plan

本文档记录 `rm_sentry_sim_ws` 的当前推进状态。它不是架构说明；当前系统事实见 `docs/current/system_overview.md`。

## Project goal

构建并验证一个仿真优先的 RoboMaster 哨兵导航栈：Gazebo 中可稳定完成定位、可通行性建图、JPS + MINCO 规划和轨迹跟踪，并为后续真车迁移保留清晰模块边界。

## Current focus

当前重点是路径控制与动力学一致性：让 JPS 拓扑路径、MINCO 时间参数化轨迹、`traj_tracker` 控制限制和 Gazebo 底盘响应保持一致，减少高速急弯、贴墙和 fallback 场景中的过冲风险。

## Current task queue

### P0

- [ ] 标定 Gazebo 底盘在当前 mecanum 插件和 `/cmd_vel_chassis` 下的实际速度、加速度和制动响应。
- [ ] 对齐 MINCO 的 `v_max/a_max/t_min`、`traj_tracker` 的 `v_max/acc_lim` 和 Gazebo 实际响应能力。
- [ ] 建立 sharp-turn 场景的最小验证流程，至少记录 `/tmp/planner.log`、`/tmp/traj_tracker_debug.csv`、`/odom` hz 和是否碰撞。

### P1

- [ ] 逐步把模块文档改成真正的代码理解笔记，而不是 topic/参数清单。
- [ ] 为 JPS、MINCO、`traj_tracker` 的局部检查补充可复用命令和通过标准。
- [ ] 评估 `esdf_grad_viz` 是否继续作为调试工具保留，还是只在需要调距时临时启动。

### P2

- [ ] 如果时间参数化跟踪仍不能满足急弯性能，再讨论真正 tracking MPC。
- [ ] 为 sim-to-real 整理参数迁移表，但不提前把真车约束写进当前仿真 pipeline。

## Recent progress log

- 2026-06-25：文档结构从大目录体系收缩为 `current/`、`modules/`、`decisions/` 三类长期知识。
- 2026-06-25：当前 Stage 3 确认为 `costmap_inflator -> jps_node -> minco_planner_node -> traj_tracker`。
- 2026-06-25：确认当前控制器是 `traj_tracker.py`。

## Done / deprecated / paused

- [x] Stage 1/2/3 分阶段启动链已建立。启动方法见 `docs/current/bringup.md`。
- [x] 当前仿真建图从 3D ROG runtime 切换为 height-gated 2.5D traversability map。原因见 `docs/decisions/ADR-002-drop-3d-rog.md`。
- [x] 当前规划从 Nav2 upstream 风格收缩到 JPS + MINCO。原因见 `docs/decisions/ADR-001-nav2-to-jps-minco.md`。
- [x] 静态场地先验由 Gazebo world 文件进入当前建图链。原因见 `docs/decisions/ADR-003-static-prior-map.md`。
- [x] 当前控制使用时间参数化 `traj_tracker`，旧 `path_tracker`、`goal_controller`、`pure_pursuit` 和固定 `traj_publisher` 不属于 active pipeline。原因见 `docs/decisions/ADR-004-traj-tracker.md`。
- [ ] 直接 MPC 控制器暂停；只有在当前轨迹时序和底盘响应问题被证据化后再重新评估。

## Pointers

- 当前系统事实：`docs/current/system_overview.md`
- 启动和健康检查：`docs/current/bringup.md`
- 模块理解：`docs/modules/*.md`
- 方案原因：`docs/decisions/*.md`
