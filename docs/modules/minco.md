# MINCO

## Code map

| Part | Location | Role |
|---|---|---|
| Launch source | `src/sentry_planner/launch/sim_planner.launch.py` | 启动 `minco_planner_node` 并覆盖运行时参数 |
| ROS node | `src/sentry_planner/sentry_planner/minco_planner_node.py` | 接收 JPS path，发布轨迹和 marker |
| Solver | `src/sentry_planner/sentry_planner/minco_solver_2d.py` | 生成多项式、计算代价、优化路点 |
| ESDF helper | `src/sentry_planner/sentry_planner/esdf_map_2d.py` | 从 raw costmap 构建 signed ESDF |
| Path postprocess | `src/sentry_planner/sentry_planner/path_postprocess.py` | 修剪 JPS path、控制路点间距、分配初始时长 |

## Module role

MINCO 模块把 `/planner/path` 的离散 JPS 路点转为时间参数化轨迹 `/planner/traj_samples`。它负责平滑、间距代价、动力学约束检查和供控制器跟踪的采样格式。

## Interface contract

Input:

- `/planner/path`：`nav_msgs/Path`，JPS 输出路径。
- `/perception/costmap_2d`：`nav_msgs/OccupancyGrid`，用于内部 `EsdfMap2D`。
- `/odom`：`nav_msgs/Odometry`，当前 Stage 3 仿真位姿源。

Output:

- `/planner/path_vis`：`nav_msgs/Path`，稠密轨迹可视化。
- `/planner/traj_samples`：`std_msgs/Float64MultiArray`，控制器输入。
- `/planner/minco_traj`：`visualization_msgs/MarkerArray`。
- `/planner/minco_info`：`visualization_msgs/Marker`。

`/planner/traj_samples` 每行格式：

```text
[t, x, y, vx, vy, ax, ay, yaw]
```

## Internal mechanism

1. `on_costmap()` 用 raw costmap 更新内部 signed `EsdfMap2D`。
2. `on_odom()` 缓存当前速度和 frame。
3. `on_path()` 收到 JPS path 后提取 `(x, y)` 路点。
4. `postprocess_jps_path()` 做 line-of-sight prune、路点 spacing control，并按 `v_alloc/t_min` 分配初始段时长。
5. 节点从当前 odom 设置起点速度边界，并把 ESDF 传给 `MincoSolver2D`。
6. solver 用 L-BFGS 优化中间路点位置；durations 当前固定。
7. 代价包含 jerk smoothness、time pressure、ESDF clearance、dynamics penalty 和 JPS soft reference。
8. `check_clearance()` 如果发现 `min_d < d_hard`，会 fallback 到未优化 JPS 路点。
9. `check_dynamics()` 统计 `max_v/max_a` 和违规数量。
10. `sample_trajectory()` 以 `sample_dt` 采样并发布 `/planner/traj_samples`。

## Parameters in computation

| Parameter | Meaning in code | Effect |
|---|---|---|
| `v_max` | solver 速度软限制 | 与 tracker/Gazebo 能力不一致会导致跟踪困难 |
| `a_max` | solver 加速度软限制 | 过高会生成底盘难以执行的轨迹 |
| `v_alloc` | 初始时长分配速度 | 越大初始时长越短 |
| `w_smooth` | jerk 平滑代价权重 | 越大越偏向平滑 |
| `w_time` | 总时长代价权重 | 越大越压缩时间，可能提高动态压力 |
| `w_obs`, `w_collision` | ESDF 软/硬间距代价 | 控制远离障碍的强度 |
| `d_soft`, `d_hard` | 优选间距和硬检查阈值 | `d_hard` 直接参与 final check |
| `w_ref` | 对原始 JPS 路点的软参考 | 太小可能偏离拓扑，太大可能贴墙 |
| `waypoint_bound_m` | 中间路点优化边界 | 限制路点从 JPS 初值移动的范围 |
| `w_dyn` | 动力学限制惩罚 | 控制速度/加速度违规压力 |
| `min_spacing`, `max_spacing` | 路点后处理间距 | 影响段数、时长和轨迹形状 |
| `sample_dt` | 轨迹输出采样间隔 | 影响 tracker 输入分辨率 |
| `t_min` | 每段最小时长 | 太小可能让轨迹变快但不平滑或 final check 失败 |

## Coupling

Upstream:
JPS path 决定拓扑和初值；raw costmap 决定 ESDF 间距；`/odom` 决定起点速度边界。

Downstream:
`traj_tracker` 直接消费 `/planner/traj_samples`。MINCO 的速度/加速度假设必须与 tracker 的 `v_max/acc_lim` 和 Gazebo 真实响应匹配。

## Important implementation details

- 当前 launch 覆盖 `t_min=1.0`；代码默认值不是有效运行时值。
- 曾尝试降低 `t_min` 提速，但会增加 fallback 和贴墙风险；不要无验证地恢复。
- `/perception/esdf_2d` 点云不是 MINCO 的主输入；MINCO 使用 raw costmap 构建内部 `EsdfMap2D`。

## Local checks

```bash
ros2 topic echo /planner/path --once
ros2 topic echo /planner/traj_samples --once
tail -n 80 /tmp/planner.log
```

重点看日志中的 `MINCO final check`、`min_d`、`wp_shift`、`max_v`、`max_a`、`v_viol`、`a_viol` 和 fallback 消息。

## Personal understanding / open questions

- 当前 `a_max=16.0` 与 tracker `acc_lim=12.0`、Gazebo 实际加速度是否匹配？
- duration 当前固定是否限制了高速急弯性能？
