# Traj Tracker

## Code map

| Part | Location | Role |
|---|---|---|
| Launch source | `src/sentry_planner/launch/sim_planner.launch.py` | 启动 `traj_tracker` 并覆盖控制参数 |
| Node file | `src/sentry_controller/sentry_controller/traj_tracker.py` | 时间参数化轨迹跟踪器 |
| Trajectory callback | `_on_traj()` | 解析 `/planner/traj_samples` |
| Odom callback | `_on_odom()` | 缓存当前 pose/velocity |
| Control loop | `_control_loop()` | 按 timer 计算速度命令 |
| Debug output | `debug_csv_path` | 写 `/tmp/traj_tracker_debug.csv` |

## Module role

`traj_tracker` 是当前 Stage 3 的 active controller。它跟踪 MINCO 发布的时间参数化轨迹，输出 `/cmd_vel_chassis` 给 Gazebo mecanum plugin。

直接 MPC 是未来选项，当前 active controller 是 `traj_tracker`。

## Interface contract

Input:

- `/planner/traj_samples`：`std_msgs/Float64MultiArray`，每行 `[t,x,y,vx,vy,ax,ay,yaw]`。
- `/odom`：`nav_msgs/Odometry`，当前仿真位姿源。

Output:

- `/cmd_vel_chassis`：`geometry_msgs/Twist`，body frame 速度命令。
- `/tmp/traj_tracker_debug.csv`：调试 CSV，如果 `debug_csv_path` 可打开。

## Internal mechanism

1. `_on_traj()` 检查数组长度，将 flat data 切成 8 列 rows，记录轨迹开始时间。
2. `_on_odom()` 更新当前 pose、yaw 和速度。
3. `_control_loop()` 在 timer 中运行；如果没有轨迹或 odom，不发有效跟踪命令。
4. 计算真实 `control_dt`：用当前 ROS time 与上一控制周期时间差，并夹在安全范围内。
5. 如果 `progress_anchor=True`，从当前附近轨迹点向前找 nearest index，再加 `lookahead_time` 得到参考时间。
6. `_sample_ref()` 对轨迹采样做线性插值，获得参考位置/速度/加速度。
7. 使用前馈速度加位置/速度反馈得到目标速度，并用 `max_feedback_speed` 限制反馈项。
8. 若偏离轨迹较大，使用 `track_slow_start/stop` 和 `offtrack_v_max` 限制速度。
9. 用 `acc_lim * control_dt` 对命令增量做加速度限幅。
10. 接近终点 `goal_tol` 时停止。
11. 写 debug CSV 并发布 `/cmd_vel_chassis`。

## Parameters in computation

| Parameter | Meaning in code | Effect |
|---|---|---|
| `traj_topic` | 轨迹输入 topic | 当前为 `/planner/traj_samples` |
| `odom_topic` | 位姿输入 topic | 当前为 `/odom` |
| `cmd_vel_topic` | 速度命令输出 | 当前为 `/cmd_vel_chassis` |
| `rate_hz` | timer 期望频率 | 不等于真实 `control_dt` |
| `v_max` | 输出速度上限 | 必须与 MINCO/Gazebo 能力一致 |
| `acc_lim` | 基于真实 `control_dt` 的加速度限幅 | 过低会刹不住，过高可能激进 |
| `kp_pos`, `kd_vel` | 位置反馈和速度阻尼 | 改变跟踪误差修正力度 |
| `lookahead_time` | 参考点前瞻时间 | 影响急弯提前量和稳定性 |
| `max_feedback_speed` | 反馈项速度上限 | 防止误差修正过猛 |
| `track_slow_start/stop` | 偏轨减速区间 | 偏离轨迹时限制速度 |
| `offtrack_v_max` | 偏轨速度上限 | 防止大误差高速追轨 |
| `debug_traj`, `debug_csv_path` | 调试输出 | 当前写 `/tmp/traj_tracker_debug.csv` |

## Coupling

Upstream:
MINCO 的 sample density、time scale、速度/加速度峰值直接决定 tracker 可执行性。

Downstream:
Gazebo mecanum plugin 对 `/cmd_vel_chassis` 的响应频率和实际加速度会影响过冲。`rate_hz` 只是 tracker timer 目标，真实执行要看 `control_dt` 和 `/odom` 更新。

## Important implementation details

- 加速度限幅必须使用真实 `control_dt`，不是简单 `1/rate_hz`。
- `w_max=0.0` 和 `k_yaw=0.0` 表示当前主要控制平面速度，不强调 yaw 跟踪。
- Debug CSV 是控制问题定位的一等证据。

## Local checks

```bash
ros2 topic echo /planner/traj_samples --once
ros2 topic hz /odom
ros2 topic hz /cmd_vel_chassis
tail -n 20 /tmp/traj_tracker_debug.csv
```

CSV 中重点看 `control_dt`、`nearest_d`、误差、参考速度、命令速度和饱和迹象。

## Personal understanding / open questions

- 当前 `acc_lim=12.0` 是否等于 Gazebo 实际可达到的加速度？
- `lookahead_time=0.25` 在 90 度急弯中是否足够提前？
