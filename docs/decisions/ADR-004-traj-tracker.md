# ADR-004: 使用时间参数化 traj_tracker 替代旧几何跟踪器

## Status

accepted

## Context

MINCO 输出的是带时间、速度和加速度信息的轨迹。旧的几何或纯路径跟踪方式不充分利用这些信息，容易把轨迹生成和控制执行割裂。

## Old approach

旧控制路径包括 `pure_pursuit`、`path_tracker`、`goal_controller` 和固定测试 `traj_publisher` 等基线或调试节点。

## Problem with old approach

- 几何跟踪器主要消费路径形状，不消费 MINCO 的时间参数化结果。
- 固定轨迹或简单 goal controller 只能做局部冒烟测试，不能代表当前规划输出。
- 过冲问题同时涉及轨迹时序、控制器加速度限制和 Gazebo 响应，旧结构难以归因。

## Options considered

- 继续使用旧几何跟踪器。
- 直接实现 tracking MPC。
- 先使用时间参数化 `traj_tracker`，稳定接口和动力学一致性，再决定是否需要 MPC。

## Decision

当前 Stage 3 使用 `traj_tracker`。它订阅 `/planner/traj_samples` 和 `/odom`，发布 `/cmd_vel_chassis`。

`/planner/traj_samples` 的每行格式为：

```text
[t, x, y, vx, vy, ax, ay, yaw]
```

## Why this decision

`traj_tracker` 让控制器直接跟踪 MINCO 时间参数化轨迹，并显式处理 `control_dt`、速度限制、加速度限制和调试 CSV。它比直接跳到 MPC 更轻，且能先暴露轨迹和底盘能力是否一致。

## Consequences

- 直接 MPC 当前不属于 active pipeline。
- 过冲调试应先看 `/tmp/traj_tracker_debug.csv` 中的 `control_dt`、误差、命令和饱和情况。
- 未来 MPC 应替换控制律，而不是破坏 `/planner/traj_samples` 这个已经建立的轨迹接口。

## Revisit condition

如果 `traj_tracker` 在已对齐 MINCO/Gazebo 动力学约束后仍无法满足急弯、贴墙或重规划瞬间的性能，再考虑 tracking MPC。

## Related modules

- `docs/modules/minco.md`
- `docs/modules/traj_tracker.md`
- `docs/modules/gazebo_chassis.md`

## Related plan entries

- `docs/current/plan.md`
