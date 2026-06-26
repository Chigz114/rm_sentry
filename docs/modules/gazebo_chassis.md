# Gazebo Chassis

## Code map

| Part | Location | Role |
|---|---|---|
| Stage 1 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` | include Gazebo simulation launch |
| Active robot xacro | `src/pb_rm_simulation/src/rm_nav_bringup/urdf/sentry_robot_sim.xacro` | 当前 bringup 使用的 robot description |
| Vendor robot xacro | `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/urdf/simulation_waking_robot.xacro` | 仿真包内对应机器人描述 |
| Chassis plugin | `libgazebo_ros_planar_move.so` / `mecanum_controller` | 订阅速度命令并发布 odom |

## Module role

Gazebo chassis 是控制闭环的 plant。它把 `/cmd_vel_chassis` 转为仿真机器人运动，并发布 `/odom`。规划/控制看似在 Stage 3，但最终能否不过冲取决于这个模块的实际响应。

## Interface contract

Input:

- `/cmd_vel_chassis`：`geometry_msgs/Twist`，来自 `traj_tracker` 或手动 teleop。

Output:

- `/odom`：`nav_msgs/Odometry`，当前 Stage 3 的位姿源。
- `odom -> base_link` TF：在当前 `sentry_robot_sim.xacro` 中 `publish_odom_tf=true`。

## Internal mechanism

1. `bringup_sim.launch.py` 读取 `measurement_params_sim.yaml` 并用 xacro 生成 robot description。
2. Gazebo 启动机器人模型和 mecanum planar move plugin。
3. plugin remap `cmd_vel:=cmd_vel_chassis`，订阅 `/cmd_vel_chassis`。
4. plugin remap `odom:=odom`，发布 `/odom`。
5. `publish_rate=50` 控制 odom 发布频率。
6. Stage 3 的 JPS、MINCO 和 `traj_tracker` 都使用 `/odom` 作为当前仿真位姿源。

## Parameters in computation

| Parameter | Meaning in xacro/plugin | Effect |
|---|---|---|
| `publish_rate` | odom 发布频率，当前为 `50` | 过低会让控制器看到滞后的 plant 状态 |
| `publish_odom` | 是否发布 odom | 关闭会破坏 Stage 3 |
| `publish_odom_tf` | 是否发布 odom TF | 当前 active xacro 为 `true` |
| `odometry_frame` | odom frame 名称 | 当前为 `odom` |
| `robot_base_frame` | base frame 名称 | 当前为 `base_link` |
| remap `cmd_vel:=cmd_vel_chassis` | 命令输入重映射 | 必须与 `traj_tracker` 输出一致 |

## Coupling

Upstream:
`traj_tracker` 输出速度和加速度变化；如果命令超过 Gazebo 实际响应能力，会表现为过冲或滞后。

Downstream:
`/odom` 被 JPS、MINCO 和 tracker 使用。odom 发布率或 frame 问题会污染整个 Stage 3。

## Important implementation details

- 修改 xacro、world、plugin 或 lidar/IMU 仿真设置后必须完全重启 Gazebo。
- 当前有两个相近 xacro 文件；需要确认 active launch 使用的是 `rm_nav_bringup/urdf/sentry_robot_sim.xacro`。
- `simulation_waking_robot.xacro` 中 `publish_odom_tf=false`，不要误把它当作当前 active xacro 的全部行为。

## Local checks

```bash
ros2 topic hz /odom
ros2 topic hz /cmd_vel_chassis
ros2 topic echo /odom --once
```

手动冒烟测试：

```bash
ros2 run sentry_controller keyboard_teleop
```

## Personal understanding / open questions

- 当前 plugin 是否有隐藏加速度限制或低通行为？
- `publish_rate=50` 是否足以支撑 tracker 的 `rate_hz=30` 和急弯控制？
