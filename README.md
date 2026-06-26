# RM Sentry Simulation Workspace

这是 RoboMaster 哨兵导航栈的 ROS 2 Humble 仿真工作区。当前目标是在 Gazebo RM3V3 场地中验证一条简洁、可解释、可调试的导航链：FAST-LIO2 感知输入、height-gated 2.5D 可通行性建图、JPS + MINCO 规划，以及 `traj_tracker` 轨迹跟踪。

当前系统事实以 `docs/current/system_overview.md` 为准；README 只作为仓库入口，不维护完整参数表或调试流程。

## 当前 Pipeline

```text
Stage 1: Gazebo + MID360 simulation + FAST-LIO2
  -> /cloud_registered + /Odometry + /odom

Stage 2: relocalization + traversability map + ESDF visualization
  -> /relocalization/status
  -> /perception/costmap_2d
  -> /perception/esdf_2d

Stage 3: costmap inflation + JPS + MINCO + traj_tracker
  -> /planner/path
  -> /planner/traj_samples
  -> /cmd_vel_chassis
```

Stage 3 当前故意使用 Gazebo `/odom` 作为仿真位姿源，避免 FAST-LIO 仿真漂移影响规划/控制测试。详细原因和模块边界见 `docs/current/system_overview.md`。

## 仓库结构

```text
AGENTS.md                 # coding agent 工作规则
docs/
  current/
    plan.md               # 当前任务、完成项、暂停项
    system_overview.md    # 当前系统事实入口
    bringup.md            # 编译、启动、停止、健康检查
  modules/                # active pipeline 模块理解笔记
  decisions/              # 重大方案取舍 ADR
src/
  pb_rm_simulation/       # 轻量 Polar Bear 仿真 vendor 子集
  sentry_mapping/         # traversability_mapper
  sentry_perception/      # relocalization, esdf2d
  sentry_planner/         # costmap_inflator, JPS, MINCO
  sentry_controller/      # traj_tracker, keyboard_teleop
```

`build/`、`install/`、`log/`、rosbag、debug CSV 和生成的 frame graph 不应作为长期项目知识提交。

## 必读文档

重新进入项目时建议按这个顺序读：

1. `AGENTS.md`
2. `docs/current/system_overview.md`
3. `docs/current/plan.md`
4. 如果要运行仿真，读 `docs/current/bringup.md`
5. 如果要改某个模块，读对应 `docs/modules/*.md`
6. 如果要理解方案取舍，读 `docs/decisions/*.md`

## Build

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

常用 targeted build：

```bash
colcon build --symlink-install --packages-select sentry_mapping
colcon build --symlink-install --packages-select sentry_perception
colcon build --symlink-install --packages-select sentry_planner
colcon build --symlink-install --packages-select sentry_controller
colcon build --symlink-install --packages-select rm_nav_bringup pb_rm_simulation
```

## Run

完整启动、后台启动、停止、健康检查和测试目标发布命令见：

```text
docs/current/bringup.md
```

三终端最短启动链：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

DISPLAY=:1 ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RM3V3 lio:=fastlio mode:=mapping lio_rviz:=False nav_rviz:=False

DISPLAY=:1 ros2 launch rm_nav_bringup sim_perception.launch.py \
  perception_rviz:=false

ros2 launch sentry_planner sim_planner.launch.py
```

RViz fixed frame 应为 `map`。

## 当前非默认或已废弃方向

- Nav2 upstream 风格 pipeline 不是当前主链。
- 3D ROG runtime 不是当前主链。
- `pure_pursuit`、`path_tracker`、`goal_controller` 和固定 `traj_publisher` 不属于当前 active controller 路径。
- direct MPC 是未来选项，当前 active controller 是 `traj_tracker`。

相关原因见 `docs/decisions/`。

## 维护规则

- 当前系统事实只维护在 `docs/current/system_overview.md`，不要在 README 中复制完整 topic/frame/参数表。
- 当前任务只维护在 `docs/current/plan.md`。
- 启动和健康检查只维护在 `docs/current/bringup.md`。
- 模块内部机制和参数含义维护在 `docs/modules/*.md`。
- 重大方案切换写入 `docs/decisions/*.md`。
- 不要向 launch/config/source 文件写入机器相关绝对路径。
