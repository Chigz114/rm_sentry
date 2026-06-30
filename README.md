# RM Sentry Simulation Workspace

这是 RoboMaster 哨兵导航栈的 ROS 2 Humble 仿真工作区，用于在 Gazebo RM3V3 场地中验证一条仿真优先的导航链。

当前系统围绕三段流程展开：

```text
Gazebo + MID360 simulation + FAST-LIO2
  -> relocalization + height-gated traversability map
  -> costmap inflation + JPS + MINCO
  -> traj_tracker
  -> /cmd_vel_chassis
```

项目重点不是维护多套导航方案，而是把感知、建图、规划、轨迹生成和底盘执行之间的接口做清楚，并持续验证路径控制与动力学约束是否一致。

代码主要分布在：

```text
src/pb_rm_simulation/   Gazebo 仿真、RM3V3 世界、MID360/IMU、FAST-LIO2 相关集成
src/sentry_mapping/    height-gated 2.5D 可通行性建图
src/sentry_perception/ relocalization 与 2D ESDF 可视化
src/sentry_planner/    costmap inflation、JPS、MINCO
src/sentry_controller/ traj_tracker 与手动 teleop
```

详细系统事实、启动方法、模块理解和方案取舍记录放在 `docs/` 下。
