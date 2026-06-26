# Bringup

本文档是当前 `rm_sentry_sim_ws` 仿真链的启动、停止和健康检查手册。系统结构见 `docs/current/system_overview.md`。

## Purpose

启动三阶段仿真链：

1. Stage 1: Gazebo + FAST-LIO2。
2. Stage 2: relocalization + traversability map + ESDF。
3. Stage 3: costmap inflation + JPS + MINCO + `traj_tracker`。

## Environment

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export DISPLAY=:1
```

`DISPLAY=:1` 是当前工作环境的本地显示假设。如在其它机器运行，按实际显示环境调整。

## Build

全量 build：

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws
conda deactivate 2>/dev/null || true
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

修改 C++、Python entry point、launch、xacro 或 package 安装资源后，重新 source：

```bash
source install/setup.bash
```

## Launch: staged mode

### Terminal 1: Stage 1

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
DISPLAY=:1 ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RM3V3 \
  lio:=fastlio \
  mode:=mapping \
  lio_rviz:=False \
  nav_rviz:=False
```

等待 `/cloud_registered`、`/Odometry` 和 `/odom` 开始发布。

### Terminal 2: Stage 2

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
DISPLAY=:1 ros2 launch rm_nav_bringup sim_perception.launch.py \
  perception_rviz:=false
```

确认重定位状态：

```bash
ros2 topic echo /relocalization/status --once
```

### Terminal 3: Stage 3

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sentry_planner sim_planner.launch.py
```

### Optional RViz

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
DISPLAY=:1 rviz2 -d $(ros2 pkg prefix rm_nav_bringup)/share/rm_nav_bringup/rviz/sim_perception.rviz
```

RViz fixed frame 应为 `map`。

## Launch: background mode

当前仍可用后台命令快速启动。长期更理想的形态是拆为 `scripts/bringup_sim.sh`、`scripts/stop_sim.sh`、`scripts/check_topics.sh`。

```bash
cd /home/arch/robomaster/rm_sentry_sim_ws

ps -C gzserver,gzclient,relocalization_node,traversability_mapper,esdf2d_node,laser_mapping,robot_state_publisher,costmap_inflator,jps_node,minco_planner_node,traj_tracker,rviz2 --no-headers -o pid 2>/dev/null | xargs -r kill -9
ps aux | grep -E 'relocalization|esdf2d|traversability_mapper|bringup_sim|sim_perception|sim_planner|rm_simulation|gzserver|gzclient|laser_mapping|robot_state_publisher|costmap_inflator|jps_node|minco_planner|traj_tracker|rviz2' | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2

nohup setsid bash -c 'export DISPLAY=:1 && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RM3V3 lio:=fastlio mode:=mapping lio_rviz:=False nav_rviz:=False' > /tmp/bringup.log 2>&1 < /dev/null & disown
sleep 12

nohup setsid bash -c 'export DISPLAY=:1 && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rm_nav_bringup sim_perception.launch.py perception_rviz:=false' > /tmp/perception.log 2>&1 < /dev/null & disown
sleep 4

nohup setsid bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch sentry_planner sim_planner.launch.py' > /tmp/planner.log 2>&1 < /dev/null & disown
```

## Stop

不要使用 `pkill -f`；它可能匹配自身命令并挂起。

停止仿真、感知、规划和 RViz：

```bash
ps -C gzserver,gzclient,relocalization_node,traversability_mapper,esdf2d_node,laser_mapping,robot_state_publisher,costmap_inflator,jps_node,minco_planner_node,traj_tracker,rviz2 --no-headers -o pid 2>/dev/null | xargs -r kill -9
ps aux | grep -E 'relocalization|esdf2d|traversability_mapper|bringup_sim|sim_perception|sim_planner|rm_simulation|gzserver|gzclient|laser_mapping|robot_state_publisher|costmap_inflator|jps_node|minco_planner|traj_tracker|rviz2' | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2
ps aux | grep -E 'relocalization|esdf2d|traversability|gzserver|gzclient|minco|jps_node|traj_tracker' | grep -v grep | wc -l
```

预期最后输出为 `0`。

## Health check

```bash
source /opt/ros/humble/setup.bash
source /home/arch/robomaster/rm_sentry_sim_ws/install/setup.bash

ros2 node list | grep -E "relocalization|traversability|esdf|inflator|jps|minco|tracker"
ros2 topic echo /relocalization/status --once
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic hz /odom
ros2 topic hz /perception/costmap_2d
ros2 topic hz /perception/esdf_2d
ros2 topic hz /planner/costmap_inflated
```

预期活跃节点：

```text
relocalization_node
traversability_mapper
esdf2d_node
costmap_inflator
jps_node
minco_planner_node
traj_tracker
```

## Send test goal

RViz：在 `map` fixed frame 下使用 `2D Goal Pose`。

CLI：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 9.0, y: 6.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## When full Gazebo restart is required

以下修改需要完全重启 Stage 1：

- 机器人 xacro；
- Gazebo world 文件；
- mecanum / chassis plugin 设置；
- lidar / IMU 仿真设置；
- 任何改变 `/odom`、`/livox/lidar` 或 `/livox/imu` 发布者的内容。

如果源码或安装资源改变，重启前先重新 build 并 source。

## Logs

| 文件 | 来源 |
|---|---|
| `/tmp/bringup.log` | Stage 1 后台启动日志 |
| `/tmp/perception.log` | Stage 2 后台启动日志 |
| `/tmp/planner.log` | Stage 3 后台启动日志 |
| `/tmp/rviz.log` | RViz 后台日志，如果手动启动 |
| `/tmp/traj_tracker_debug.csv` | `traj_tracker` 每周期诊断 |
