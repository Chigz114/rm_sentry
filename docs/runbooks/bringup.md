# Bringup Runbook

This is the short start/stop guide for the current `rm_sentry_sim_ws` simulation chain.

Use this file when you need simple commands. Use the debug and validation docs only after the stack is running.

## Environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export DISPLAY=:1
```

`DISPLAY=:1` is the expected local display in this environment.

## Build

```bash
cd <workspace>
conda deactivate 2>/dev/null
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Build only edited packages when possible:

```bash
colcon build --symlink-install --packages-select sentry_perception
colcon build --symlink-install --packages-select sentry_mapping
colcon build --symlink-install --packages-select sentry_planner
colcon build --symlink-install --packages-select sentry_controller
colcon build --symlink-install --packages-select rm_nav_bringup
colcon build --symlink-install --packages-select pb_rm_simulation
colcon build --symlink-install --packages-select livox_ros_driver2 ros2_livox_simulation imu_complementary_filter fast_lio
```

Source again after building:

```bash
source install/setup.bash
```

## Start: Three-Terminal Mode

### Terminal 1: Gazebo + FAST-LIO2

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
DISPLAY=:1 ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RM3V3 \
  lio:=fastlio \
  mode:=mapping \
  lio_rviz:=False \
  nav_rviz:=False
```

Wait until `/cloud_registered` and `/Odometry` are publishing.

### Terminal 2: Perception

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
DISPLAY=:1 ros2 launch rm_nav_bringup sim_perception.launch.py \
  perception_rviz:=false
```

Wait for relocalization status:

```bash
ros2 topic echo /relocalization/status --once
```

### Terminal 3: Planning + Control

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sentry_planner sim_planner.launch.py
```

### Optional RViz

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
DISPLAY=:1 rviz2 -d $(ros2 pkg prefix rm_nav_bringup)/share/rm_nav_bringup/rviz/sim_perception.rviz
```

RViz Fixed Frame should be `map`.

## Start: Background Mode

Use this for quick full-chain startup.

```bash
cd <workspace>

ps -C gzserver,gzclient,relocalization_node,traversability_mapper,esdf2d_node,laser_mapping,robot_state_publisher,costmap_inflator,jps_node,minco_planner_node,traj_tracker,path_tracker,rviz2 --no-headers -o pid 2>/dev/null | xargs -r kill -9
ps aux | grep -E 'relocalization|esdf2d|traversability_mapper|bringup_sim|sim_perception|sim_planner|rm_simulation|gzserver|gzclient|laser_mapping|robot_state_publisher|costmap_inflator|jps_node|minco_planner|traj_tracker|path_tracker|rviz2' | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2

nohup setsid bash -c 'export DISPLAY=:1 && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RM3V3 lio:=fastlio mode:=mapping lio_rviz:=False nav_rviz:=False' > /tmp/bringup.log 2>&1 < /dev/null & disown
sleep 12

nohup setsid bash -c 'export DISPLAY=:1 && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rm_nav_bringup sim_perception.launch.py perception_rviz:=false' > /tmp/perception.log 2>&1 < /dev/null & disown
sleep 4

nohup setsid bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch sentry_planner sim_planner.launch.py' > /tmp/planner.log 2>&1 < /dev/null & disown

DISPLAY=:1 nohup setsid bash -c 'export DISPLAY=:1 && source /opt/ros/humble/setup.bash && source install/setup.bash && rviz2 -d $(ros2 pkg prefix rm_nav_bringup)/share/rm_nav_bringup/rviz/sim_perception.rviz' > /tmp/rviz.log 2>&1 < /dev/null & disown
```

## Stop

Do not use `pkill -f`; it can match its own command and hang.

Stop simulation, perception, planning, and RViz:

```bash
ps -C gzserver,gzclient,relocalization_node,traversability_mapper,esdf2d_node,laser_mapping,robot_state_publisher,costmap_inflator,jps_node,minco_planner_node,traj_tracker,path_tracker,rviz2 --no-headers -o pid 2>/dev/null | xargs -r kill -9
ps aux | grep -E 'relocalization|esdf2d|traversability_mapper|bringup_sim|sim_perception|sim_planner|rm_simulation|gzserver|gzclient|laser_mapping|robot_state_publisher|costmap_inflator|jps_node|minco_planner|traj_tracker|path_tracker|rviz2' | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2
ps aux | grep -E 'relocalization|esdf2d|traversability|gzserver|gzclient|minco|jps_node|traj_tracker|path_tracker' | grep -v grep | wc -l
```

Expected final output is `0`.

Stop everything except RViz:

```bash
ps -C gzserver,gzclient,relocalization_node,traversability_mapper,esdf2d_node,laser_mapping,robot_state_publisher,costmap_inflator,jps_node,minco_planner_node,traj_tracker,path_tracker --no-headers -o pid 2>/dev/null | xargs -r kill -9
ps aux | grep -E 'relocalization|esdf2d|traversability_mapper|bringup_sim|sim_perception|sim_planner|rm_simulation|gzserver|gzclient|laser_mapping|robot_state_publisher|costmap_inflator|jps_node|minco_planner|traj_tracker|path_tracker' | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

Stop only RViz:

```bash
ps -C rviz2 --no-headers -o pid 2>/dev/null | xargs -r kill -9
```

## Health Check

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep -E "relocalization|traversability|esdf|inflator|jps|minco|tracker"
ros2 topic echo /relocalization/status --once
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic hz /perception/costmap_2d
ros2 topic hz /perception/esdf_2d
ros2 topic hz /planner/costmap_inflated
```

Expected active nodes:

```text
relocalization_node
traversability_mapper
esdf2d_node
costmap_inflator
jps_node
minco_planner_node
traj_tracker
```

## Send A Test Goal

RViz: use `2D Goal Pose` in the `map` frame.

CLI:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 9.0, y: 6.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## When A Full Gazebo Restart Is Required

Restart Stage 1 completely after changing:

- robot xacro;
- Gazebo world files;
- mecanum/chassis plugin settings;
- lidar/IMU simulation settings;
- anything that changes `/odom`, `/livox/lidar`, or `/imu/data` publishers.

Rebuild changed packages before restarting if source files changed.

## Logs

Background mode writes:

| File | Source |
|---|---|
| `/tmp/bringup.log` | Stage 1 |
| `/tmp/perception.log` | Stage 2 |
| `/tmp/planner.log` | Stage 3 |
| `/tmp/rviz.log` | RViz |
| `/tmp/traj_tracker_debug.csv` | tracker debug CSV |
