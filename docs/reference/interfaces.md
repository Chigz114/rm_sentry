# Interface Reference

This document records the current high-value topic, frame, and message contracts. Verify against running ROS when debugging, because launch files and remaps are the final runtime authority.

## Main Data Topics

| Topic | Type | Publisher | Consumer | Notes |
|---|---|---|---|---|
| `/livox/lidar` | Livox-style lidar message | simulated Livox driver stack | FAST-LIO2 | configured by Stage 1 stack |
| `/imu/data` | IMU | sim/IMU filter path | FAST-LIO2 | FAST-LIO sim yaml uses this topic |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO2 | relocalization, traversability | dense registered cloud |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO2 | traversability | LIO odometry |
| `/odom` | `nav_msgs/Odometry` | Gazebo/chassis path | JPS, MINCO, traj_tracker | current Stage 3 pose source |
| `/relocalization/status` | `std_msgs/String` | relocalization | humans/agents | seed/ICP status |
| `/perception/costmap_2d` | `nav_msgs/OccupancyGrid` | traversability_mapper | costmap_inflator, MINCO, ESDF | active planning costmap |
| `/perception/esdf_2d` | `sensor_msgs/PointCloud2` | esdf2d_node | RViz/humans | visualization ESDF, not the main random-access MINCO source |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 2D Goal Pose or CLI | jps_node | target for path planning |
| `/planner/costmap_inflated` | `nav_msgs/OccupancyGrid` | costmap_inflator | jps_node | JPS binary grid |
| `/planner/path` | `nav_msgs/Path` | jps_node | minco_planner_node, RViz | JPS waypoint path |
| `/planner/jps_viz` | `visualization_msgs/MarkerArray` | jps_node | RViz | JPS markers |
| `/planner/path_vis` | `nav_msgs/Path` | minco_planner_node | RViz | dense MINCO visualization |
| `/planner/traj_samples` | `std_msgs/Float64MultiArray` | minco_planner_node | traj_tracker | current control-facing trajectory |
| `/planner/minco_traj` | `visualization_msgs/MarkerArray` | minco_planner_node | RViz | MINCO markers |
| `/planner/minco_info` | `visualization_msgs/Marker` | minco_planner_node | RViz | MINCO text/debug marker |
| `/cmd_vel_chassis` | `geometry_msgs/Twist` | traj_tracker/manual tools | Gazebo mecanum/chassis plugin | current command output |

## Trajectory Sample Contract

`/planner/traj_samples` is published as `std_msgs/Float64MultiArray`.

Each row is:

```text
[t, x, y, vx, vy, ax, ay, yaw]
```

The current tracker reads this timed trajectory and computes velocity commands using feedforward plus feedback, then applies speed and acceleration limits.

If this schema changes, update both:

- `src/sentry_planner/sentry_planner/minco_planner_node.py`
- `src/sentry_controller/sentry_controller/traj_tracker.py`

## Costmap Semantics

`/perception/costmap_2d`:

| Cell Value | Meaning |
|---|---|
| `100` | occupied |
| `0` | free |
| `-1` | unknown |

`esdf2d_node` currently uses:

```text
intensity = distance_to_nearest_obstacle_m + 10.0
```

This is for RViz-friendly visualization. MINCO currently subscribes to the occupancy grid and maintains its own internal distance representation for planning costs.

## Frames

Expected core frame chain:

```text
map -> lidar_odom -> base_link
```

Important conventions:

| Component | Frame Expectation |
|---|---|
| `relocalization_node` | publishes `map -> lidar_odom` |
| `traversability_mapper` | publishes costmap in `map` |
| `esdf2d_node` | preserves costmap frame |
| RViz | fixed frame should be `map` for current perception/planning view |

Current Stage 3 uses `/odom` directly for pose. This is a simulation-specific control simplification and should not be silently copied into real-robot operation.

## QoS Notes

Known pattern:

- Sensor-like inputs often use best-effort subscribers.
- `relocalization_node` subscribes to `/cloud_registered` with reliable QoS for ICP accumulation.
- Costmap and ESDF publishers use reliable volatile publishers.

If a topic appears in `ros2 topic list` but callbacks do not fire, check QoS compatibility before changing algorithm logic.

## Verification Commands

```bash
ros2 topic info /cloud_registered -v
ros2 topic info /perception/costmap_2d -v
ros2 topic info /planner/traj_samples -v
ros2 topic echo /relocalization/status --once
ros2 run tf2_tools view_frames
```
