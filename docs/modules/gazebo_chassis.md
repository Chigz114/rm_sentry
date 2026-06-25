# Gazebo Chassis And Robot Integration

## Purpose

Provide the simulated robot, MID360 sensor path, chassis odometry, and command interface used by the navigation stack.

## Runtime Status

Active in Stage 1 and consumed by Stage 3.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/cmd_vel_chassis` | `geometry_msgs/Twist` | command frame | controller-dependent | `traj_tracker` or manual tools | chassis command |
| robot description | URDF/Xacro | robot frames | static at launch | `rm_nav_bringup` xacro | includes simulated sensor integration |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | sim odom | plugin-dependent | JPS, MINCO, tracker | current Stage 3 pose source |
| `/livox/lidar` | Livox-style lidar message | sensor frame | sim-dependent | FAST-LIO2 | simulated MID360 input |
| `/livox/imu` | IMU raw | IMU frame | sim-dependent | complementary filter | remapped to `/imu/data` |

## Internal Mechanism

The Stage 1 launch includes the Polar Bear simulation subset, loads the robot xacro, starts Gazebo world RM3V3, and exposes simulated chassis and sensor topics. The chassis plugin consumes `/cmd_vel_chassis` and publishes odometry.

## State

Gazebo owns the simulated robot pose, physics state, plugin state, and sensor streams.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| mecanum plugin `publish_rate` | `50` | robot xacro/plugin config | more responsive odom/control feedback | lower update rate, more lag |
| world | `RM3V3` | `bringup_sim.launch.py` argument | selects other world if available | n/a |
| `use_sim_time` | `True` | launch arguments | syncs ROS to sim clock | real-time clock behavior |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| controller publishes but robot does not move | plugin/topic mismatch or Gazebo issue | `/cmd_vel_chassis` subscribers |
| overshoot despite sane tracker math | plant update rate or acceleration response mismatch | `/odom` hz and command/odom comparison |
| no lidar/imu | simulation or sensor plugin not running | `/livox/lidar`, `/livox/imu` hz |

## Code Map

| Role | File or Function |
|---|---|
| Stage 1 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` |
| robot xacro integration | `src/pb_rm_simulation/src/rm_nav_bringup/urdf/sentry_robot_sim.xacro` |
| simulation robot xacro | `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/urdf/simulation_waking_robot.xacro` |
| world asset | `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/world/RM3V3/rm3v3_sym_v1.world` |

## Validation Hooks

Use `docs/testbook/system_validation.md` and `docs/testbook/control_validation.md`.

Quick checks:

```bash
ros2 topic hz /odom
ros2 topic info /cmd_vel_chassis -v
ros2 topic hz /livox/lidar
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- The exact real-chassis command/odometry contract should be documented during real-robot migration.
