# Runtime Flows

This document records the current runtime stages and data flow for `rm_sentry_sim_ws`.

## Current Three-Stage Flow

```text
Stage 1: Simulation + LIO
  Gazebo world + simulated robot + simulated MID360
    -> Livox-style lidar/IMU topics
    -> FAST-LIO2
    -> /cloud_registered, /Odometry

Stage 2: Perception + Map Frame Alignment
  /cloud_registered
    -> relocalization_node
    -> TF map -> lidar_odom

  /cloud_registered + /Odometry + TF
    -> traversability_mapper
    -> /perception/costmap_2d
    -> esdf2d_node
    -> /perception/esdf_2d

Stage 3: Planning + Control
  /perception/costmap_2d
    -> costmap_inflator
    -> /planner/costmap_inflated
    -> jps_node + /goal_pose + /odom
    -> /planner/path
    -> minco_planner_node + /perception/costmap_2d + /odom
    -> /planner/traj_samples
    -> traj_tracker + /odom
    -> /cmd_vel_chassis
```

## Stage 1: Gazebo + FAST-LIO2

Launch:

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RM3V3 lio:=fastlio mode:=mapping lio_rviz:=False nav_rviz:=False
```

Source:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py`
- `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml`

Important outputs:

| Topic | Type | Meaning |
|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO registered cloud |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO odometry |
| `/odom` | `nav_msgs/Odometry` | Gazebo/chassis odometry used by current planner/tracker stage |

Notes:

- `bringup_sim.launch.py` is now a lightweight Stage 1 entry: Gazebo + simulated MID360 + IMU filter + FAST-LIO.
- Older upstream-style Nav2, SLAM, segmentation, Point-LIO, ICP, and TEB branches were removed from the local `pb_rm_simulation` vendor subset.
- FAST-LIO sim config uses MID360/Livox settings and `dense_publish_en: true` so the traversability mapper can consume dense registered clouds.

## Stage 2: Relocalization + Height-Gated Traversability + ESDF

Launch:

```bash
ros2 launch rm_nav_bringup sim_perception.launch.py perception_rviz:=false
```

Source:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py`
- `src/sentry_perception/sentry_perception/relocalization_node.py`
- `src/sentry_mapping/src/traversability_mapper_node.cpp`
- `src/sentry_perception/sentry_perception/esdf2d_node.py`

Nodes:

| Node | Role |
|---|---|
| `relocalization_node` | Publishes `map -> lidar_odom` from seed plus optional ICP refinement |
| `traversability_mapper` | Builds height-gated 2.5D occupancy grid in `map` frame |
| `esdf2d_node` | Converts occupancy grid into 2D ESDF point cloud for visualization and reference |

Important outputs:

| Topic or TF | Meaning |
|---|---|
| `/relocalization/status` | ICP/seed status |
| `map -> lidar_odom` | global alignment transform |
| `/perception/costmap_2d` | current planning occupancy grid |
| `/perception/esdf_2d` | 2D ESDF visualization cloud, intensity = distance + 10 |

Notes:

- Current simulation mapping uses `traversability_mapper`; old ROG-Map runtime files have been removed from `sim_ws`.
- `traversability_mapper` directly consumes `/cloud_registered` and `/Odometry`.
- `traversability_mapper` uses the generated Gazebo world file as a static prior and publishes a `map` frame costmap.

## Stage 3: JPS + MINCO + Traj Tracker

Launch:

```bash
ros2 launch sentry_planner sim_planner.launch.py
```

Source:

- `src/sentry_planner/launch/sim_planner.launch.py`
- `src/sentry_planner/sentry_planner/costmap_inflator.py`
- `src/sentry_planner/sentry_planner/jps_node.py`
- `src/sentry_planner/sentry_planner/minco_planner_node.py`
- `src/sentry_planner/sentry_planner/minco_solver_2d.py`
- `src/sentry_controller/sentry_controller/traj_tracker.py`

Nodes:

| Node | Role |
|---|---|
| `costmap_inflator` | Inflates `/perception/costmap_2d` for JPS topology planning |
| `jps_node` | Finds grid path from current pose to `/goal_pose` |
| `minco_planner_node` | Converts JPS path into a smooth timed trajectory |
| `traj_tracker` | Tracks `/planner/traj_samples` and publishes `/cmd_vel_chassis` |

Current Stage 3 intentionally uses `/odom` for JPS/MINCO/tracker pose in simulation. This avoids FAST-LIO simulation drift contaminating the controller while still using FAST-LIO cloud outputs for perception.

## Frame Expectations

Expected conceptual frame chain:

```text
map -> lidar_odom -> base_link
```

Current planning products:

| Output | Expected Frame |
|---|---|
| `/perception/costmap_2d` | `map` |
| `/perception/esdf_2d` | `map` |
| `/planner/path` | should match costmap/goal handling |
| `/planner/path_vis` | path visualization from MINCO |

Current Stage 3 pose source:

| Node | Pose Source |
|---|---|
| `jps_node` | `/odom`, `use_raw_odom=True` in launch |
| `minco_planner_node` | `/odom` in launch |
| `traj_tracker` | `/odom` in launch |

This split is deliberate for current simulation tests.

## Runtime Health Checks

Expected checks from `docs/runbooks/bringup.md`:

```bash
ros2 topic echo /relocalization/status --once
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic hz /perception/costmap_2d
ros2 topic hz /perception/esdf_2d
ros2 topic hz /planner/costmap_inflated
ros2 node list | grep -E "relocalization|traversability|esdf|inflator|jps|minco|tracker"
```

If any stage is missing, debug upstream first. Stage 3 cannot be judged until Stage 2 is publishing a plausible costmap.
