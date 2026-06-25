# Dataflow And Frames

This file is the current high-level dataflow and frame contract. For full topic tables, use `docs/reference/interfaces.md`.

## Main Dataflow

```text
/livox/lidar + /imu/data
  -> FAST-LIO2
  -> /cloud_registered + /Odometry

/cloud_registered
  -> relocalization_node
  -> TF map -> lidar_odom

/cloud_registered + /Odometry + TF
  -> traversability_mapper
  -> /perception/costmap_2d
  -> esdf2d_node
  -> /perception/esdf_2d

/perception/costmap_2d
  -> costmap_inflator
  -> /planner/costmap_inflated
  -> jps_node
  -> /planner/path
  -> minco_planner_node
  -> /planner/traj_samples
  -> traj_tracker
  -> /cmd_vel_chassis
```

## Frame Contract

Expected conceptual frame chain:

```text
map -> lidar_odom -> base_link
```

Current simulation-specific Stage 3 pose source:

```text
/odom
```

This split is deliberate. Stage 2 uses FAST-LIO cloud/odometry plus relocalization for mapping. Stage 3 uses Gazebo/chassis `/odom` for current planner/controller tests.

## Topic Contract

| Topic | Role |
|---|---|
| `/cloud_registered` | dense registered cloud for relocalization and traversability |
| `/Odometry` | FAST-LIO odometry for traversability |
| `/odom` | current Stage 3 simulation pose source |
| `/relocalization/status` | relocalization health signal |
| `/perception/costmap_2d` | active planning costmap |
| `/perception/esdf_2d` | ESDF visualization/reference |
| `/planner/costmap_inflated` | JPS input grid |
| `/goal_pose` | target pose from RViz 2D Goal Pose or CLI |
| `/planner/path` | JPS output path |
| `/planner/traj_samples` | timed MINCO trajectory for controller |
| `/cmd_vel_chassis` | chassis command output |

## Verification

Use:

- `docs/testbook/localization_validation.md`
- `docs/testbook/mapping_validation.md`
- `docs/testbook/planning_validation.md`
- `docs/testbook/control_validation.md`
- `docs/testbook/system_validation.md`
