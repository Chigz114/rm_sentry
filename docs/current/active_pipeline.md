# Active Pipeline

This file records the currently recommended runtime path. It is intentionally short.

## Pipeline

```text
Stage 1: Gazebo + FAST-LIO2
  rm_nav_bringup bringup_sim.launch.py
  -> /cloud_registered
  -> /Odometry
  -> /odom

Stage 2: Relocalization + Mapping
  rm_nav_bringup sim_perception.launch.py
  /cloud_registered -> relocalization_node -> map -> lidar_odom
  /cloud_registered + /Odometry + TF -> traversability_mapper -> /perception/costmap_2d
  /perception/costmap_2d -> esdf2d_node -> /perception/esdf_2d

Stage 3: Planning + Control
  sentry_planner sim_planner.launch.py
  /perception/costmap_2d -> costmap_inflator -> /planner/costmap_inflated
  /planner/costmap_inflated + /goal_pose + /odom -> jps_node -> /planner/path
  /planner/path + /perception/costmap_2d + /odom -> minco_planner_node -> /planner/traj_samples
  /planner/traj_samples + /odom -> traj_tracker -> /cmd_vel_chassis
```

## Current Controller

`traj_tracker` is the active controller.

The old `path_tracker`, `goal_controller`, pure-pursuit code, and fixed trajectory publisher have been removed from the current workspace. MPC remains a future option, not the current implementation.

## Current Mapping Path

`traversability_mapper` is the active sim mapping path.

The old ROG-Map wrapper/config/launch/RViz files have been removed from this workspace. Use the archive/history docs only if a task explicitly asks to reconstruct or compare that path.

## Launch Order

Use the staged order in `docs/runbooks/bringup.md`:

1. Stage 1: Gazebo + FAST-LIO2.
2. Stage 2: relocalization + traversability + ESDF.
3. Stage 3: planner + tracker.
