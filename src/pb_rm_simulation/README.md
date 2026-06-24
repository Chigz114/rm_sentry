# Polar Bear Simulation Vendor Subset

This directory is a lightweight vendor subset derived from the Polar Bear RoboMaster simulation stack.

It keeps the `pb_rm_simulation` name and package identity for attribution and continuity, but it is no longer a full clone of the upstream repository. The original upstream project included Gazebo worlds, Nav2/TEB navigation, Point-LIO, ICP localization, real-robot bringup examples, maps, PCD files, and documentation assets. This workspace only keeps the pieces needed by the current RM sentry simulation chain.

## Kept Packages

```text
src/rm_simulation/pb_rm_simulation/          # Gazebo launch, RM3V3 world, robot xacro fallback, MID360 mesh
src/rm_simulation/livox_laser_simulation_RO2/ # Gazebo MID360 plugin and scan pattern
src/rm_nav_bringup/                         # lightweight sim bringup, FAST-LIO config, RViz
src/rm_localization/FAST_LIO/               # FAST-LIO2 localization/mapping node
src/rm_driver/livox_ros_driver2/src/        # Livox CustomMsg definitions and driver library dependency
src/rm_perception/imu_complementary_filter/ # IMU orientation filter used before FAST-LIO
```

## Removed From This Subset

The following upstream areas were intentionally removed because they are not part of the current JPS + MINCO + `traj_tracker` simulation path:

- Nav2 / TEB / costmap converter packages;
- Point-LIO branch;
- ICP registration branch;
- linefit segmentation and pointcloud-to-laserscan branch;
- fake velocity transform branch;
- RMUC/RMUL PCD, map, posegraph, and large mesh assets;
- old real-robot bringup examples;
- upstream `.git`, submodule metadata, devcontainer, Docker, and demo media.

## Active Stage 1 Flow

The current active flow is:

```text
rm_nav_bringup/bringup_sim.launch.py
-> pb_rm_simulation/rm_simulation.launch.py
-> Gazebo RM3V3 world + simulated MID360 + /livox/imu
-> imu_complementary_filter
-> fast_lio/fastlio_mapping
-> /cloud_registered + /Odometry
```

Stage 2 and Stage 3 live outside this vendor subset:

```text
sentry_mapping + sentry_perception
-> sentry_planner
-> sentry_controller
```

## Attribution

The package names and retained file layout intentionally preserve the Polar Bear RoboMaster origin. Local modifications in this workspace are focused on portability, RM3V3 simulation, FAST-LIO parameterization, and integration with the current sentry planning/control stack.
