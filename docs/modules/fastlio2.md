# FAST-LIO2 Sim State Estimation

## Purpose

Produce registered point clouds and LIO odometry from the simulated MID360/Livox-style lidar and IMU topics.

## Runtime Status

Active in Stage 1.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/livox/lidar` | Livox-style lidar message | Livox frames | sim-dependent | simulated Livox driver stack | configured by FAST-LIO yaml |
| `/imu/data` | IMU | IMU frame | sim-dependent | IMU filter path | remapped from `/livox/imu` through complementary filter |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | LIO/global output frame | lidar/update-dependent | relocalization, traversability | dense publish is enabled |
| `/Odometry` | `nav_msgs/Odometry` | LIO odom | LIO/update-dependent | traversability | not the current Stage 3 pose source |
| `/odom` static relation to `lidar_odom` | TF | `odom -> lidar_odom` | static | TF tree | provided by Stage 1 launch compatibility path |

## Internal Mechanism

The active launch starts Gazebo, the simulated MID360 path, an IMU complementary filter, and `fast_lio/fastlio_mapping`. FAST-LIO consumes the Livox lidar and filtered IMU topics using the sim yaml configuration, then publishes registered cloud and odometry outputs.

## State

FAST-LIO maintains its own incremental state estimation and map state internally. This card records only the ROS contract used by the rest of the workspace.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `preprocess.blind` | `0.5` | `fastlio_mid360_sim.yaml` | filters more near-field points | includes closer points, may add near-field noise |
| `mapping.fov_degree` | `360.0` | `fastlio_mid360_sim.yaml` | wider accepted field | narrower accepted field |
| `mapping.det_range` | `100.0` | `fastlio_mid360_sim.yaml` | farther accepted points | shorter range |
| `mapping.extrinsic_est_en` | `false` | `fastlio_mid360_sim.yaml` | online extrinsic if enabled | fixed sim extrinsic |
| `publish.dense_publish_en` | `true` | `fastlio_mid360_sim.yaml` | required dense cloud for traversability | sparse output may break mapping quality |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| no `/cloud_registered` | FAST-LIO or input topic not running | `ros2 topic hz /livox/lidar` and `/imu/data` |
| traversability has no updates | cloud missing or too sparse | `ros2 topic hz /cloud_registered` |
| costmap appears shifted due to LIO drift | Stage 2 alignment issue or LIO sim drift | `/relocalization/status`, TF tree |

## Code Map

| Role | File or Function |
|---|---|
| Stage 1 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` |
| FAST-LIO sim parameters | `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml` |
| robot/LiDAR extrinsic source | `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/measurement_params_sim.yaml` |

## Validation Hooks

Use `docs/testbook/localization_validation.md` and `docs/runbooks/bringup.md`.

Quick checks:

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic info /cloud_registered -v
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- The real-robot migration contract for replacing sim `/odom` with a real pose source is not finalized in this workspace.
