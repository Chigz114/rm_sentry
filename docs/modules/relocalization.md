# Relocalization

## Purpose

Publish the global alignment transform `map -> lidar_odom` so Stage 2 perception outputs can live in the field/world frame.

## Runtime Status

Active in Stage 2.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO cloud frame | LIO-dependent | FAST-LIO2 | accumulated for optional ICP refinement |
| world file | Gazebo world XML | map/world | static | package share path | `pb_rm_simulation/world/RM3V3/rm3v3_sym_v1.world` |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `map -> lidar_odom` | TF | map/lidar_odom | `50.0` Hz configured | traversability, RViz | seed plus optional ICP |
| `/relocalization/status` | `std_msgs/String` | n/a | 1 Hz in source | humans/agents | health and ICP result summary |

## Internal Mechanism

The node starts from a known birth-zone seed and can refine the map-frame alignment with 2D ICP against geometry loaded from the generated world file. It then publishes `map -> lidar_odom` continuously.

## State

- Seed pose: `seed_x`, `seed_y`, `seed_yaw`.
- Accumulated cloud samples before ICP.
- Current transform estimate.
- Static reference geometry parsed from the world file.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `seed_x` | `5.5` | `sim_perception.launch.py` | shifts initial x | shifts initial x opposite |
| `seed_y` | `3.5` | `sim_perception.launch.py` | shifts initial y | shifts initial y opposite |
| `seed_yaw` | `0.15` | `sim_perception.launch.py` | rotates initial estimate | rotates initial estimate opposite |
| `accumulate_count` | `30` | `sim_perception.launch.py` | more ICP input, slower first refinement | faster but less averaged |
| `refine_with_icp` | `True` | `sim_perception.launch.py` | enables correction | seed-only transform |
| `tf_rate_hz` | `50.0` | `sim_perception.launch.py` | higher TF publish rate | lower TF publish rate |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| no `map -> lidar_odom` | relocalization not running or crashed | `ros2 node list`, `ros2 topic echo /relocalization/status --once` |
| costmap globally shifted | seed/ICP/world alignment wrong | status text and RViz field overlay |
| callbacks do not receive cloud | QoS or topic mismatch | `ros2 topic info /cloud_registered -v` |

## Code Map

| Role | File or Function |
|---|---|
| Stage 2 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` |
| Node implementation | `src/sentry_perception/sentry_perception/relocalization_node.py` |
| World geometry input | `src/pb_rm_simulation/src/rm_simulation/pb_rm_simulation/world/RM3V3/rm3v3_sym_v1.world` |

## Validation Hooks

Use `docs/testbook/localization_validation.md` and `docs/testbook/mapping_validation.md`.

Quick checks:

```bash
ros2 topic echo /relocalization/status --once
ros2 run tf2_tools view_frames
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- Real-robot startup seeding and global localization policy are not finalized here.
