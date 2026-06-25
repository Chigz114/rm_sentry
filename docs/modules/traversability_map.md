# Height-Gated Traversability Map

## Purpose

Convert the registered 3D cloud and pose information into a 2.5D occupancy grid suitable for planning in the RM3V3 simulation field.

## Runtime Status

Active in Stage 2. This replaces the removed ROG-Map runtime for the current sim pipeline.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO cloud frame | LIO-dependent | FAST-LIO2 | point source for height gating |
| `/Odometry` | `nav_msgs/Odometry` | LIO odom | LIO-dependent | FAST-LIO2 | used by mapper |
| `map -> lidar_odom` | TF | map/lidar_odom | relocalization-dependent | relocalization | global alignment |
| world file | Gazebo world XML | map/world | static | package share path | static prior source |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/perception/costmap_2d` | `nav_msgs/OccupancyGrid` | `map` | `10.0` Hz configured | ESDF, inflation, MINCO | `100` occupied, `0` free, `-1` unknown |

## Internal Mechanism

The mapper maintains a fixed field-sized grid. It estimates/clamps ground height, applies height-gated obstacle logic, uses distance-normalized hit thresholds, decays log odds over time, and unions dynamic observations with a static world prior loaded from the Gazebo world file.

## State

- Fixed occupancy grid geometry.
- Per-cell log odds.
- Ground estimates/clamps.
- Static prior occupancy derived from world geometry.
- Latest odometry and TF lookup state.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `resolution` | `0.1` | `sim_perception.launch.py` | coarser grid, lower compute | finer grid, higher compute |
| `width_m` / `height_m` | `13.0` / `9.0` | `sim_perception.launch.py` | larger field coverage | smaller coverage |
| `x_offset` / `y_offset` | `6.0` / `4.0` | `sim_perception.launch.py` | shifts grid origin relationship | shifts opposite |
| `h_climb` | `0.10` | `sim_perception.launch.py` | fewer low obstacles marked | more low height differences marked |
| `ground_clamp_lo/hi` | `-0.30/-0.08` | `sim_perception.launch.py` | wider ground clamp band | tighter clamp band |
| `delta_hit` | `1.0` | `sim_perception.launch.py` | faster occupancy confidence | slower occupancy confidence |
| `decay_tau` | `3.0` | `sim_perception.launch.py` | longer persistence | faster clearing |
| `occ_thresh` | `0.5` | `sim_perception.launch.py` | harder to mark occupied | easier to mark occupied |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| no costmap | Stage 2 node missing or inputs absent | `ros2 topic hz /perception/costmap_2d` |
| obstacles shifted from Gazebo | frame/TF/relocalization mismatch | RViz fixed frame `map`, TF tree |
| low obstacles missing | height gate or ground clamp issue | `h_climb`, ground clamp parameters |
| stale obstacles remain too long | decay too slow or static prior involved | `decay_tau`, static prior source |

## Code Map

| Role | File or Function |
|---|---|
| Stage 2 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` |
| Mapper implementation | `src/sentry_mapping/src/traversability_mapper_node.cpp` |

## Validation Hooks

Use `docs/testbook/mapping_validation.md`.

Quick checks:

```bash
ros2 topic hz /perception/costmap_2d
ros2 topic echo /perception/costmap_2d --once
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- The real-robot map source and static-prior strategy remain migration topics.
