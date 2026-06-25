# Localization Validation

Use this document to decide whether Stage 1 FAST-LIO2 and Stage 2 relocalization are healthy enough for perception and planning tests.

## Scope

```text
simulated MID360 + IMU
  -> FAST-LIO2
  -> /cloud_registered + /Odometry
  -> relocalization_node
  -> map -> lidar_odom
```

This document does not validate planning or control performance.

## Preconditions

Stage 1 and Stage 2 should be running.

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic echo /relocalization/status --once
ros2 run tf2_tools view_frames
```

RViz fixed frame should be `map` when checking field alignment.

## Required Test Scenarios

| Scenario | Purpose |
|---|---|
| cold startup from default RM3V3 birth zone | checks seed and ICP startup |
| stationary after startup | checks pose stability and TF publication |
| slow manual or planned movement | checks cloud/odom continuity |
| restart Stage 2 without restarting Gazebo | checks whether relocalization recovers from existing Stage 1 outputs |

## Acceptance Criteria

| Criterion | Pass Condition |
|---|---|
| FAST-LIO cloud | `/cloud_registered` publishes steadily |
| FAST-LIO odom | `/Odometry` publishes steadily |
| relocalization status | `/relocalization/status` publishes and reports a usable seed or ICP result |
| TF | `map -> lidar_odom` exists |
| visual alignment | registered cloud/costmap aligns with RM3V3 geometry closely enough for planning tests |
| no large jumps | no unexplained pose/frame jump during the validation window |

## Metrics To Record

| Metric | Source |
|---|---|
| `/cloud_registered` rate | `ros2 topic hz` |
| `/Odometry` rate | `ros2 topic hz` |
| relocalization status text | `/relocalization/status` |
| TF tree snapshot | `tf2_tools view_frames` |
| visual map alignment | RViz observation |

## Known Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| no `/cloud_registered` | FAST-LIO or simulated lidar missing | Stage 1 launch log and `/livox/lidar` |
| no `map -> lidar_odom` | relocalization missing or failed | `/relocalization/status` |
| costmap shifted globally | seed/ICP/world alignment problem | relocalization status and world file source |

## Evidence Packet

For important localization changes, create a run under `docs/evidence/` and include:

- `metadata.md`
- `topic_hz.txt`
- `tf_tree.txt` or generated frame graph
- `human_observation.md`
- `agent_readable_summary.md`

## Pass/Fail Template

```text
Localization validation:
  date:
  route/scenario:
  git/build state:
  /cloud_registered hz:
  /Odometry hz:
  relocalization status:
  map -> lidar_odom present:
  visible alignment:
  pose jump observed:
  verdict: pass / fail / inconclusive
```
