# Mapping Validation

Use this document to decide whether the active height-gated traversability map and ESDF visualization are healthy enough for planning tests.

## Scope

```text
/cloud_registered + /Odometry + map -> lidar_odom
  -> traversability_mapper
  -> /perception/costmap_2d
  -> esdf2d_node
  -> /perception/esdf_2d
```

This document does not validate JPS/MINCO trajectory quality or controller tracking.

## Preconditions

Localization validation should pass first.

```bash
ros2 topic hz /perception/costmap_2d
ros2 topic hz /perception/esdf_2d
ros2 topic echo /perception/costmap_2d --once
ros2 topic echo /relocalization/status --once
```

RViz should use fixed frame `map`.

## Required Test Scenarios

| Scenario | Purpose |
|---|---|
| stationary startup view | checks static prior and frame alignment |
| low platform/low obstacle area | checks height-gated detection |
| open field area | checks false positives |
| dynamic obstacle moved or robot motion past obstacle | checks temporal clearing/decay behavior |
| full RM3V3 overview in RViz | checks global map extent and offset |

## Acceptance Criteria

| Criterion | Pass Condition |
|---|---|
| costmap exists | `/perception/costmap_2d` publishes |
| ESDF exists | `/perception/esdf_2d` publishes |
| frame | costmap and ESDF are in `map` |
| static geometry | major RM3V3 walls/obstacles align with Gazebo/RViz |
| low obstacle detection | known low obstacles are not silently cleared |
| clearing | temporary dynamic observations decay as configured unless static prior owns them |
| map extent | grid covers the current operating area |

## Metrics To Record

| Metric | Source |
|---|---|
| costmap rate | `ros2 topic hz /perception/costmap_2d` |
| ESDF rate | `ros2 topic hz /perception/esdf_2d` |
| occupied/free/unknown rough distribution | `ros2 topic echo /perception/costmap_2d --once`, RViz, or evidence metrics |
| visual alignment | RViz/Gazebo observation |
| stale obstacle clearing time | timed observation |

## Known Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| obstacle appears shifted | TF/relocalization mismatch | localization validation |
| low obstacle missing | `h_climb` or ground clamp issue | mapper parameters |
| map too noisy | hit thresholds or ground estimate issue | `n_min_*`, `delta_hit`, ground clamps |
| stale dynamic obstacle | `decay_tau` or static prior ownership | mapper logs and static world geometry |

## Evidence Packet

For important mapping changes, create a run under `docs/evidence/` and include:

- `metadata.md`
- `params_snapshot.yaml`
- `topic_hz.txt`
- screenshots or RViz notes
- `human_observation.md`
- `agent_readable_summary.md`

## Pass/Fail Template

```text
Mapping validation:
  date:
  scenario:
  git/build state:
  localization validation:
  costmap hz:
  ESDF hz:
  costmap frame:
  static alignment:
  low obstacle result:
  false positive observation:
  clearing observation:
  verdict: pass / fail / inconclusive
```
