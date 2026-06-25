# System Validation

Use this document to decide whether the end-to-end simulation stack is healthy after a cross-module change.

## Scope

```text
Gazebo + FAST-LIO2
  -> relocalization + traversability + ESDF
  -> costmap inflation + JPS
  -> MINCO
  -> traj_tracker
  -> Gazebo chassis
```

Module-level validation should be used first when a failure is localized.

## Preconditions

The three runtime stages should be running.

```bash
ros2 node list | grep -E "fastlio|relocalization|traversability|esdf|inflator|jps|minco|tracker"
ros2 topic hz /cloud_registered
ros2 topic hz /perception/costmap_2d
ros2 topic hz /planner/costmap_inflated
ros2 topic info /cmd_vel_chassis -v
```

## Required Test Routes

| Route | Purpose |
|---|---|
| open-space goal | verifies basic full-chain operation |
| long S curve | exposes speed, timing, tracking, and replanning issues |
| 90 degree corner near wall | exposes wall-risk and overshoot |
| narrow but valid passage | checks planning feasibility without over-inflation |
| stop-at-goal route | checks final approach and command stop |

Use the same start state and goal positions when comparing before/after changes.

## Acceptance Criteria

| Criterion | Pass Condition |
|---|---|
| startup | all active nodes exist after staged bringup |
| perception | costmap aligns with field and updates |
| planning | JPS and MINCO produce outputs for valid goals |
| final check | nominal routes avoid unexpected MINCO fallback |
| control | tracker receives trajectory and publishes commands |
| safety | no collision or unacceptable wall-risk in required routes |
| goal behavior | robot stops near goal without persistent command output |
| evidence | important system changes produce an evidence packet |

## Metrics To Record

| Metric | Source |
|---|---|
| topic rates | `ros2 topic hz` |
| relocalization status | `/relocalization/status` |
| planner final check | `/tmp/planner.log` |
| fallback count | `/tmp/planner.log` |
| peak/mean tracking error | `/tmp/traj_tracker_debug.csv` |
| min visible wall margin | RViz/Gazebo observation |
| collision count | Gazebo/human observation |
| total route time | timestamped run notes |

## Known Failure Routing

| Symptom | Read First |
|---|---|
| no cloud or LIO odom | `docs/testbook/localization_validation.md` |
| costmap shifted or missing | `docs/testbook/mapping_validation.md` |
| no path, fallback, wall-hugging path | `docs/testbook/planning_validation.md` |
| overshoot, slow response, command saturation | `docs/testbook/control_validation.md` |

## Evidence Packet

For system-level changes, create a run under `docs/evidence/` using `docs/evidence/templates/`.

Minimum useful packet:

- `metadata.md`
- `params_snapshot.yaml`
- `topic_hz.txt`
- `metrics.csv`
- `human_observation.md`
- `agent_readable_summary.md`

## Pass/Fail Template

```text
System validation:
  date:
  route:
  git/build state:
  stage 1 health:
  stage 2 health:
  stage 3 health:
  planner final check:
  fallback:
  peak tracking error:
  wall-risk observation:
  collision:
  goal stop:
  evidence packet:
  verdict: pass / fail / inconclusive
```
