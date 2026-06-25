# Costmap Inflation

## Purpose

Inflate the active perception costmap into a binary planning grid for JPS topology search.

## Runtime Status

Active in Stage 3.

## Inputs

| Input | Type | Frame | Rate | Source | Notes |
|---|---|---|---|---|---|
| `/perception/costmap_2d` | `nav_msgs/OccupancyGrid` | `map` | mapper-dependent | traversability_mapper | active occupancy grid |

## Outputs

| Output | Type | Frame | Rate | Consumer | Notes |
|---|---|---|---|---|---|
| `/planner/costmap_inflated` | `nav_msgs/OccupancyGrid` | input frame | costmap callback | `jps_node` | binary inflated grid |

## Internal Mechanism

The node expands occupied cells by a configured metric radius and republishes the inflated grid. It is intentionally kept relatively thin so feasible narrow passages are not automatically removed.

## State

Stores the latest inflation parameter and publishes each inflated input grid.

## Key Parameters

| Parameter | Current Value | Source | Effect When Increased | Effect When Decreased |
|---|---:|---|---|---|
| `inflation_radius_m` | `0.30` | `sim_planner.launch.py` | JPS stays farther from obstacles but may lose narrow feasible passages | more feasible passages but more wall-adjacent topology |

## Failure Signatures

| Symptom | Likely Meaning | First Check |
|---|---|---|
| JPS no path in narrow areas | inflation too large or upstream obstacle geometry wrong | compare raw/inflated costmaps |
| JPS path hugs walls | inflation thin by design; safety expected downstream | MINCO final check and wall-risk validation |
| no inflated map | costmap_inflator missing or no input | `ros2 topic hz /planner/costmap_inflated` |

## Code Map

| Role | File or Function |
|---|---|
| Stage 3 launch | `src/sentry_planner/launch/sim_planner.launch.py` |
| Node implementation | `src/sentry_planner/sentry_planner/costmap_inflator.py` |

## Validation Hooks

Use `docs/testbook/planning_validation.md`.

Quick checks:

```bash
ros2 topic hz /planner/costmap_inflated
ros2 topic echo /planner/costmap_inflated --once
```

## Ownership Notes

Add human-authored recall notes later.

## Open Questions

- The best split between binary inflation and downstream clearance optimization should be revisited only with evidence from narrow-passage and wall-risk routes.
