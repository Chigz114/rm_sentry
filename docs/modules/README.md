# Module Cards

This directory contains fixed-format module contracts for the active `rm_sentry_sim_ws` pipeline.

Each module card uses the same sections:

```text
Purpose
Runtime Status
Inputs
Outputs
Internal Mechanism
State
Key Parameters
Failure Signatures
Code Map
Validation Hooks
Ownership Notes
Open Questions
```

Rules:

- Treat launch/yaml/source as the source of truth for effective behavior.
- Keep module cards short enough to scan.
- Put tuning procedures in `docs/runbooks/`.
- Put pass/fail criteria in `docs/testbook/`.
- Put historical reasons in `docs/history/`.
- Do not backfill uncertain history into module cards.

Active module cards:

| Module | Card |
|---|---|
| FAST-LIO2 sim state estimation | `fastlio2.md` |
| Relocalization | `relocalization.md` |
| Height-gated traversability | `traversability_map.md` |
| 2D ESDF | `esdf2d.md` |
| Costmap inflation | `costmap_inflation.md` |
| JPS | `jps.md` |
| MINCO | `minco.md` |
| Trajectory tracker | `traj_tracker.md` |
| Gazebo chassis and robot integration | `gazebo_chassis.md` |
