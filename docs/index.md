# RM Sentry Sim Workspace Documentation Index

This is the routing entry for this `rm_sentry_sim_ws` checkout.

The current workspace is a simulation-first navigation stack for a RoboMaster sentry robot. The active chain is:

```text
Gazebo + FAST-LIO2
  -> relocalization + height-gated traversability + 2D ESDF
  -> costmap inflation + JPS
  -> MINCO timed trajectory
  -> traj_tracker
  -> /cmd_vel_chassis
```

## Read First

| Need | Read |
|---|---|
| Agent or new contributor onboarding | [`agent_bootstrap.md`](agent_bootstrap.md) |
| Runtime stage and data flow | [`architecture/runtime_flows.md`](architecture/runtime_flows.md) |
| Topics, frames, and message contracts | [`reference/interfaces.md`](reference/interfaces.md) |
| Current high-risk parameters and source files | [`reference/parameters.md`](reference/parameters.md) |
| Active vs legacy components | [`reference/active_legacy.md`](reference/active_legacy.md) |
| Why the current navigation stack looks this way | [`decisions/navigation_decisions.md`](decisions/navigation_decisions.md) |
| Start/stop commands | [`runbooks/bringup.md`](runbooks/bringup.md) |
| Planning failure, MINCO fallback, wall-hugging | [`runbooks/debug_planning.md`](runbooks/debug_planning.md) |
| Overshoot, speed, chassis response | [`runbooks/debug_control.md`](runbooks/debug_control.md) |
| Planning pass/fail checks | [`validation/planning_validation.md`](validation/planning_validation.md) |
| Control pass/fail checks | [`validation/control_validation.md`](validation/control_validation.md) |
| Long historical work log | [`height_gated_traversability_plan.md`](height_gated_traversability_plan.md) |
| File inventory snapshot | [`file_inventory.md`](file_inventory.md) |

## Documentation Roles

`docs/index.md` is a routing page. It should stay short.

`docs/agent_bootstrap.md` tells a coding agent what to read first, what is active, and which older files are not the main chain.

`docs/architecture/runtime_flows.md` records runtime stages and data flow. It should be updated when launch topology changes.

`docs/reference/interfaces.md` records topic/frame contracts and where to verify them.

`docs/reference/parameters.md` records high-risk effective parameters and their source of truth. It must not become a copy of every yaml file.

`docs/reference/active_legacy.md` is the central map of current, baseline, and historical components.

`docs/runbooks/bringup.md` replaces the older operations reference with short start/stop commands.

`docs/runbooks/debug_planning.md` and `docs/runbooks/debug_control.md` are the first operational guides for planning and control failures.

`docs/validation/planning_validation.md` and `docs/validation/control_validation.md` define pass/fail checks after tuning or code changes.

`docs/decisions/navigation_decisions.md` records durable design decisions and failed routes worth remembering.

`docs/height_gated_traversability_plan.md` remains a work log and experiment history. It is valuable, but it is not the first source to read for current runtime truth.

## Source-Of-Truth Policy

If these docs disagree with launch files, yaml files, code defaults, or running ROS parameters, treat the executable source as authoritative and update the docs.

Primary runtime entry files:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py`
- `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py`
- `src/sentry_planner/launch/sim_planner.launch.py`
