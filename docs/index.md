# RM Sentry Sim Workspace Documentation Index

This is the routing entry for this `rm_sentry_sim_ws` checkout.

Start with current truth, then load only the layer needed for the task.

## Read First

| Need | Read |
|---|---|
| Current project state and active chain | [`current/project_state.md`](current/project_state.md) |
| Current architecture overview | [`current/architecture.md`](current/architecture.md) |
| Active pipeline only | [`current/active_pipeline.md`](current/active_pipeline.md) |
| Dataflow and frame contract | [`current/dataflow_and_frames.md`](current/dataflow_and_frames.md) |
| Agent or new contributor onboarding | [`current/agent_bootstrap.md`](current/agent_bootstrap.md) |
| Active vs legacy components | [`current/active_legacy.md`](current/active_legacy.md) |
| Runtime stage and data flow | [`architecture/runtime_flows.md`](architecture/runtime_flows.md) |
| Module contracts | [`modules/README.md`](modules/README.md) |
| Topics, frames, and message contracts | [`reference/interfaces.md`](reference/interfaces.md) |
| Current high-risk parameters and source files | [`reference/parameters.md`](reference/parameters.md) |
| Start/stop commands | [`runbooks/bringup.md`](runbooks/bringup.md) |

## Documentation Layers

| Layer | Directory | Role |
|---|---|---|
| Current truth | `current/` | project state, agent routing, active/legacy map |
| Architecture | `architecture/` | runtime stages and cross-module dataflow |
| Modules | `modules/` | fixed-format module contracts |
| Reference | `reference/` | topic/frame/parameter/file facts, with executable sources authoritative |
| Runbooks | `runbooks/` | operational how-to and debugging flows |
| Testbook | `testbook/` | pass/fail checks and required metrics |
| Evidence | `evidence/` | structured records from real runs |
| History | `history/` | decisions, bug records, indexes, and timeline |
| Archive | `archive/` | deprecated implementations and non-default paths |

## Common Routes

| Task | Read |
|---|---|
| Bring up or stop the simulation | [`runbooks/bringup.md`](runbooks/bringup.md) |
| Debug planning, fallback, or wall risk | [`runbooks/debug_planning.md`](runbooks/debug_planning.md) |
| Debug overshoot, speed, or chassis response | [`runbooks/debug_control.md`](runbooks/debug_control.md) |
| Validate localization or relocalization | [`testbook/localization_validation.md`](testbook/localization_validation.md) |
| Validate mapping/costmap/ESDF | [`testbook/mapping_validation.md`](testbook/mapping_validation.md) |
| Validate planning | [`testbook/planning_validation.md`](testbook/planning_validation.md) |
| Validate control | [`testbook/control_validation.md`](testbook/control_validation.md) |
| Validate full system behavior | [`testbook/system_validation.md`](testbook/system_validation.md) |
| Understand why the current navigation stack looks this way | [`history/decisions/navigation_decisions.md`](history/decisions/navigation_decisions.md) |
| Record a new run | [`evidence/README.md`](evidence/README.md) |
| Check deprecated paths | [`archive/deprecated_implementations.md`](archive/deprecated_implementations.md) |

## Active Pipeline

```text
Gazebo + FAST-LIO2
  -> relocalization + height-gated traversability + 2D ESDF
  -> costmap inflation + JPS
  -> MINCO timed trajectory
  -> traj_tracker
  -> /cmd_vel_chassis
```

For the authoritative state board, read `current/project_state.md`.

## Source-Of-Truth Policy

If these docs disagree with launch files, yaml files, code defaults, or running ROS parameters, treat the executable source as authoritative and update the docs.

Primary runtime entry files:

- `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py`
- `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py`
- `src/sentry_planner/launch/sim_planner.launch.py`

## Historical Long Log

`height_gated_traversability_plan.md` is preserved as the long historical work log. It is valuable for deep context, but it is not the first source for current runtime truth.
