# Robotics Project Documentation Methodology

This document is a reusable documentation method for robotics projects that involve simulation, real sensors, middleware, planning, control, and coding agents. It is intentionally independent of this repository's current implementation.

## Core Principle

Do not ask a human or an agent to remember the whole project.

Maintain a small, stable routing layer that leads readers to the current source of truth, validation path, and decision context. Large robotics projects fail documentation-wise when code, launch files, parameters, topics, experiments, and historical explanations all accumulate in one long plan. The result is readable history, but poor operational truth.

The documentation system should answer five questions:

1. What system is currently active?
2. Where is the authoritative truth for runtime behavior?
3. How do modules connect through topics, frames, services, actions, messages, and launch order?
4. How can a person or agent validate that a module is working now?
5. Why were major design choices made, and which attempted paths should not be repeated blindly?

## Five Layers

### 1. Architecture Map

Purpose: describe the system boundary and runtime shape.

This layer answers "what exists" and "how data flows", not "how to tune". It should include system context, major modules, launch stages, lifecycle assumptions, sim/real differences, and the main data path.

Good files:

- `docs/index.md`
- `docs/architecture/overview.md`
- `docs/architecture/runtime_flows.md`

Rules:

- Keep diagrams close to runtime reality.
- Mark active, optional, and legacy branches.
- Include frame and clock assumptions for robotics projects.
- Avoid historical narrative except when it changes current operation.

### 2. Authoritative Reference

Purpose: expose consultable truth without duplicating source files.

The source of truth for runtime behavior should remain code, launch files, yaml files, generated interface definitions, and running nodes. Reference documentation should point to those sources and explain which parameters/interfaces matter.

Good files:

- `docs/reference/interfaces.md`
- `docs/reference/parameters.md`
- `docs/reference/active_legacy.md`
- generated API/interface pages if available

Rules:

- Do not copy full yaml files into documentation.
- Record where each parameter is defined and where it is overridden.
- Separate defaults from launch-time effective values.
- Mark high-risk parameters and coupled parameters.
- Treat topics, frames, QoS, clock source, and message schema as first-class interfaces.

### 3. How-To And Runbooks

Purpose: provide executable operational paths.

Runbooks answer "what should I do when X happens?" They should be shorter than design docs and more concrete than module descriptions.

Good files:

- `docs/runbooks/bringup.md`
- `docs/runbooks/debug_mapping.md`
- `docs/runbooks/debug_planning.md`
- `docs/runbooks/debug_control.md`
- `docs/runbooks/tuning.md`

Rules:

- Start with the cheapest checks.
- Include expected topic rates, status messages, and log signatures.
- Say what not to adjust first when a common failure appears.
- End each runbook with a validation criterion.

### 4. Validation Layer

Purpose: define how to decide whether a module is currently correct.

Robotics systems often look plausible in RViz while frames, timestamps, QoS, update rates, or controller saturation are wrong. Validation docs should define minimum checks that convert "looks okay" into "known good enough for this project".

Good files:

- `docs/validation/mapping_validation.md`
- `docs/validation/planning_validation.md`
- `docs/validation/control_validation.md`

Rules:

- Prefer measurable checks over prose.
- Include representative scenarios, not only unit tests.
- Include topic rates, frame consistency, failure thresholds, and logs.
- Keep validation distinct from tuning advice.

### 5. Decision Memory

Purpose: preserve why the project is shaped this way.

Decision docs should not be long work reports. They should capture durable choices, rejected alternatives, and consequences.

Good files:

- `docs/decisions/navigation_decisions.md`
- `docs/decisions/failed_attempts.md`
- ADR-style files for major choices

Rules:

- State the decision, context, alternatives, and consequences.
- Record failed attempts with enough detail to avoid repetition.
- Link back to logs or experiment notes when needed.
- Keep volatile progress updates elsewhere.

## Agent Bootstrap Layer

Purpose: give coding agents a routing surface, not a giant memory dump.

An agent bootstrap document should tell an agent:

- what to read first;
- which runtime chain is active;
- which legacy files are traps;
- where parameters and interfaces are sourced;
- how to validate a local change;
- what commands are safe for bringup, build, and cleanup.

It should be short, concrete, and boundary-setting. A useful bootstrap is a map and checklist, not an encyclopedia.

Good file:

- `docs/agent_bootstrap.md`

## Source-Of-Truth Rules

Every documentation set should define ownership rules:

| Information Type | Preferred Source Of Truth | Documentation Role |
|---|---|---|
| Runtime launch graph | launch files | summarize active stages and entrypoints |
| Parameter values | launch/yaml/code defaults/running node params | point to source, list high-risk effective values |
| Topic names and message types | code, interface files, `ros2 topic info` | maintain interface contract and diagnostics |
| Frame tree | TF publishers and running TF tree | document expected tree and failure modes |
| Build commands | package manifests and workspace convention | provide shortest safe commands |
| Historical decisions | decision docs and experiment logs | explain why and what not to repeat |
| File inventory | generated index or periodic audit | route readers, avoid becoming stale truth |

If documentation conflicts with code, launch, or a running node, update the documentation or mark the discrepancy. Do not let a handwritten parameter table silently override actual configuration.

## Recommended Minimal Structure

For a robotics repository that is changing quickly, start with this:

```text
docs/
  index.md
  agent_bootstrap.md

  architecture/
    overview.md
    runtime_flows.md

  reference/
    interfaces.md
    parameters.md
    active_legacy.md

  runbooks/
    bringup.md
    debug_mapping.md
    debug_planning.md
    debug_control.md
    tuning.md

  validation/
    mapping_validation.md
    planning_validation.md
    control_validation.md

  decisions/
    navigation_decisions.md
    failed_attempts.md
```

Do not create every file at once unless there is content worth maintaining. A small set of accurate routing documents is better than a large stale documentation tree.

## Writing Rules

- Put navigation before explanation.
- Put current facts before history.
- Put active chains before alternatives.
- Put validation beside every important module.
- Keep source references explicit.
- Separate reference, runbook, and decision memory.
- Mark legacy code clearly.
- Record failure signatures, not only success paths.
- Prefer tables for interfaces and parameters.
- Keep long experiment logs out of bootstrap and reference docs.

## Maintenance Rules

- Any launch change that changes runtime topology must update architecture/runtime docs.
- Any high-risk parameter change must update the parameter reference or decision notes.
- Any new recurring debug procedure must become a runbook.
- Any repeated failed experiment must be recorded as a failed attempt.
- Any renamed topic, frame, executable, or package must update the interface reference.
- Generated artifacts should be generated or ignored, not manually curated forever.

## Practical Rollout

Use three steps:

1. Create the smallest useful entry layer: `index`, `agent_bootstrap`, `runtime_flows`, `interfaces`, `parameters`, and one decision file.
2. Split reference from runbook content as the project stabilizes.
3. Add validation docs and automate what can be generated.

The end goal is not more Markdown. The goal is a durable, searchable, versioned project knowledge system where humans and agents can quickly find the right local truth.

