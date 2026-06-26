# ADR-001: 从 Nav2 upstream 风格收缩到 JPS + MINCO

## Status

accepted

## Context

当前目标是在 RM3V3 仿真场地中建立可解释、可调试的哨兵导航链。旧的上游风格 Nav2 管线包含较多不在当前目标内的分支和配置，容易让 agent 或人误判 active pipeline。

## Old approach

依赖上游仿真栈中更完整的导航结构和多种 planner/controller 选项。

## Problem with old approach

- 当前调试重点是可通行性图、路径拓扑、轨迹时序和底盘响应的一致性，不需要保留多套并行导航方案。
- 上游 Nav2 风格路径会让旧 launch、旧参数和旧文档成为重复真理来源。
- 多套 planner/controller 并存时，过冲、贴墙或 fallback 很难归因。

## Options considered

- 保留 Nav2 upstream 作为默认主链。
- 同时维护 Nav2 和自定义 JPS/MINCO。
- 将 active pipeline 收缩为 JPS + MINCO + `traj_tracker`。

## Decision

当前仿真主链采用：

```text
/perception/costmap_2d
  -> costmap_inflator
  -> jps_node
  -> minco_planner_node
  -> traj_tracker
  -> /cmd_vel_chassis
```

Nav2 upstream 风格管线不作为默认路径。

## Why this decision

JPS 提供清晰的拓扑搜索结果，MINCO 负责时间参数化和平滑，`traj_tracker` 明确消费 `/planner/traj_samples`。这条链更短，接口更少，更适合定位当前的动力学一致性问题。

## Consequences

- 当前文档和调试都围绕 Stage 3 的四个 active node 展开。
- 不再把 Nav2 参数作为默认调试对象。
- 如果未来需要恢复 Nav2 对比，应作为明确实验任务，而不是从旧文档隐式恢复。

## Revisit condition

如果 JPS + MINCO 无法满足更复杂任务需求，或需要接入 Nav2 生态中的 recovery / behavior tree / global costmap 能力，再重新评估。

## Related modules

- `docs/modules/costmap_inflation.md`
- `docs/modules/jps.md`
- `docs/modules/minco.md`
- `docs/modules/traj_tracker.md`

## Related plan entries

- `docs/current/plan.md`
