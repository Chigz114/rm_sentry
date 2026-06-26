# ADR-002: 从 3D ROG runtime 切到 height-gated 2.5D traversability map

## Status

accepted

## Context

当前场景是平地 RoboMaster 仿真导航，核心需求是判断 xy 平面是否可通行，并给 JPS/MINCO 提供稳定的 2D costmap。

## Old approach

旧思路是：

```text
FAST-LIO2 -> 3D ROG runtime -> 2.5D ESDF / costmap
```

## Problem with old approach

- 低矮平台和仿真 MID360 几何会让 3D raycasting 将部分低矮障碍误判为 free。
- 完整 3D ROG 对当前平地 RM 场景过重，调试面也更大。
- 当前规划接口最终需要的是 2D `OccupancyGrid`，而不是完整 3D map。

## Options considered

- 继续调 3D ROG 参数。
- 保留 ROG 并在其后增加补丁式 2D 修正。
- 直接构建 height-gated 2.5D traversability map。

## Decision

当前 Stage 2 使用 `sentry_mapping/traversability_mapper`。它消费 `/cloud_registered`、`/Odometry`、`map -> lidar_odom` TF 和 Gazebo world prior，发布 `/perception/costmap_2d`。

## Why this decision

height-gated 2.5D map 直接服务当前规划需求，能显式处理地面高度、低矮障碍和静态先验，调试路径短于完整 3D ROG。

## Consequences

- 当前 sim workspace 不把 ROG runtime 作为 active pipeline。
- `traversability_mapper_node.cpp` 是建图主入口。
- 系统弱化了复杂 3D 场景泛化能力；如果以后出现坡道、悬空结构或隧道类任务，需要重新评估。

## Revisit condition

如果任务要求完整 3D 结构理解，或 height-gated map 在真实传感器上无法稳定分辨地面和障碍，再重新比较 ROG 或其它 3D mapping 方案。

## Related modules

- `docs/modules/traversability_map.md`
- `docs/modules/relocalization.md`
- `docs/modules/esdf2d.md`

## Related plan entries

- `docs/current/plan.md`
