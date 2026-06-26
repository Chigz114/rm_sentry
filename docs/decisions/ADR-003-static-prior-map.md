# ADR-003: 将 Gazebo world 静态先验并入 traversability map

## Status

accepted

## Context

仿真场地中的墙体、平台和固定障碍物来自 Gazebo world 文件。仅依赖实时点云会受视角、遮挡、点云稀疏和低矮障碍观测影响。

## Old approach

静态场地结构主要通过实时感知间接进入 costmap，文档中也曾存在静态/动态来源分散的问题。

## Problem with old approach

- 已知静态障碍如果只依赖点云，会造成启动早期或遮挡场景的 costmap 不完整。
- 静态几何来源分散会让调试时难以判断障碍来自 world prior 还是点云证据。
- 对平面 RM 场景，Gazebo world 是可靠的静态先验来源。

## Options considered

- 不使用静态先验，只依赖 `/cloud_registered`。
- 单独维护静态 map 文件。
- 从 launch 传入 Gazebo world 文件，由 `traversability_mapper` 解析并栅格化。

## Decision

`sim_perception.launch.py` 将 RM3V3 world 文件传给 `traversability_mapper`。节点在 `map` frame 中栅格化静态 box，并与动态 log-odds 证据合并生成 `/perception/costmap_2d`。

## Why this decision

它减少了额外资产和同步成本，且让静态障碍与当前仿真 world 保持同源。

## Consequences

- 更改 Gazebo world 文件会影响 costmap 静态先验。
- `traversability_mapper` 的 costmap 不再是纯点云观测结果。
- 调试 false positive 时需要区分静态先验占据和动态点云占据。

## Revisit condition

如果 world 文件不再能代表实际环境，或需要在真车上运行，应把静态先验替换为可配置地图或禁用仿真专用 prior。

## Related modules

- `docs/modules/traversability_map.md`
- `docs/modules/gazebo_chassis.md`

## Related plan entries

- `docs/current/plan.md`
