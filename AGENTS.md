# AGENTS.md

本文件只定义 coding agent 在 `rm_sentry_sim_ws` 中工作的规则。它不是系统架构、运行时 pipeline 或当前任务的真理来源。

当前系统事实看 `docs/current/system_overview.md`。当前任务和推进状态看 `docs/current/plan.md`。启动、停止和健康检查命令看 `docs/current/bringup.md`。

## 必读顺序

在做非平凡修改前先读：

1. `docs/current/system_overview.md`
2. `docs/current/plan.md`
3. 如果任务涉及启动、停止、验证或日志，读 `docs/current/bringup.md`
4. 与任务相关的 `docs/modules/*.md` 或 `docs/decisions/*.md`

不要从旧 launch、旧 package 名、已删除代码路径或历史文档中推断当前 active pipeline。

## 修改前规则

- 先确认目标文件属于当前 active pipeline，或任务明确要求处理 deprecated path。
- 先确认行为由代码默认值、launch 覆盖、yaml、运行时参数还是仿真插件控制。
- 对 bug 不要直接 patch；先列出 hypothesis 和最小验证方法。
- 不要一次跨多个核心模块大改，除非任务明确要求迁移或重构。
- 不要引入机器相关绝对路径到 launch、config 或源码中。
- 不要依赖 `build/`、`install/`、`log/`、rosbag、debug CSV 或 frame graph 这类生成物。

## Deprecated Path 规则

旧 Nav2 upstream、ROG-Map runtime、`pure_pursuit`、`path_tracker`、`goal_controller`、固定 `traj_publisher` 等路径不得作为新工作的默认基础。

如果旧路径看起来相关，先查：

- `docs/current/system_overview.md`
- `docs/decisions/*.md`
- 运行时 launch 和当前源码

只有在用户明确要求历史重建、比较或恢复时，才修改 deprecated path。

## 验证要求

不能只说 build pass 或 RViz 看起来正常。非平凡修改至少说明一种验证：

- targeted build 结果；
- launch 结果；
- topic / frame / QoS 检查；
- 日志或指标；
- module local checks；
- 明确说明未验证及原因。

如果改动影响 ROS 代码、launch 或参数，优先运行对应 package 的 targeted `colcon build --symlink-install --packages-select ...`。纯 Markdown 迁移不需要 build。

## 输出格式

非平凡修改后必须报告：

- changed files；
- changed logic；
- changed parameters；
- affected topics / frames / modules；
- verification performed；
- remaining uncertainty；
- docs that should be updated。

## 文档更新规则

- 运行时拓扑、active module、核心 topic/frame 改变时，更新 `docs/current/system_overview.md`。
- 当前任务、完成项、暂停项或废弃项改变时，更新 `docs/current/plan.md`。
- 启动、停止、日志、健康检查流程改变时，更新 `docs/current/bringup.md`。
- 模块接口、参数计算含义或内部机制改变时，更新对应 `docs/modules/*.md`。
- 架构、方案或长期维护取舍改变时，新增或更新 `docs/decisions/*.md`。
