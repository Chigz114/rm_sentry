# Modules

本目录是 active pipeline 的模块理解笔记，不是自动生成 reference。当前系统事实见 `docs/current/system_overview.md`。

每个模块文档统一回答：

- Code map：核心文件、类、函数、callback、launch 参数来源；
- Module role：模块在 pipeline 中负责哪段数据变换；
- Interface contract：输入输出 topic/message/frame/语义；
- Internal mechanism：按代码执行顺序解释数据如何被处理；
- Parameters in computation：参数在计算中的意义；
- Coupling：上下游如何影响它；
- Important implementation details：仿真专用、真车可迁移、visualization 等边界；
- Local checks：本模块最小检查；
- Personal understanding / open questions：留给人继续补充。

## Active module docs

| 文档 | 模块 |
|---|---|
| `fastlio2.md` | Stage 1 Gazebo + FAST-LIO2 |
| `relocalization.md` | `map -> lidar_odom` 重定位 |
| `traversability_map.md` | height-gated 2.5D 可通行性图和静态先验 |
| `esdf2d.md` | 2D ESDF 可视化点云 |
| `costmap_inflation.md` | JPS 输入 costmap 膨胀 |
| `jps.md` | JPS 栅格搜索 |
| `minco.md` | JPS path 到 MINCO 时间参数化轨迹 |
| `traj_tracker.md` | `/planner/traj_samples` 轨迹跟踪 |
| `gazebo_chassis.md` | Gazebo mecanum 执行路径 |
