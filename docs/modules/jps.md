# JPS

## Code map

| Part | Location | Role |
|---|---|---|
| Launch source | `src/sentry_planner/launch/sim_planner.launch.py` | 启动 `jps_node` 并覆盖 costmap/odom 参数 |
| Node file | `src/sentry_planner/sentry_planner/jps_node.py` | JPS 搜索 ROS node |
| Costmap callback | `_on_costmap()` | 缓存 inflated costmap 和 frame |
| Odom callback | `_on_odom()` | 更新当前机器人位置 |
| Goal callback | `_on_goal()` | 接收目标并触发规划 |
| Search | `_jps_search()`, `_jump()`, `_pruned_neighbors()` | JPS 栅格搜索 |
| Publishing | `_publish_path()`, `_publish_viz()` | 发布 `Path` 和 marker |

## Module role

`jps_node` 在 `/planner/costmap_inflated` 上搜索从当前位姿到 `/goal_pose` 的拓扑路径，输出 `/planner/path` 给 MINCO。它不是最终轨迹优化器。

## Interface contract

Input:

- `/planner/costmap_inflated`：`nav_msgs/OccupancyGrid`，JPS 二值搜索地图。
- `/goal_pose`：`geometry_msgs/PoseStamped`，目标点。
- `/odom`：`nav_msgs/Odometry`，当前仿真位姿源。

Output:

- `/planner/path`：`nav_msgs/Path`，JPS 路点路径。
- `/planner/jps_viz`：`visualization_msgs/MarkerArray`，RViz marker。

## Internal mechanism

1. `_on_costmap()` 缓存 inflated grid、resolution、origin 和 frame。
2. `_on_odom()` 更新当前机器人位置；当前 launch 设置 `use_raw_odom=True`，直接使用 `/odom` pose。
3. `_on_goal()` 保存目标并调用 `_plan_and_publish()`。
4. `_world_to_grid()` 将 robot/goal 坐标转为 grid index。
5. `_jps_search()` 使用 open set、heuristic、jump point 和 pruned neighbors 在二值栅格上搜索。
6. `_simplify_path()` 和 line-of-sight 逻辑减少冗余点。
7. `_grid_to_world()` 将路径转回世界坐标，发布 `/planner/path` 和 marker。

## Parameters in computation

| Parameter | Meaning in code | Effect |
|---|---|---|
| `costmap_topic` | JPS 输入栅格 | 当前为 `/planner/costmap_inflated` |
| `goal_topic` | 目标 topic | 当前为 `/goal_pose` |
| `odom_topic` | 当前位姿 topic | 当前为 `/odom` |
| `odom_frame` | 非 raw odom 模式下的 TF frame | 当前 raw 模式基本绕过 |
| `use_raw_odom` | 是否直接用 odom pose | 当前为 `True`，避免 FAST-LIO 漂移影响 Stage 3 |
| `path_topic` | 输出 path topic | 当前为 `/planner/path` |
| `max_iter` | 搜索迭代预算 | 太小可能搜索失败 |
| `goal_tolerance_m` | 目标接受容差 | 决定接近目标的成功范围 |

## Coupling

Upstream:
依赖 costmap inflation 的 free/occupied 结果和 `/odom` 与 costmap frame 的一致性。

Downstream:
MINCO 接收 `/planner/path`，再做路点后处理、ESDF 间距和轨迹生成。JPS 输出贴墙不一定是最终轨迹，但会强烈影响 MINCO 初值和拓扑。

## Important implementation details

- 当前 JPS 不计算连续距离代价，只在二值 grid 中搜索。
- `use_raw_odom=True` 是仿真专用简化，真车不应直接照搬。
- 如果 JPS 输出空 path，MINCO 没有有效轨迹可优化。

## Local checks

```bash
ros2 topic hz /planner/costmap_inflated
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 9.0, y: 6.0, z: 0.0}, orientation: {w: 1.0}}}'
ros2 topic echo /planner/path --once
```

## Personal understanding / open questions

- JPS path 的 line-of-sight 简化是否会在窄通道中制造过长直线段？
- `goal_tolerance_m` 与 MINCO/track 的最终目标容差是否需要统一？
