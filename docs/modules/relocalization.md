# Relocalization

## Code map

| Part | Location | Role |
|---|---|---|
| Launch source | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` | 启动 `relocalization` 并覆盖 seed/ICP 参数 |
| Node file | `src/sentry_perception/sentry_perception/relocalization_node.py` | 发布 `map -> lidar_odom` 和 `/relocalization/status` |
| Status publisher | `_publish_status()` | 输出 seed/ICP 状态 |
| TF publisher | `_publish_tf()` | 持续发布 `map -> lidar_odom` |
| ICP path | `_on_cloud()`, `_run_icp()`, `_icp_2d()` | 累计点云并做 2D ICP 精化 |
| Prior loader | `_load_prior_from_world()` | 从 Gazebo world 生成 2D 先验点云 |

## Module role

`relocalization_node` 把 FAST-LIO 的 `lidar_odom` 局部坐标系对齐到仿真世界的 `map` 坐标系。Stage 2 的 costmap 要在 `map` 中发布，因此这个 TF 是感知侧全局对齐的关键。

## Interface contract

Input:

- `/cloud_registered`：`sensor_msgs/PointCloud2`，在 ICP 启用时作为 source 点云。
- Gazebo `.world` 文件：通过 launch 的 `world_file` 参数传入，作为 target/prior。

Output:

- TF `map -> lidar_odom`：持续发布。
- `/relocalization/status`：`std_msgs/String`，1 Hz 状态文本。

## Internal mechanism

1. 节点读取 `seed_x`、`seed_y`、`seed_yaw`，构造初始 `map <- lidar_odom` 2D 齐次变换。
2. 如果提供 `world_file`，解析 Gazebo world 中的静态几何，生成 `map` frame 下的 2D prior point cloud。
3. 定时发布当前 `map -> lidar_odom` TF。
4. 如果 `refine_with_icp=True`，订阅 `/cloud_registered` 并累计 `accumulate_count` 帧。
5. `_run_icp()` 将累计点云作为 source，把 world prior 作为 target，调用 `_icp_2d()` 估计 refined transform。
6. ICP 结果如果相对 seed 移动过大，会被拒绝；否则更新当前 TF。
7. `/relocalization/status` 报告 `SEED_ONLY` 或 `ICP_DONE` 等状态。

## Parameters in computation

| Parameter | Meaning in code | Effect |
|---|---|---|
| `seed_x`, `seed_y`, `seed_yaw` | 初始 `map <- lidar_odom` 位姿 | 决定 ICP 前的全局对齐，错了会导致 costmap 整体错位 |
| `world_file` | Gazebo world prior 来源 | 空值会禁用 ICP prior |
| `accumulate_count` | ICP 前累计点云帧数 | 更大更稳但启动更慢 |
| `icp_max_iter`, `icp_tol`, `icp_max_dist` | ICP 迭代、收敛和匹配距离 | 控制 ICP 精化范围和计算成本 |
| `voxel_size` | prior/source 降采样尺度 | 影响 ICP 点数和匹配细节 |
| `refine_with_icp` | 是否从 seed 进入 ICP | 关闭后只发布 seed TF |
| `tf_rate_hz` | TF 发布频率 | 过低可能影响下游 transform lookup |
| `cloud_topic` | source 点云 topic | 当前 launch 覆盖为 `/cloud_registered` |

## Coupling

Upstream:
FAST-LIO 点云质量和 Gazebo world prior 会直接影响 ICP。

Downstream:
`traversability_mapper` 查 `map -> lidar_odom` 并在 `map` frame 中生成 costmap。RViz fixed frame 也依赖这个对齐。

## Important implementation details

- 当前 seed 来自 `sim_perception.launch.py`，不是代码默认值。
- ICP 有 sanity check，偏离 seed 过大时会拒绝结果。
- 真车迁移时 Gazebo world prior 不再天然成立。

## Local checks

```bash
ros2 topic echo /relocalization/status --once
ros2 run tf2_tools view_frames
```

RViz 中检查 `/perception/costmap_2d` 是否与 RM3V3 场地对齐。

## Personal understanding / open questions

- 当前 seed 的误差范围是否足够小，让 ICP 稳定收敛？
- 如果 ICP 被拒绝，是否需要在 status 中更结构化地暴露原因？
