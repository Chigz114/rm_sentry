# Traversability Map

## Code map

| Part | Location | Role |
|---|---|---|
| Launch source | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` | 启动 `traversability_mapper` 并覆盖参数 |
| Node file | `src/sentry_mapping/src/traversability_mapper_node.cpp` | height-gated 2.5D costmap 实现 |
| Build target | `src/sentry_mapping/CMakeLists.txt` | 编译安装 `traversability_mapper` |
| Static prior parser | `loadStaticPrior()` | 从 Gazebo world 文件解析静态障碍 |
| Point cloud callback | `cloudCallback()` | 将点云投影到 `map` grid 并更新 log-odds |
| Publisher | `publishGrid()` | 发布 `/perception/costmap_2d` |

## Module role

`traversability_mapper` 是当前 Stage 2 的建图主模块。它把 FAST-LIO 点云、FAST-LIO odometry、`map -> lidar_odom` TF 和 Gazebo world 静态先验融合成 2D `OccupancyGrid`。

它替代旧 3D ROG runtime，直接服务 JPS/MINCO 所需的平面可通行性判断。

## Interface contract

Input:

- `/cloud_registered`：`sensor_msgs/PointCloud2`，FAST-LIO 配准点云。
- `/Odometry`：`nav_msgs/Odometry`，FAST-LIO odometry。
- TF `map -> lidar_odom`：来自 `relocalization_node`。
- Gazebo world file：通过 `world_file` 参数传入，作为静态先验。

Output:

- `/perception/costmap_2d`
- `nav_msgs/OccupancyGrid`
- `frame_id=map`
- cell 语义：`100=occupied`，`0=free`

## Internal mechanism

1. 节点启动时读取 resolution、地图尺寸、offset、高度阈值、log-odds 和 topic 参数。
2. 根据 `width_m/height_m/resolution` 计算 grid 宽高，并由 `x_offset/y_offset` 推出 origin。
3. 如果 `world_file` 非空，解析 Gazebo world 中的 box，并把静态障碍 rasterize 到 `static_prior_`。
4. 订阅 `/cloud_registered` 和 `/Odometry`，并周期性发布 costmap。
5. 每次点云回调中查 `map <- lidar_odom` transform，把点从 `lidar_odom` 相关坐标变换到 `map`。
6. 对每个 cell 收集本帧最小 z，并用点云 z 分布估计/夹紧地面高度。
7. 若 cell 中点相对地面高度超过 `h_climb`，按近/中/远命中阈值更新 log-odds。
8. log-odds 按 `decay_tau` 衰减，并被 `log_odds_cap/floor` 限制。
9. 发布时，`static_prior_` 或 `log_odds > occ_thresh` 的 cell 输出为 `100`。

## Parameters in computation

| Parameter | Meaning in code | Effect |
|---|---|---|
| `resolution` | grid cell 尺寸 | 决定 costmap 精度和计算量 |
| `width_m`, `height_m`, `x_offset`, `y_offset` | grid 覆盖范围和 origin | 配错会导致地图裁剪或整体偏移 |
| `h_climb` | 相对地面高度阈值 | 控制低矮障碍是否被判 occupied |
| `ground_clamp_lo/hi`, `ground_init` | 地面高度估计的初值和范围 | 影响高度门控基准 |
| `n_min_near/mid/far`, `near_dist`, `mid_dist` | 不同距离的命中数量阈值 | 控制近远场噪声和灵敏度 |
| `delta_hit` | 命中时 log-odds 增量 | 越大越容易积累为 occupied |
| `decay_tau` | log-odds 时间衰减常数 | 越大障碍保留越久 |
| `occ_thresh` | 占据输出阈值 | 越高越保守地输出 occupied |
| `frame_id`, `odom_frame` | 输出 frame 和 TF lookup 目标 | 当前为 `map` / `lidar_odom` |
| `world_file` | 静态先验来源 | 改 world 会改变静态占据 |

## Coupling

Upstream:
依赖 FAST-LIO 点云质量、`/Odometry`、重定位 TF 和 Gazebo world 文件。

Downstream:
`costmap_inflator`、MINCO 内部 ESDF 和 `esdf2d_node` 都消费 `/perception/costmap_2d`。此模块的 frame/origin 错误会直接污染规划和可视化。

## Important implementation details

- 静态 prior 与动态 log-odds 是 union 关系；静态障碍即使没有当前点云命中也会输出 occupied。
- 这是仿真优先实现，真车迁移时不能直接依赖 Gazebo world prior。
- 当前输出没有 `-1 unknown`，发布时主要是 `0/100`。

## Local checks

```bash
ros2 topic hz /cloud_registered
ros2 topic echo /relocalization/status --once
ros2 topic hz /perception/costmap_2d
ros2 topic echo /perception/costmap_2d --once
```

RViz 中检查 costmap 是否与 RM3V3 场地障碍对齐。

## Personal understanding / open questions

- 当前 `h_climb` 与仿真地面 z 分布是否已经量化？
- 静态 prior 和动态点云占据是否需要在调试输出中分层可视化？
