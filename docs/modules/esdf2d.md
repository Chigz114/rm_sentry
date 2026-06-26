# 2D ESDF

## Code map

| Part | Location | Role |
|---|---|---|
| Launch source | `src/pb_rm_simulation/src/rm_nav_bringup/launch/sim_perception.launch.py` | 启动 `esdf2d` 并覆盖 topic/demo 参数 |
| Node file | `src/sentry_perception/sentry_perception/esdf2d_node.py` | ROS2 node 实现 |
| Input callback | `on_costmap()` | 接收 `OccupancyGrid` 并重算距离场 |
| Distance transform | `scipy.ndimage.distance_transform_edt` | 计算 free cell 到最近 obstacle 的欧氏距离 |
| PointCloud2 construction | `pc2.create_cloud()` | 将 grid cell 转成带 intensity 的点云 |

## Module role

`esdf2d_node` 把 `/perception/costmap_2d` 转为 ESDF 风格点云，主要用于 RViz 可视化和人工检查。

当前 MINCO 不直接消费 `/perception/esdf_2d` 点云；它订阅 `/perception/costmap_2d`，在 `sentry_planner/esdf_map_2d.py` 中维护内部 signed ESDF。

## Interface contract

Input:

- `/perception/costmap_2d`
- `nav_msgs/OccupancyGrid`
- frame 继承 costmap header，当前应为 `map`
- cell 语义：`100=occupied`，`0=free`，`-1=unknown`

Output:

- `/perception/esdf_2d`
- `sensor_msgs/PointCloud2`
- 每个 grid cell 一个点，`x/y` 是 cell 中心世界坐标，`z=0`
- `intensity = distance_to_nearest_obstacle_m + 10.0`

如果 `demo_copy_enable=True`，还会发布 `/perception/esdf_2d_demo` 偏移副本。

## Internal mechanism

1. `on_costmap()` 读取 width、height、resolution 和 origin。
2. 将一维 `OccupancyGrid.data` reshape 为 `(height, width)`。
3. 根据 `treat_unknown_as_obstacle` 构造 obstacle mask。
4. 对 `~obstacle_mask` 调用 `distance_transform_edt`，得到每个 free cell 到最近 obstacle 的 cell 距离。
5. 乘以 resolution 转成米，并用 `max_distance_m` 钳位。
6. 用 `origin + (index + 0.5) * resolution` 把 grid index 转成世界坐标。
7. 打包 `[x, y, z, intensity]` 为 `PointCloud2`。
8. 如启用 demo copy，再复制点云并加 xyz offset 后发布。

## Parameters in computation

| Parameter | Meaning in code | Effect |
|---|---|---|
| `costmap_topic` | 订阅哪个 `OccupancyGrid` | 配错会导致 ESDF 无输入 |
| `esdf_topic` | 主 ESDF 点云发布 topic | RViz/下游显示需要同步 |
| `demo_copy_topic` | demo 副本 topic | 只影响可视化 |
| `demo_copy_enable` | 是否发布偏移副本 | 当前 launch 为 `True` |
| `demo_copy_x/y/z_offset` | demo 点云平移量 | 不改变主 ESDF |
| `treat_unknown_as_obstacle` | unknown cell 是否进入 obstacle mask | `True` 更保守，`False` 更像自由空间可视化 |
| `max_distance_m` | 距离可视化钳位上限 | 影响远离障碍区域的 intensity 饱和 |

## Coupling

Upstream:
如果 `/perception/costmap_2d` 的 origin、resolution 或 frame 错，ESDF 点云也会整体错。

Downstream:
RViz 可用它观察障碍距离场是否连续。不要把它误认为 MINCO 的唯一距离场来源。

## Important implementation details

- `distance_transform_edt` 计算的是二维欧氏距离，不包含 z 结构。
- intensity 加 `10.0` 是历史可视化约定，不代表安全距离。
- 这个节点偏 visualization/debugging；控制闭环强依赖仍是 costmap 和 MINCO 内部 ESDF。

## Local checks

```bash
ros2 topic hz /perception/costmap_2d
ros2 topic hz /perception/esdf_2d
ros2 topic echo /perception/esdf_2d --once
```

RViz 中检查 ESDF 点云是否与 costmap 和场地障碍对齐。

## Personal understanding / open questions

- 我是否理解 `OccupancyGrid` index 到 world coordinate 的转换？
- 是否需要未来把 ESDF 从 visualization 改为共享服务或共享数据结构？
