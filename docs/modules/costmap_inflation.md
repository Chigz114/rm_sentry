# Costmap Inflation

## Code map

| Part | Location | Role |
|---|---|---|
| Launch source | `src/sentry_planner/launch/sim_planner.launch.py` | 启动 `costmap_inflator` 并设置 `inflation_radius_m=0.30` |
| Node file | `src/sentry_planner/sentry_planner/costmap_inflator.py` | 膨胀 `OccupancyGrid` |
| Input callback | `_on_costmap()` | 接收 raw costmap 并发布 inflated costmap |
| Kernel builder | `_build_kernel()` | 根据 resolution 和 radius 构造圆形 dilation kernel |

## Module role

`costmap_inflator` 把 `/perception/costmap_2d` 转为 JPS 使用的二值膨胀栅格 `/planner/costmap_inflated`。它只服务 JPS 拓扑搜索，不代表最终轨迹安全的全部责任。

## Interface contract

Input:

- `/perception/costmap_2d`
- `nav_msgs/OccupancyGrid`
- `100` 被视作 occupied

Output:

- `/planner/costmap_inflated`
- `nav_msgs/OccupancyGrid`
- 保留原 header/info，data 为膨胀后的 `0/100`

## Internal mechanism

1. `_on_costmap()` 读取 input grid 的 width、height、resolution。
2. 将 data reshape 成二维数组，`data == 100` 为 occupied mask。
3. `_build_kernel()` 按 `inflation_radius_m / resolution` 计算半径 cell 数，并生成圆形布尔 kernel。
4. 使用 binary dilation 扩张 occupied mask。
5. 把结果重新打包为 `OccupancyGrid`，occupied 输出 `100`，其它输出 `0`。

## Parameters in computation

| Parameter | Meaning in code | Effect |
|---|---|---|
| `inflation_radius_m` | 以米为单位的 JPS 二值障碍膨胀半径 | 过大可能堵死窄通道，过小会让 JPS 路径贴墙 |
| `input_topic` | raw costmap topic | 当前为 `/perception/costmap_2d` |
| `output_topic` | inflated costmap topic | 当前为 `/planner/costmap_inflated` |

## Coupling

Upstream:
依赖 traversability map 的 resolution、origin、frame 和 occupied 语义。

Downstream:
JPS 只看 `/planner/costmap_inflated` 的 free/occupied。MINCO 仍消费 raw `/perception/costmap_2d` 构建内部 ESDF，因此 JPS 膨胀不是唯一安全边界。

## Important implementation details

- 当前 launch 覆盖 `inflation_radius_m=0.30`，代码默认值不是有效运行时值。
- 不要只靠增大膨胀半径修复贴墙风险；这会损失拓扑可行通道。

## Local checks

```bash
ros2 topic hz /perception/costmap_2d
ros2 topic hz /planner/costmap_inflated
ros2 topic echo /planner/costmap_inflated --once
```

## Personal understanding / open questions

- 当前 0.30 m 膨胀与机器人 footprint、MINCO `d_hard/d_soft` 的关系是否已量化？
