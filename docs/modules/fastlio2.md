# FAST-LIO2

## Code map

| Part | Location | Role |
|---|---|---|
| Stage 1 launch | `src/pb_rm_simulation/src/rm_nav_bringup/launch/bringup_sim.launch.py` | 启动 Gazebo、IMU 滤波和 `fast_lio/fastlio_mapping` |
| FAST-LIO config | `src/pb_rm_simulation/src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml` | 设置 lidar/IMU topic、Livox 模式和 dense publish |
| Robot xacro | `src/pb_rm_simulation/src/rm_nav_bringup/urdf/sentry_robot_sim.xacro` | 定义 MID360/IMU 与底盘仿真 |

## Module role

FAST-LIO2 是 Stage 1 的状态估计和点云配准模块。它把仿真 MID360 lidar 和 IMU 转为 `/cloud_registered` 与 `/Odometry`，供 Stage 2 重定位和可通行性建图使用。

当前 Stage 3 不直接使用 FAST-LIO odometry；规划和控制使用 Gazebo `/odom`。

## Interface contract

Input:

- `/livox/lidar`：Gazebo Livox 插件输出。
- `/livox/imu` 经 `imu_complementary_filter` remap 为 `/imu/data` 后供 FAST-LIO 使用。

Output:

- `/cloud_registered`：`sensor_msgs/PointCloud2`，配准点云，Stage 2 直接消费。
- `/Odometry`：`nav_msgs/Odometry`，FAST-LIO odometry，`traversability_mapper` 使用。
- `/odom`：Gazebo/chassis odometry，不是 FAST-LIO 输出，但由同一 Stage 1 仿真启动链提供给 Stage 3。

## Internal mechanism

`bringup_sim.launch.py` 先 include `rm_simulation.launch.py` 启动 Gazebo 世界和机器人，再启动 `imu_complementary_filter`，最后在 `lio:=fastlio` 条件下启动 `fastlio_mapping`。

FAST-LIO 使用 yaml 中的 Livox/MID360 配置融合 lidar 和 IMU，发布稠密配准点云。Stage 2 假设 `/cloud_registered` 足够稠密，能支撑 height-gated map 的单帧 z 统计和命中更新。

## Parameters in computation

| Parameter | Meaning in code/config | Effect |
|---|---|---|
| `common.lid_topic` | FAST-LIO lidar 输入 topic，当前为 `/livox/lidar` | 配错会导致无点云输入 |
| `common.imu_topic` | FAST-LIO IMU 输入 topic，当前为 `/imu/data` | 配错会导致 LIO 初始化/融合失败 |
| `preprocess.lidar_type` | Livox 类型配置，当前为 `1` | 影响点云预处理方式 |
| `preprocess.blind` | 近场盲区距离 | 过大可能丢近场障碍，过小可能引入近场噪声 |
| `publish.dense_publish_en` | 是否发布稠密点云，当前为 `true` | 关闭会削弱 traversability map 输入 |

## Coupling

Upstream:
Gazebo lidar/IMU topic、xacro 外参和 IMU filter 输出必须与 FAST-LIO yaml 对齐。

Downstream:
`relocalization_node` 和 `traversability_mapper` 都依赖 `/cloud_registered`。`traversability_mapper` 还消费 `/Odometry`。

## Important implementation details

- `bringup_sim.launch.py` 中 `nav_rviz` 和 `mode` 是兼容参数；当前轻量 Stage 1 不启动 Nav2。
- Stage 3 使用 `/odom` 是有意设计，不应把 FAST-LIO 漂移直接当成控制器问题。

## Local checks

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
```

## Personal understanding / open questions

- FAST-LIO 在当前仿真中的漂移是否只影响 Stage 2 建图，而不会直接污染 Stage 3 控制？
- `dense_publish_en` 对可通行性图的点密度影响是否需要量化？
