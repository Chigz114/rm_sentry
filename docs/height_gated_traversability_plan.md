# Height-Gated 2.5D Traversability Map 实施计划

> 日期：2026-06-22
> 状态：阶段 1、阶段 2 已实现并验证；阶段 3.0-3.4 已实现并验证（3.4 MINCO 用 Python 实现，偏离原 C++/GCOPTER 方案）
> 更新：2026-06-22，补全 3.4 MINCO 实现状态、参数偏离记录、调试历程与已知问题
> 清理说明：2026-06-25 后，`goal_controller.py`、`path_tracker.py`、`pure_pursuit.py`、`traj_publisher.py` 以及旧 ROG 文件 `perception_mapper_node.cpp`、`rog_map_sentry.yaml`、`mapping.launch.py`、`replay_mapping.launch.py`、`mapping.rviz` 均已从当前 workspace 移除；下文相关内容仅作为历史演化记录。

## 背景

### 问题
LiDAR 提至 0.4m 后，0.2m 高台障碍物被 ROG-Map raycasting 清除为 free。
根因：LiDAR 高于高台顶面，射线掠过高台顶面去打后方高墙时，
穿过高台体素将其标记为 free，概率上 free 压倒 occupied。

### 决策
放弃完整 3D ROG-Map，改用 height-gated 2.5D traversability map。
理由（用户确认）：
- 场地只有 0.2m / 0.4m 不可跨越障碍 + 敌方机器人（>0.3m），无需 3D 语义
- height-gate 单阈值（h_climb≈0.1m）即可区分障碍与地面
- 场地平整，全局 z_g 风险低；车体小倾角由局部地面估计吸收
- 避障只需面朝我方半面，短衰减（0.5~1s）即可处理动态障碍
- 先验场地地图给出 free/occupied 二值基线，不维护 unknown

## 当前代码现状

### 数据流（已实现）
```
/cloud_registered + /Odometry
  → traversability_mapper: height-gate + log-odds + 衰减
  → /perception/costmap_2d (OccupancyGrid)
  → esdf2d_node: distance_transform_edt
  → /perception/esdf_2d (PointCloud2)
```

### 关键文件
| 文件 | 作用 | 改动 |
|------|------|------|
| `sentry_mapping/src/traversability_mapper_node.cpp` | **新建**：height-gate + log-odds 节点 | ✅ 已实现 |
| `sentry_mapping/src/perception_mapper_node.cpp` | 继承 ROGMapROS + 2.5D 投影 | 已删除；仅保留历史记录 |
| `sentry_mapping/CMakeLists.txt` | 构建配置 | ✅ 新增 traversability_mapper 目标 |
| `sentry_perception/sentry_perception/esdf2d_node.py` | OccupancyGrid → ESDF | **不改** |
| `rm_nav_bringup/launch/sim_perception.launch.py` | 感知启动 | ✅ 换节点 + 参数 |
| `rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml` | FAST-LIO2 仿真配置 | ✅ dense publish + 低滤波 + 倒置外参 |
| `rm_nav_bringup/config/simulation/measurement_params_sim.yaml` | LiDAR 安装位姿 | ✅ rpy 改为 π 0 0（倒置） |
| `rm_nav_bringup/config/simulation/segmentation_sim.yaml` | 地面分割配置 | ✅ gravity_aligned_frame 设为 base_link |
| `rm_nav_bringup/launch/bringup_sim.launch.py` | 仿真启动 | ✅ laserscan height 翻转 |
| `rm_simulation/.../mid360.xacro` | LiDAR 仿真插件 | ✅ samples 30000（原 30000，未改） |
| `rm_nav_bringup/config/simulation/rog_map_sim.yaml` | ROG-Map 配置 | 已删除；sim_ws 不再保留 ROG 运行配置 |
| `data/generated_worlds/rm3v3_sym_v1.world` | Gazebo 场地定义（box 障碍物） | 阶段 2 解析生成先验栅格 |

### 关键约束
- `sentry_mapping` / `sentry_perception` / `rog_map` 曾在 sim_ws 中作为指向 real_ws 的**符号链接**；当前 `sentry_mapping`、`sentry_perception` 已是本地包，`rog_map` symlink 已删除
- `/cloud_registered` 帧名：仿真 `lidar_odom`，真车 `camera_init`
- costmap 当前固定全场（13×9m @ 0.1m，frame=lidar_odom，follow_robot=false）
- esdf2d QoS：sub=BEST_EFFORT，pub=RELIABLE，兼容沿用

## 目标架构

```
静态先验层 (RMUC.pgm)  ──┐
                          ├─ O = static ∨ dynamic ─→ OccupancyGrid ─→ esdf2d (不变) ─→ /perception/esdf_2d
/cloud_registered ─→ height-gated 动态层 (local-ground + log-odds + 短衰减) ──┘
```

## 阶段 1：height-gated 动态层（解决 0.2m 漏检 bug，不依赖先验/定位）

### 新建节点 `traversability_mapper`

放在 `sentry_mapping` 包，**不继承 ROGMapROS**，直接处理点云。

#### 1.1 订阅
- `/cloud_registered` (PointCloud2)
- `/Odometry` (Odometry)

#### 1.2 固定全场网格 ✅
- 复用现有参数：13×9m @ 0.1m，frame=lidar_odom
- 每个 cell 维护：
  - `log_odds`（动态障碍置信，float）
  - `hit_counts`（本帧命中计数，int，每帧清零）
  - `frame_min_z`（本帧最低 Z，float，每帧清零）
- 全局变量：
  - `global_ground_z`（全局地面估计，float，EMA 更新）

#### 1.3 全局地面估计 ✅（偏离原方案）

**原方案**：per-cell running-min，每个 cell 独立维护 `ground_z`。
**问题**：0.2m 高台表面的点（z≈-0.2）成为该 cell 的"最低点"，running-min 把高台表面当作地面。于是 h = z - ground_z = 0，高台不被判为障碍。这是 0.2m 障碍漏检的根因。

**实际方案**：全局地面估计，所有点取 10th percentile Z + EMA 平滑。
- 每帧收集所有有效点的 Z 值，用 `std::nth_element` 取 p10（O(n)）
- EMA 更新：`global_ground_z = α * p10 + (1-α) * global_ground_z`，α=0.1
- 钳位到 [-0.55, -0.25]
- p10 过滤了噪声离群点（比绝对 min 稳健），EMA 防止单帧抖动
- 高台表面 z≈-0.2，全局地面 z≈-0.4，h≈0.2 > h_climb=0.1，正确检测

**参数偏离**：
- `ground_clamp_lo`: -0.35 → **-0.55**（LiDAR 倒置后地面约 -0.4，原下限 -0.35 会 clip 掉真实地面）
- `ground_clamp_hi`: -0.15 → **-0.25**（同理，上限需覆盖 -0.4 附近）

#### 1.4 Height-gate ✅
- 点高度 `h = z - global_ground_z`
- `h > h_climb`（0.1m）记一次命中
- 0.2m 障碍 h≈0.2 稳过阈值

#### 1.5 距离归一化判障 ✅（偏离原方案）

**原方案**：近 N_min=4，中 N_min=2，远 N_min=1。
**问题**：仿真 LiDAR 仅 30000 点/帧，覆盖 360°×62° 视场。落到 2m 外一个 2m×5m 高台上的点只有几百个，平均每 0.1m cell 不到 1 个点。N_min=2 或 4 时绝大多数 cell 无法过阈值，导致远端障碍大面积漏检。

**实际方案**：全部 N_min=1。
- log-odds 累积 + 衰减机制本身已能过滤噪声，不需要 per-frame 多点验证
- 单次 hit 加 delta_hit=1.0，需多帧持续 hit 才能超过 occ_thresh=0.5（因为衰减在持续扣减）
- 噪声 hit 会被衰减快速清除，不会形成持久假障碍

#### 1.6 短时衰减 ✅（偏离原方案）

**原方案**：τ=0.7s，用 sim clock 计算 dt。
**问题 1**：τ=0.7 时每次 timer tick（0.1s）衰减 0.1/0.7=0.143。但 cloud 频率仅 ~5 Hz（0.2s/帧），两次 cloud 之间衰减 0.286。单次 hit 只加 delta_hit=0.4，净增仅 0.114。大多数 cell 不是每帧都被打到，漏一帧就净减。**decay 远快于 hit 累积，所有障碍被衰减掉**。

**问题 2**：wall timer（10Hz）用 `get_clock()->now()`（sim clock）计算 dt。Gazebo 仿真慢时两次 tick之间 sim time 不推进，dt=0，decay=0。当 sim time 突然跳大步时 dt 暴大，造成间歇性全红或全清。

**实际方案**：
- τ=0.7 → **5.0**（每次 tick 衰减仅 0.02，hit 累积 1.0 远大于衰减）
- delta_hit=0.4 → **1.0**（单次 hit 即可推动 log_odds 过阈值）
- 用 **std::chrono::steady_clock** 替代 sim clock 计算 dt，确保 decay 时间差稳定
- 带下限 log_odds_floor=-2.0，防穿负

#### 1.7 输出 ✅
- `log_odds > occ_thresh` → 100 (OCCUPIED)
- 否则 → 0 (FREE)
- 发 `/perception/costmap_2d` (OccupancyGrid)
- **无 unknown 状态**

### 构建改动

#### CMakeLists.txt ✅
- 新增 `traversability_mapper` 可执行目标
- 不需要 rog_map / yaml-cpp
- 只需 rclcpp / sensor_msgs / nav_msgs / pcl_conversions / Eigen3

#### sim_perception.launch.py ✅
- 第 1 个 Node 从 `perception_mapper` 换成 `traversability_mapper`
- 实际参数：
  - `h_climb`: 0.10
  - `n_min_near`: 1, `n_min_mid`: 1, `n_min_far`: 1
  - `delta_hit`: 1.0
  - `decay_tau`: 5.0
  - `occ_thresh`: 0.5
  - `ground_clamp_lo`: -0.55, `ground_clamp_hi`: -0.25
  - `ground_init`: -0.40
- ROG-Map 不再启动

#### esdf2d_node.py ✅
- **不改**，继续吃 OccupancyGrid

### 阶段 1 验证 ✅（已完成）
- ✅ 高台 occupied 检测到（~1200 cells，稳定不衰减）
- ✅ 不再出现全红/全清间歇性 bug
- ✅ 静止状态下 costmap 稳定输出
- ✅ keyboard_teleop 绕场跑：高台在机器人移动/远离后保持稳定，不消失
- ✅ 放移动障碍：移走后拖尾在 τ 内清空

### 额外改动（Plan 外：LiDAR 倒置）

为改善近场地面和低矮障碍的检测，将 LiDAR 从正装改为倒置安装（rpy=π 0 0），俯角从 7° 增到 55°。涉及以下连锁修改：

| 文件 | 改动 |
|------|------|
| `measurement_params_sim.yaml` | rpy: `0 0 0` → `π 0 0` |
| `fastlio_mid360_sim.yaml` | `extrinsic_R`: I → `diag(1,-1,-1)`；`blind`: 0.1 → 0.2；`dense_publish_en`: false → true；`point_filter_num`: 3 → 1；`filter_size_surf`: 0.5 → 0.2 |
| `bringup_sim.launch.py` | laserscan `min_height/max_height`: `-1.0/0.1` → `-0.1/1.0`（Z 轴反向） |
| `segmentation_sim.yaml` | `gravity_aligned_frame`: `""` → `"base_link"`（在重力对齐系中做地面分割） |

## 阶段 2：静态先验融合 + 全局定位 ✅（已实现并验证）

> 偏离原方案：原计划用 Nav2 `RMUC.pgm/.yaml` 格式先验 + 真车初始定位接口；
> 实际改为**直接解析 Gazebo world 文件**生成先验 + **Known Start Zone Seed + ICP Refinement** 全局定位。

### 2.1 加载先验 ✅
- 从 Gazebo world 文件 `rm3v3_sym_v1.world` 解析静态障碍物（全部为 box，含 pose + size）
- 用 `tinyxml2`（C++ 端 `traversability_mapper`）/ `xml.etree.ElementTree`（Python 端 `relocalization_node`）解析 SDF XML
- 按名称过滤非障碍物模型（`spawn` / `control` / `light` / `floor`）
- 实测解析出 **8 个障碍物**：
  - 4 面墙（`wall_left/right/top/bottom`，0.2m 厚）
  - 2 个高台（`highland_a/b`，2×5m）
  - 2 个高台唇沿（`highland_lip_a/b`，0.1×1.8m）
- 启动时一次性栅格化到 `static_prior_`（`vector<bool>`，与动态层同分辨率 0.1m、同尺寸 13×9m）
- 实测标记 **3247 cells** 为占据（约占全图 11700 cells 的 27.8%）
- ~~`RMUC.pgm/.yaml`~~：不使用 Nav2 格式先验，直接从 world 文件生成

### 2.2 全局定位：Known Start Zone Seed + ICP Refinement ✅（新实现）

> **替代原方案**的 spawn 位姿固定变换。原方案在 `loadStaticPrior()` 中用 `spawn_x/y/yaw` 做固定坐标变换，
> 现改为 **TF 查找 `map → lidar_odom`** + **relocalization_node** 发布该 TF。

#### 架构

```
FAST-LIO2 → /Odometry (lidar_odom → base_link)
                          ↑ 消费者通过 TF 查找 map → lidar_odom
relocalization_node → TF broadcast (map → lidar_odom)
    ├── seed pose (已知出生区域)
    ├── 累积 /cloud_registered 点云 (30 帧 ≈ 3s)
    ├── 2D ICP refinement (source: lidar_odom 点云, target: prior map 点云)
    └── 发布修正后的 map → lidar_odom TF
```

#### TF 链

```
map → lidar_odom → base_link
         ↑              ↑
    relocalization_node  FAST-LIO2
```

- `relocalization_node` 发布 `map → lidar_odom`（3x3 2D 齐次变换矩阵）
- FAST-LIO2 发布 `lidar_odom → base_link`（里程计）
- `traversability_mapper` 和 `esdf2d_node` 的 costmap/ESDF 输出 frame = `map`

#### relocalization_node 实现

| 组件 | 说明 |
|------|------|
| seed pose | launch 参数 `seed_x/y/yaw`，给已知出生区域的粗略位姿 |
| 点云累积 | 订阅 `/cloud_registered`（QoS: **RELIABLE**），累积 `accumulate_count` 帧后触发 ICP |
| Z 轴过滤 | 只保留 `z > 0.1m` 的点（障碍物），过滤地面点 |
| 2D ICP | `scipy.spatial.cKDTree` 做 NN 查找 + `_umeyama_2d` 求最优刚体变换，3x3 齐次矩阵 |
| 拒绝机制 | 若 ICP 结果离 seed 超过 2m 或 RMSE 过高，回退到 seed |
| 状态监控 | 发布 `/relocalization/status`（String），报告 `ACCUMULATING` / `ICP_DONE` |
| prior 点云 | 从 world file 解析障碍物 box，**填充内部**（非仅轮廓），3468 个点 |

#### 关键参数（sim_perception.launch.py）

```python
seed_x: 5.5          # 故意偏移 0.5m（真值 6.0）
seed_y: 3.5          # 故意偏移 0.5m（真值 4.0）
seed_yaw: 0.15       # 故意偏移 8.6°（真值 0.0）
accumulate_count: 30  # 3s @10Hz
icp_max_dist: 2.0    # m, 匹配距离上限
icp_max_iter: 50
icp_tol: 1e-4
voxel_size: 0.1
```

#### 验证结果

- seed: `(5.500, 3.500, 8.6°)` → ICP 修正后: `(6.126, 4.022, 0.5°)` ≈ 真值 `(6.0, 4.0, 0°)`
- ICP: 24 次迭代, RMSE = 0.082m, translation delta = 0.77m, yaw delta = 9.25°
- 误差: ~13cm, ~0.5°

### 2.3 融合 ✅（更新：map frame）
- `traversability_mapper` 的 `frame_id` 从 `lidar_odom` 改为 `map`
- `loadStaticPrior()` 不再做坐标变换，prior 直接在 map (world) 坐标系
- `cloudCallback` / `odomCallback` 通过 TF 查找 `map → lidar_odom` 变换点云和里程计
- 输出时：`grid.data[idx] = (static_prior_[idx] || log_odds_[idx] > occ_thresh_) ? 100 : 0`
- 静态层**永不衰减** → 远端固定障碍不会被衰减掉
- 仍由 esdf2d 每帧整图重算 ESDF（<1ms）
- 无 unknown 状态（先验给出 free 基线）

### 构建改动 ✅（更新）
| 文件 | 改动 |
|------|------|
| `sentry_mapping/src/traversability_mapper_node.cpp` | `loadStaticPrior()` 不再做 spawn 变换，prior 直接在 map 坐标系；`cloudCallback`/`odomCallback` 改用 TF 查找 `map → lidar_odom`；`frame_id` 改为 `map`；修复 `odomCallback`/`cloudCallback` 死锁（见 §2.5 bug 修复） |
| `sentry_mapping/CMakeLists.txt` | `find_package(tinyxml2_vendor)` + `find_package(TinyXML2)`，链接 `tinyxml2::tinyxml2`，include `${TINYXML2_INCLUDE_DIR}` |
| `sentry_mapping/package.xml` | 新增 `<depend>tinyxml2_vendor</depend>` |
| `sentry_perception/sentry_perception/relocalization_node.py` | **新建**：seed + ICP 全局定位节点，发布 `map → lidar_odom` TF + `/relocalization/status` |
| `sentry_perception/setup.py` | 新增 `relocalization_node` entry point |
| `rm_nav_bringup/launch/sim_perception.launch.py` | 新增 `relocalization_node` 节点 + 参数；`traversability_mapper` 参数改为 `frame_id=map`，grid offset 改为场地中心 `(6.0, 4.0)`，ground_clamp 修正 |

### 阶段 2 验证 ✅（更新）
- ✅ 解析 8 个障碍物，3247 cells 标记占据
- ✅ costmap 稳定 10.0 Hz 输出，frame = `map`
- ✅ ESDF 稳定 20 Hz 输出，frame = `map`
- ✅ ICP 从 seed `(5.5, 3.5, 8.6°)` 修正到 `(6.126, 4.022, 0.5°)`，RMSE = 0.082m
- ✅ RViz 中先验栅格覆盖墙体/高台位置，与点云对齐
- ✅ 机器人周围无虚假占据（ground_clamp 修正后）

### 2.4 遇到的问题与修复

#### Bug 1: ICP 矩阵维度不匹配（4x4 → 3x3）

**现象**：`relocalization_node` 崩溃 `ValueError: matmul: Input operand 1 has a mismatch in its core dimension 0`。

**根因**：ICP 代码混合使用 4x4 齐次矩阵和 2D 点 (N,2)，`src_h = (N,3)` 与 4x4 矩阵相乘维度不匹配。

**修复**：全部改为 3x3 2D 齐次矩阵。涉及 `_seed_to_matrix`、`_umeyama_2d`、`_icp_2d`、`_publish_tf`、`_publish_status` 中的矩阵索引从 `[0,3]`/`[1,3]` 改为 `[0,2]`/`[1,2]`。

#### Bug 2: ICP 收敛到错误解（delta=6.28m, RMSE=3.45m）

**现象**：ICP 运行了但结果离 seed 6.28m，被拒绝机制回退到 seed。

**根因**：`_icp_2d` 迭代中 `umeyama` 的输入是**已变换的 source 点** `src_f = src_t[mask]`，算出的是增量变换，但代码用 `T = T_new` 直接替换（而非复合 `T = T_inc @ T`），导致每轮 T 被重置为小增量变换，source 被拉回原点附近，ICP 彻底跑飞。

**修复**：`umeyama` 输入改为**原始 source 点** `source[mask]`（而非已变换的 `src_t[mask]`），correspondences 仍用变换后的点查找。这样 `T_new` 是完整的 lidar_odom→map 变换，`T = T_new` 替换语义正确。

#### Bug 3: QoS 不匹配导致点云收不到

**现象**：`ACCUMULATING: 0/50` 卡住，`/cloud_registered` 有数据但 relocalization_node 收不到。

**根因**：FAST-LIO2 的 `/cloud_registered` publisher 是 `RELIABLE`，relocalization_node 订阅用 `BEST_EFFORT`。ROS2 中 RELIABLE publisher 不会向 BEST_EFFORT subscriber 发送数据。

**修复**：relocalization_node 的 cloud subscription QoS 改为 `RELIABLE`。

#### Bug 4: odomCallback / cloudCallback 死锁

**现象**：costmap 不输出，`have_odom_` 始终为 false。

**根因**：`odomCallback` 开头 `if (!tf_ready_) return;`，`cloudCallback` 开头 `if (!have_odom_) return;`，而 `tf_ready_` 只在 `cloudCallback` 中设置，`have_odom_` 只在 `odomCallback` 中设置 → 互相等待死锁。

**修复**：`odomCallback` 去掉 `tf_ready_` 前置检查，直接尝试 TF 查找，成功时同时设置 `have_odom_ = true` 和 `tf_ready_ = true`。`cloudCallback` 去掉 `have_odom_` 前置检查。

#### Bug 5: 地面钳位范围不匹配导致机器人周围虚假占据

**现象**：机器人周围出现大量 occupied 区域（~2000 个虚假格子），不被 τ 衰减机制清除。

**根因**：plan 文档假设地面在 `-0.4`，钳位范围 `[-0.55, -0.25]`。但实际 `/cloud_registered` 的地面在 `-0.14`，`ground_clamp_hi=-0.25` 把 `ground_z` 强行压到 -0.25（比真实地面低 0.11m）。导致平整地面 `h = -0.14-(-0.25) = 0.11 > h_climb(0.1)` 被误判为障碍。这些误检每帧重新命中，所以不被 τ 衰减。

**修复**：钳位范围改为 `[-0.30, -0.08]`，`ground_init` 改为 `-0.14`。修正后 `ground_z` 正确追踪到 -0.142，虚假占据从 2006 降到 556，机器人 1m 内从 283 降到 0。

#### Bug 6: grid offset 未随 frame 切换更新

**现象**：costmap 切到 `map` frame 后只显示场地左下角四分之一。

**根因**：grid 的 `x_offset/y_offset` 原为 0（以 lidar_odom 原点为中心），切到 map frame 后场地中心在 `(6.0, 4.0)`，grid 覆盖范围 `(-6.5, -4.5)~(6.5, 4.5)` 只包含场地左下部分。

**修复**：`x_offset` 改为 6.0，`y_offset` 改为 4.0，grid 覆盖范围变为 `(-0.5, -0.5)~(12.5, 8.5)`，完整覆盖场地。

### 额外修复（Plan 外：FAST-LIO2 blind 优先级 bug）

调 `blind: 0.2 → 0.5` 过滤车体自身误检时，发现 `/cloud_registered` 仍有 < 0.5m 的点。

**根因**：开源 FAST-LIO2 原版 `preprocess.cpp` 的 AVIA 分支（`avia_handler`）存在 C++ 运算符优先级 bug：

```cpp
// 原版（错误）：&& 优先级高于 ||，blind 检查被旁路
if ((abs(dx) > 1e-7) || (abs(dy) > 1e-7) || (abs(dz) > 1e-7)
    && (dist² > blind²))
// 实际解析为 dx>ε || dy>ε || (dz>ε && dist²>blind²)
// 只要 dx 或 dy 有变化（几乎总成立）就保留点，blind 完全失效
```

**修复**：给前三个 `||` 条件加括号，使 `blind` 成为独立的与条件：

```cpp
if (((abs(dx) > 1e-7) || (abs(dy) > 1e-7) || (abs(dz) > 1e-7))
    && (dist² > blind²))
```

| 文件 | 改动 |
|------|------|
| `rm_localization/FAST_LIO/src/preprocess.cpp` | `avia_handler` blind 条件加括号修复优先级 |
| `fastlio_mid360_sim.yaml` | `blind`: 0.2 → **0.5**（过滤车体自身） |

注：该 bug 仅存在于 AVIA（Livox CustomMsg）路径；Velodyne/Ouster 路径的 blind 检查是独立 `if (range < blind²) continue;`，不受影响。仿真用 `lidar_type=1`（AVIA）故命中此 bug。**真车同样走 AVIA 路径，此修复对真车同样必要。**

## 实际参数（已验证，更新）

```yaml
# 网格
grid_resolution: 0.1
grid_width_m: 13.0
grid_height_m: 9.0
frame_id: map           # 改：从 lidar_odom 改为 map（全局定位后）
x_offset: 6.0           # 改：grid 中心对齐场地中心
y_offset: 4.0           # 改：grid 中心对齐场地中心
follow_robot: false

# height-gate
h_climb: 0.10          # m, 离地高度阈值

# 地面估计（修正：钳位范围匹配实际 LiDAR 地面高度）
ground_clamp_lo: -0.30  # 改：原 -0.55，实际地面在 -0.14
ground_clamp_hi: -0.08  # 改：原 -0.25，原值导致地面被误判为障碍
ground_init: -0.14      # 改：原 -0.40，实际 /cloud_registered 地面 p50≈-0.11

# 距离归一化（偏离：全部降为 1）
n_min_near: 1           # 原值 4，仿真点云密度不足
n_min_mid: 1            # 原值 2
n_min_far: 1            # 原值 1

# log-odds（偏离：delta_hit 和 decay_tau 大幅调整）
delta_hit: 1.0          # 原值 0.4，需大于衰减速率才能累积
log_odds_cap: 2.0       # 上限
decay_tau: 3.0          # s, 原值 0.7，原值衰减太快导致障碍全被清除
occ_thresh: 0.5         # 判障阈值

# relocalization_node
seed_x: 5.5            # 故意偏移 0.5m（真值 6.0）
seed_y: 3.5            # 故意偏移 0.5m（真值 4.0）
seed_yaw: 0.15         # 故意偏移 8.6°（真值 0.0）
accumulate_count: 30    # 3s @10Hz
icp_max_dist: 2.0      # m
icp_max_iter: 50
icp_tol: 1e-4
voxel_size: 0.1
cloud_topic: /cloud_registered  # QoS: RELIABLE（匹配 FAST-LIO2 publisher）
```

## 注意事项（按风险排序，已更新）

1. **符号链接连带**：改 sentry_mapping/sentry_perception 会同时改真车 ws。新逻辑对真车也是想要的，但若行为需差异化，用参数区分而非硬编码。

2. **坐标系一致性**：~~动态层在 lidar_odom，先验在场地系。阶段 2 融合前必须解决对齐。~~ 已通过 relocalization_node 发布 `map → lidar_odom` TF 解决。动态层和先验现在统一在 `map` frame。

3. **~~地面估计被动态物污染~~**：~~敌方机器人若完全盖住某 cell，running-min 可能误抬地面 → 漏检。~~ 已改为全局 p10+EMA，不再有 per-cell 地面估计。全局估计的代价是无法处理斜坡等非平面地形，但 RM 场地平整，可接受。

4. **`/cloud_registered` 帧 ID**：仿真是 lidar_odom，真车 FAST-LIO 默认 camera_init。relocalization_node 订阅 `/cloud_registered` 时 **QoS 必须用 RELIABLE**（FAST-LIO2 publisher 是 RELIABLE，BEST_EFFORT 订阅收不到数据）。

5. **log-odds 上下限**：必须设上限防单帧噪声把 cell 打成永久障碍；设下限防穿负。✅ 已实现 cap=2.0, floor=-2.0。

6. **ROG-Map 退场**：移除后失去其 raycasting，这是预期。`rog_map_sim.yaml`、sim_ws 的 `src/rog_map` symlink，以及 `perception_mapper` wrapper/config/launch/RViz 文件均已在后续清理中删除。✅

7. **QoS**：现有 pub=RELIABLE、esdf2d sub=BEST_EFFORT，兼容沿用。✅

8. **tilt 残差**：点云已在重力对齐 odom 系，静态倾斜已被 LIO 补偿；残留亚度级误差由全局地面估计吸收。快速机动时留意远端假障碍。

9. **衰减时钟**：⚠️ 必须用 steady_clock 而非 sim clock 计算 dt。wall timer + sim clock 在 Gazebo 仿真慢时会导致 dt=0 或 dt 暴跳，造成间歇性全红/全清。✅ 已修复。

10. **衰减 vs 累积平衡**：⚠️ decay_tau 和 delta_hit 必须联合调参。确保 `delta_hit > decay_per_cloud_interval`，否则 hit 永远无法累积过阈值。当前 cloud ~5Hz（0.2s/帧），decay=0.2/3.0=0.067，delta_hit=1.0，净增 0.933/帧。

11. **先验地图与实际不符**：RMUC 场地固定已知，风险≈0。若场地被改，静态层会留幻影，可接受。

12. **LiDAR 倒置连锁**：倒置后需同步修改 extrinsic_R、blind、laserscan height、segmentation gravity_aligned_frame。sensor frame 的 vertical FOV 角度不需要翻转（frame 本身已翻转）。

13. **FAST-LIO2 blind 优先级 bug**：⚠️ AVIA 路径的 blind 过滤因 C++ 运算符优先级（`&&` 高于 `||`）失效，导致 `blind` 参数完全无效、车体自身点云进入 `/cloud_registered`。已加括号修复。改 blind 值需重新 `colcon build --packages-select fast_lio`（C++ 源码，非运行时参数）。✅

---

# 阶段 3：寻路 + 控制（JPS → 逐点追踪 → MINCO）

> 状态：3.0-3.3 已实现并验证（航向控制留给 3.4）；3.4 MINCO 规划中
> 设计原则：每步独立可验证，避免多因素耦合。控制器与 JPS 解耦，逐步引入。

## 设计决策（用户确认）

- **JPS + MINCO 方案**：JPS 出几何路径（输入 occupancy grid），MINCO 做轨迹优化（输入 ESDF）。话题与语义严格区分。
- **逐步验证**：先单独 JPS（手点终点）→ 再逐点追踪控制 → 最后 MINCO。
- **不搭一次性脚手架**：控制器从一开始就用"手点目标 → 驱动小车"的可复用形式，3.2 输入单点、3.3 输入 JPS 序列，控制器零返工。
- **不引入路径平滑**：逐点追踪本质是追踪点序列，平滑留给 MINCO。3.3 的折线抖动可接受。

## 阶段划分

| 阶段 | 内容 | 验证标准 |
|------|------|---------|
| **3.0** | costmap 膨胀（按机器人半径，~0.3-0.4m，≤0.5m） | RViz 膨胀层正确，**所有通道仍连通** |
| **3.1** | JPS 节点，RViz 手点 goal | 各种起终点出合理无碰撞折线，含无解处理 |
| **3.2** | 控制器：手点单目标，直接驱动小车（速度/加速度饱和） | 空旷区平稳到点，无超调震荡 |
| **3.3** | 同一控制器追踪 JPS 点序列（切换半径 + 5Hz 重规划 + 最近点锚定） | 闭环跑通，避开静态障碍到达手点目标 |
| **3.4** | 引入 MINCO，ESDF 输入 | 轨迹平滑、动力学可行、优于折线 |

## 3.0 障碍物膨胀 ✅（已实现并验证）

### 膨胀值计算 ✅

从 URDF `sentry_robot_sim.xacro` 提取机器人尺寸：
- base_link box: 0.2×0.3m（旋转 90° → 占地 0.3×0.2m）
- 4 轮位于 (±0.1, ±0.13)，轮半径 0.06m → 外延至 x: ±0.16, y: ±0.19
- 外接半径 = √(0.16² + 0.19²) ≈ **0.25m**
- 膨胀 = 0.25 + 0.1 安全裕度 = **0.35m**

### 实现 ✅

新建 `sentry_planner` 包（ament_python），节点 `costmap_inflator`：
- 订阅 `/perception/costmap_2d` (OccupancyGrid, BEST_EFFORT)
- 二值圆形膨胀（numpy mask 卷积），膨胀半径 0.35m = 3.5 cells @ 0.1m 分辨率
- 发布 `/planner/costmap_inflated` (OccupancyGrid, RELIABLE)
- 实测输出 10.0 Hz，与输入 costmap 同频

| 文件 | 作用 |
|------|------|
| `sentry_planner/sentry_planner/costmap_inflator.py` | 膨胀节点 |
| `sentry_planner/launch/sim_planner.launch.py` | planner 启动（inflator + JPS） |
| `sentry_planner/setup.py` / `package.xml` | 包配置 |

### 验证 ✅
- ✅ RViz 中障碍物（墙体/高台）周围有清晰膨胀带
- ✅ highland_a 与 highland_b 之间对角通道仍连通，且有较大余裕
- ✅ 膨胀图 10.0 Hz 稳定输出

## 3.1 JPS 节点 ✅（已实现并验证）

### 实现 ✅

节点 `jps_node`，自写 JPS（Jump Point Search）实现：
- 订阅 `/planner/costmap_inflated` (BEST_EFFORT) + `/goal_pose` (PoseStamped, RViz 2D Nav Goal) + `/odom` (Gazebo ground truth, BEST_EFFORT)
- `use_raw_odom=True`：直接用 odom position 作为 map 坐标，绕过 TF `map → lidar_odom` 变换（仿真中 FAST-LIO 易发散，TF 不可靠）
- JPS 搜索：8 方向跳跃点搜索 + forced neighbor 检测 + 剪枝邻居
- 后处理：Bresenham line-of-sight 路径简化（移除直线段中间冗余点）
- 发布 `/planner/path` (nav_msgs/Path) + `/planner/jps_viz` (MarkerArray, 绿色折线 + 橙色航点球)
- 坐标系：`map` frame（`use_raw_odom` 模式下 Gazebo world frame = map frame，无需 TF 变换）

| 文件 | 作用 |
|------|------|
| `sentry_planner/sentry_planner/jps_node.py` | JPS 搜索 + 路径简化 + 可视化 |

### 验证 ✅
- ✅ 空旷区域点击终点 → 直线/少折点路径
- ✅ 绕过高台点击 → 合理折线避障
- ✅ 多个目标点测试（4 组）全部成功找到路径
- ✅ 路径简化有效（如 14 grid pts → 6 waypoints, 17 grid pts → 9 waypoints）
- ✅ RViz 中绿色路径折线 + 橙色航点球正确显示

## 3.2 控制器（手点单目标，可复用）✅（已实现并验证）

### 实现 ✅

新建 `goal_controller` 节点（`sentry_controller` 包）：
- 订阅 `/goal_pose` (PoseStamped, RViz 2D Nav Goal) + `/Odometry` (FAST-LIO2, 位置) + `/odom` (Gazebo ground truth, 底盘 yaw)
- 发布 `/cmd_vel_chassis` (Twist, 直接驱动 Gazebo mecanum_controller)
- 全向底盘运动学：速度向量直接投影到 body frame，无需转向到目标方向
- 速度饱和：`v = min(v_max, k_p * dist)`
- 加速度/角速度限幅：每控制周期指令变化率限制
- 到点容差 0.15m，到达后停止

| 文件 | 作用 |
|------|------|
| `sentry_controller/sentry_controller/goal_controller.py` | 单目标控制器 |
| `sentry_planner/launch/sim_planner.launch.py` | 加入 goal_controller 节点 |

### 关键设计决策

1. **底盘 yaw 来源**：仿真中 `base_link`（LiDAR 安装座）持续旋转，FAST-LIO2 `/Odometry` 的 yaw 是旋转顶的 yaw，不是底盘的 yaw。底盘 yaw 从 Gazebo `/odom`（ground truth）获取。位置仍用 `/Odometry`（与 goal 同在 `lidar_odom` 系）。
2. **绕过 `fake_vel_transform`**：直接发布到 `/cmd_vel_chassis`，避免 `fake_vel_transform` 的虚拟旋转叠加导致轨道效应。`keyboard_teleop` 也用此 topic。
3. **角速度策略**：全向底盘不需要转向到目标方向，远距离时 `wz=0`（纯平移），近距离时缓慢对齐航向。

### 调试过程

- **问题 1**：初始版本用 `/Odometry` 的 yaw → 机器人绕目标做圆周运动（旋转顶 yaw 与底盘 yaw 不一致）
- **问题 2**：发布到 `/cmd_vel` 经 `fake_vel_transform` 转换 → TF 时间戳问题导致速度被清零
- **问题 3**：发布到 `/cmd_vel_chassis` 但仍用 `/Odometry` yaw → 机器人能移动但方向偏移
- **修复**：位置用 `/Odometry`（lidar_odom 系），yaw 用 `/odom`（Gazebo 底盘 ground truth）

### 验证 ✅
- ✅ 目标 (-1, 1)：从 (1.1, 0) 出发，距离 2.34m → 0.136m，平稳到达
- ✅ 速度随距离饱和减速（v_max=0.5, k_p=1.5），无超调震荡
- ✅ 加速度限幅生效（启动时平滑加速）
- ✅ 到点后自动停止
- ⚠️ 目标点在障碍物内时机器人会卡在障碍物边缘（预期行为，3.3 中 JPS 会避免此情况）

## 3.3 逐点追踪 JPS 序列 ✅（位置追踪验证通过，航向控制留给 3.4）

### 实现 ✅

新建 `path_tracker` 节点（`sentry_controller` 包），复用 3.2 的速度/加速度饱和逻辑：
- 订阅 `/planner/path_vis` (MINCO 密集采样) + `/odom` (Gazebo ground truth, 位置+yaw)
- `use_raw_odom=True`：直接用 odom position 作为 map 坐标，绕过 TF 变换（与 JPS 一致）
- 发布 `/cmd_vel_chassis`
- **切换半径** 0.15m：进入半径内立即切下一 waypoint，避免 stop-go 抖动
- **最近点锚定**：每次收到新路径，重新锚定到离机器人最近的点

| 文件 | 作用 |
|------|------|
| `sentry_controller/sentry_controller/path_tracker.py` | JPS 逐点追踪 |

### 验证 ✅
- ✅ 手动设置 JPS 路径，机器人沿路径逐点追踪到达终点
- ✅ 速度调参：v_max=4.0, k_p=12.0, acc_lim=4.0（从初始 0.5 提速 8 倍），运动顺畅
- ✅ 切换半径生效，无明显停顿

### ⚠️ 已知问题（sim 伪影，留给 3.4 统一解决）

**航向控制不稳定**：当前 `wz` 逻辑是退化形式（`dist > goal_tol*3` 时 `wz=0`，近距离才补对齐），表现为：
- 转弯时有时转有时不转（阈值切换导致）
- 转弯有延迟
- 最初转向方向反了（已修复：`wz` 取反）

**根因**：仿真中 `base_link`（LiDAR 安装座）持续旋转，位置取自 `/Odometry`、yaw 取自 `/odom`，两源间有微小时间差与坐标耦合。低速不明显，加角速度后放大。**真车上 `base_link` 不转，无此现象。**

**决策**：不在退化逻辑上打补丁。航向控制由 3.4 MINCO 的轨迹跟踪器统一解决（轨迹自带 `yaw(t)`，不再靠阈值切换）。位置追踪作为 MINCO 的 fallback/对照保留。

## 3.4 MINCO 轨迹优化 ✅（已实现并验证，偏离原 C++/GCOPTER 方案）

### 实现概述

**原方案**：C++ 节点 + GCOPTER header-only 库 + signed ESDF + 自定义 `Trajectory2D` msg + 时间采样轨迹跟踪器。

**实际方案**：纯 Python 实现（`minco_solver_2d.py` + `minco_planner_node.py` + `esdf_map_2d.py` + `path_postprocess.py`），用 scipy L-BFGS 优化，输出 `nav_msgs/Path` 密集采样，`path_tracker` 改为 lookahead 纯追踪。

**偏离原因**：Python 实现开发更快，scipy L-BFGS 足够用，避免引入 C++/GCOPTER 构建复杂度。后续若需提速可移植 C++。

### 关键文件

| 文件 | 作用 |
|------|------|
| `sentry_planner/sentry_planner/minco_solver_2d.py` | MINCO 5 阶（minimum jerk）多项式轨迹求解器，scipy L-BFGS 优化 |
| `sentry_planner/sentry_planner/minco_planner_node.py` | ROS2 节点：订阅 JPS path + costmap，后处理 → MINCO 优化 → 发布密集采样路径 |
| `sentry_planner/sentry_planner/esdf_map_2d.py` | Signed ESDF 距离场 + 双线性插值 + 向量化批量查询 (`get_distances_batch`) |
| `sentry_planner/sentry_planner/path_postprocess.py` | JPS 路径后处理管线：剪枝 → 间距控制 → 短段合并 → 时间分配 |
| `sentry_controller/sentry_controller/path_tracker.py` | Lookahead 纯追踪控制器（替代原计划的 traj_tracker 时间采样） |
| `sentry_planner/launch/sim_planner.launch.py` | 启动 inflator + JPS + MINCO + path_tracker |

### 数据流（实际）

```
/planner/path (JPS)
  → path_postprocess: line-of-sight prune (eps=0.25rad) → spacing (0.8-1.5m) → merge short segments → time alloc
  → minco_solver_2d: 5th-order polynomial, L-BFGS optimize J = w_smooth·J_smooth + w_time·J_time + w_obs·J_clearance_soft + w_collision·J_clearance_hard + w_dyn·J_dyn
  → final collision check: min_d < d_hard → fallback to JPS raw path
  → /planner/path_vis (nav_msgs/Path, 密集采样 0.05s)
  → path_tracker: lookahead pure pursuit → /cmd_vel_chassis
```

### 与原方案的关键偏离

| 原方案 | 实际实现 | 原因 |
|--------|----------|------|
| C++ + GCOPTER header-only | Python + scipy L-BFGS | 开发效率，避免 C++ 构建复杂度 |
| Signed ESDF + obstacle cost (w_obs) | 双层 ESDF clearance 代价（soft + hard barrier） | 中心线偏好 + 安全底线，膨胀降至足迹级 |
| 自定义 Trajectory2D msg（带系数+时间） | nav_msgs/Path 密集采样 | 简化接口，第一版不需要 MPC 拼接 |
| traj_tracker 按时间采样 (x,y,yaw,v)(t) | path_tracker lookahead 纯追踪 | 复用已有 path_tracker，lookahead 足够 |
| 航点间距 0.3-0.8m | 间距 0.8-1.5m + prune_eps=0.25rad | 保留 JPS 转折点给 MINCO 足够自由度优化 |
| 中间速度=0（rest-to-rest） | 启发式中间速度（弦方向 × min相邻段速 × 0.8） | 零中间速导致分段 rest-to-rest，轨迹近直线 |

### 启发式中间速度估计

原 MINCO 初始化中间航点速度为零（rest-to-rest），导致每段独立多项式连接处速度连续但加速度不连续，优化结果趋近直线。

**修复**（`minco_solver_2d.py` `_heuristic_intermediate_va`）：
- 每个中间航点的速度方向 = 相邻两段弦方向的平均
- 速度大小 = min(相邻段距离/时间) × 0.8
- 起末速度仍为当前 odom 速度 / 零
- 效果：MINCO 生成明显曲率，不再与 JPS 折线重合

### JPS 后处理管线（`path_postprocess.py`）

```
JPS raw path
  → line_of_sight_prune (prune_eps=0.25rad ≈ 14°)   # 只保留显著转角
  → spacing_control (min=0.8m, max=1.5m)             # 插入/删除点控制间距
  → _merge_short_segments (min_spacing=0.8m)          # 合并过近航点为 midpoint
  → allocate_times (T_i = dist_i / v_alloc, t_min=1.0s)
```

**`_merge_short_segments`**：迭代合并距离 < min_spacing 的相邻航点为其中点（保留首末点），确保 MINCO 每段有足够长度生成曲率。

### ESDF Clearance 双层代价架构

**设计动机**：原方案靠 0.6m costmap 膨胀避障，但膨胀叠加（机器人两侧 0.6m + 墙膨胀 0.6m = 2.4m）导致窄通道被错误封死。改为**职责分离**：JPS 只管拓扑（薄膨胀），MINCO 用 ESDF 管安全余量（中心线偏好）。

**三层架构**：

| 层 | 职责 | 用什么图 | 数值 |
|---|---|---|---|
| JPS | hard 可行性 / 拓扑 | inflated costmap | inflation = 0.30m |
| MINCO | soft clearance / 中心线 | **raw ESDF**（未膨胀 costmap 构建） | d_soft=0.50, d_hard=0.25, w_obs=3000, w_collision=10000 |
| Final check | 安全兜底 | raw ESDF | min_d < d_hard → fallback JPS |

**双层 barrier 代价函数**（`minco_solver_2d.py` `_clearance_cost`）：

```
J_clearance = w_obs · Σ max(0, d_soft − d_i)²  +  w_collision · Σ max(0, d_hard − d_i)²
```

- `d > d_soft`（开阔区）→ 代价为 0，不过度约束
- `d_soft > d > d_hard`（贴近障碍）→ soft barrier 推离
- `d < d_hard`（危险区）→ hard barrier 强力推开
- 通道宽 < 2·d_soft 时两侧 barrier 同时作用 → 轨迹平衡在中心线

**关键实现细节**：
- ESDF 基于**原始未膨胀** `/perception/costmap_2d` 构建（`EsdfMap2D` 订阅该话题），量到真实障碍距离
- 采样 dt=0.05s（clearance）/ dt=0.08s（dynamics），向量化 `_sample_xy` 避免循环
- **向量化批量查询** `get_distances_batch(xs, ys)`：numpy 双线性插值，避免 Python 循环
- **ESDF OOB 处理**：超出 costmap 范围的点返回 `max_dist`（自由空间），而非 `0.0`（障碍表面）
- **Final collision check**：优化后密集采样 `check_clearance`，`min_d < d_hard` 时回退到 JPS 原始途径点
- **L-BFGS 提前停止**：`ftol=1e-2`, `gtol=1e-3`，通常 10-15 次迭代收敛，无需跑满 150 次

### Path Tracker 改造

从 3.3 的逐点 PD 追踪改为 **lookahead 纯追踪**：
- 找到路径上离机器人最近的点
- 沿路径向前看 `lookahead_dist=1.5m`，得到目标点
- 以 `cruise_speed=2.5m/s` 驱向目标点
- 全向底盘：速度向量直接投影到 body frame
- **yaw 来源**：从 `/odom`（Gazebo ground truth）获取，`use_raw_odom=True` 同时取位置和 yaw
- 控制频率 30Hz（高频控制减少弯道切角）

### 实际参数（sim_planner.launch.py）

```yaml
# MINCO planner
v_max: 4.0
a_max: 4.0
v_alloc: 1.5          # 时间分配速度（越小时间越长）
w_smooth: 1.0
w_time: 100.0
w_obs: 3000.0         # soft clearance 代价权重（与 smoothness/time 同量级）
w_collision: 10000.0   # hard collision 代价权重（≫ w_obs）
d_soft: 0.50           # soft barrier 距离（中心线偏好，推离障碍）
d_hard: 0.25           # hard barrier 距离（安全底线）
w_dyn: 500.0
min_spacing: 0.8       # 航点最小间距
max_spacing: 1.5       # 航点最大间距
t_min: 1.0             # 最短段时间
sample_dt: 0.05        # 采样间隔
max_iter: 150          # L-BFGS 最大迭代（ftol=1e-2 提前停止）
odom_topic: /odom      # Gazebo ground truth（FAST-LIO 仿真发散）

# Path tracker
v_max: 3.0
lookahead_dist: 0.5    # 短预瞄紧贴轨迹（防弯道切角撞墙）
cruise_speed: 1.5      # 降速过弯
rate_hz: 30.0          # 高频控制
switch_radius: 0.15
goal_tol: 0.20
acc_lim: 2.0           # 限加速度防过冲
use_raw_odom: true     # 绕过 TF，直接用 /odom position

# Costmap inflation
inflation_radius_m: 0.30  # 仅机器人足迹级，安全余量由 MINCO ESDF clearance 负责
```

### 调试历程

1. **MINCO 路径与 JPS 几乎相同（无曲率）**
   - 根因：中间航点速度=0 → rest-to-rest 分段 → 近直线
   - 修复：启发式中间速度估计（弦方向 × 0.8）

2. **机器人速度慢、有停顿感（stop-and-go）**
   - 根因：path_tracker 逐点 PD 控制，每个 waypoint 到达后减速
   - 修复：改为 lookahead 纯追踪 + cruise_speed 常速

3. **机器人撞墙（costmap 显示 0.4m 间距）**
   - 根因：lookahead 在弯道看向更远的点，切了弯道导致偏离安全路径
   - 修复：inflation_radius 从 0.35 增到 0.6m；加 `_merge_short_segments` 确保航点间距
   - **后续修正**：0.6m 膨胀过保守（0.6×4 叠加 = 2.4m 通道被封），改为 0.30m + MINCO ESDF clearance 双层代价
   - **二次修正**：w_obs=100 太小（障碍代价 ~50 vs smoothness ~700+），MINCO 不推离障碍。改为 w_obs=3000, d_soft=0.50
   - **二次修正**：path_tracker lookahead=1.5m + cruise=2.5m/s 弯道切角撞墙。改为 lookahead=0.5m, cruise=1.5m/s, rate=30Hz

4. **部分弯道仍接近直线**
   - 根因：JPS 在某些转角处生成两个靠得很近的节点
   - 修复：`_merge_short_segments` 合并近点 + `prune_eps=0.25rad` 只保留显著转角

5. **运动卡顿（jerky，非停顿）**
   - 根因 1：path_tracker 用 `/odom`（~1Hz）获取 yaw，控制循环 20Hz → 19/20 周期用 stale yaw
   - 修复 1：改用 `/Odometry`（~10Hz）获取 yaw
   - 根因 2：11 个重复 esdf2d 进程占用大量 CPU → gzserver 81% → Gazebo RTF < 1
   - 修复 2：杀掉所有残留 esdf2d 进程
   - 根因 3：Gazebo planar_move 插件发布 odom TF 与 sim clock 时间戳不一致 → TF_OLD_DATA 警告刷屏
   - 修复 3：禁用 `publish_odom_tf`（path_tracker 已不依赖 /odom）
   - 根因 4：path_tracker 控制频率 20Hz 远超 odom 10Hz
   - 修复 4：控制频率降到 10Hz

### 验证 ✅
- ✅ MINCO 生成明显曲率，不再与 JPS 折线重合
- ✅ lookahead 纯追踪消除 stop-and-go，运动连续
- ✅ inflation 0.30m + ESDF clearance 下机器人不撞墙且走通道中心线
- ✅ MINCO final check: min_d=0.41~0.51 >= d_hard=0.250（优化后安全验证通过）
- ✅ 卡顿问题解决（杀残留进程 + 禁用 odom TF + 降控制频率）
- ✅ TF_OLD_DATA 警告消除
- ✅ 向量化 ESDF 批量查询 + `_sample_xy` + L-BFGS ftol=1e-2 将优化耗时从 60s+ 降到 **14~121ms**
- ✅ ESDF OOB 修复：超出 costmap 返回 max_dist 而非 0.0
- ✅ path_tracker 紧贴轨迹：lookahead=0.5m + cruise=1.5m/s + 30Hz，弯道不切角
- ✅ use_raw_odom 绕过 TF：JPS + MINCO + path_tracker 均用 `/odom`（Gazebo ground truth），避免 FAST-LIO 仿真发散

### ⚠️ 已知问题与后续工作

1. ~~**w_obs=0.0**~~：**已解决**。当前使用双层 ESDF clearance 代价（soft `w_obs=3000` + hard `w_collision=10000`），d_soft=0.50m，JPS 膨胀降至 0.30m，MINCO 走通道中心线。
2. **无自定义 Trajectory2D msg**：当前用 nav_msgs/Path 密集采样，无速度/加速度/系数信息。后续接 MPC 或重规划拼接时需升级接口。
3. **无时间采样跟踪**：path_tracker 用 lookahead 而非按轨迹时间采样，无法精确跟踪速度剖面。后续可改为 traj_tracker。
4. **无 5Hz 重规划**：当前仅在 JPS 发新路径时触发 MINCO。后续需加定时器重规划以处理动态障碍。
5. ~~**无 fallback**~~：**已解决**。MINCO final check `min_d < d_hard` 时回退到 JPS 原始路径。
6. **Gazebo RTF < 1**：30000 samples 雷达插件导致 gzserver ~80% CPU，sim time 慢于 real time。这是仿真性能限制，不影响算法正确性。
7. ~~**优化耗时 ~6s**~~：**已解决**。向量化采样 + L-BFGS ftol=1e-2 + max_iter=150，优化耗时降至 **14~121ms**（< 0.1s 目标基本达成）。
8. **d_soft 需场景标定**：`d_soft=0.50` 需根据最窄必经通道宽度确认，过大会封死窄通道。

### 原设计方案（保留作参考）

> 以下为原 C++/GCOPTER 设计方案，实际实现偏离较大但架构思路保留供后续移植参考。

### 0. ESDF 现状确认（已核验 2026-06-22）

| 项 | 值 |
|----|----|
| 话题 | `/perception/esdf_2d` |
| 类型 | `sensor_msgs/PointCloud2`（非栅格数组） |
| 尺寸 | 130×90 = 11700 点，覆盖 13m×9m |
| **分辨率** | **0.1m** |
| origin | (-6.5, -4.5)，frame `lidar_odom` |
| intensity 语义 | `到最近障碍欧氏距离 + 10.0` (m)，截断 5.0m |
| 障碍来源 | **原始** costmap `/perception/costmap_2d`（cell==100），**未膨胀** |
| 算法 | `scipy.ndimage.distance_transform_edt`（真欧氏距离） |
| 发布触发 | 事件驱动（costmap 更新时） |

**关键推论**：
- ESDF 用的是**未膨胀**原始障碍图 → 给的是到障碍的**真实**距离。因此 MINCO 的安全距离 `d_safe` 必须自己包含机器人半径（`d_safe = r_robot + margin`），不能再叠加膨胀，否则双重保守。
- 分辨率 0.1m 偏粗，碰撞梯度会有台阶感 → 必须做**双线性插值**取距离和梯度，且建议对 ESDF 做一次轻度平滑（高斯 σ≈1 cell）再求梯度。
- PointCloud2 不适合频繁随机查询 → **MINCO 节点应直接订阅 `/perception/costmap_2d`，自己用 numpy/C++ 维护一份 ESDF 距离场数组**（重算 < 2ms），避免从点云反解栅格。现有 `intensity` 点云仅用于 RViz 对照。

**⚠️ 重要修正：必须用 Signed ESDF（否则优化不收敛）**

现有 `esdf2d_node` 的 `distance_transform_edt` 给的是 free→最近 occupied 的**正距离，障碍内部距离=0、梯度=0**。优化迭代中采样点**经常暂时落入障碍内部**，此时梯度为零，优化器不知往哪推 → 卡住或穿障。

MINCO 节点内部必须自建 **signed ESDF**：
```
d_signed(x,y) = d_free_to_occ − d_occ_to_free
```
- free 区为正、occupied 区为负，**障碍内部也有有效梯度指向自由空间**
- 实现：对 obstacle_mask 和 ~obstacle_mask 各跑一次 `distance_transform`，相减。成本翻倍但仍 < 2ms
- collision cost 用 `d_signed`，`d_safe − d_signed` 在障碍内部为大正值 → 强推力

**命名卫生（避免概念混淆，三者严格区分）**：
| 名称 | 含义 |
|------|------|
| `occupied_raw` | 真实障碍（原始 costmap cell==100） |
| `occupied_inflated` | 膨胀后障碍（给 JPS 搜索用的碰撞边界） |
| `signed_esdf` (MINCO 用) | 基于 `occupied_raw` 的 signed 距离场，供 MINCO 碰撞代价 |

变量命名不得含糊（如禁止裸 `esdf_distance`，需写明 `signed_esdf_from_raw`）。

### 1. 总体架构

```
JPS path (/planner/path) ─┐
                          ├─> minco_planner_node ─┬─> /planner/path_vis   (nav_msgs/Path, 密集采样, RViz/debug)
costmap_2d (/perception/ ─┘   │  ├ 自建 signed ESDF │
           costmap_2d)        │  ├ JPS 后处理管线    └─> /planner/minco_traj (Trajectory2D 自定义 msg, 正式控制接口)
                              │  └ MincoBackend(GCOPTER)            │
                              └ 定时器 5Hz 重规划                    ↓
                                                          traj_tracker (改造 path_tracker)
                                                          按时间采样 (x,y,yaw,v)(t) → /cmd_vel_chassis
```

- **新增节点 `minco_planner_node`**（`sentry_planner` 包，**C++**，因要 include GCOPTER header）
- **新增/改造跟踪器 `traj_tracker`**（`sentry_controller` 包，Python 即可）

### 2. 库选型决策

- **核心库用 GCOPTER（ZJU-FAST-Lab），header-only 抠取**：`minco.hpp`、`lbfgs.hpp`、`root_finder.hpp` 拷进 `sentry_planner/include/gcopter/`。只用其轨迹参数化 + L-BFGS，**不引入它的飞行器约束/前端**。
- **不自己手写 MINCO 梯度回传**：`∂c/∂q, ∂c/∂T` 穿过稀疏带状系统的链式法则极易出错，调试成本远高于复用。
- **GCOPTER 封装到 `MincoBackend` 类，不让类型扩散到 ROS 层**：
  ```cpp
  class MincoBackend {
   public:
    bool optimize(const JpsPath& path, const SignedEsdf& esdf,
                  const BoundaryState& start, const BoundaryState& goal,
                  Trajectory2D& traj);   // 对外只暴露我们自己的 Trajectory2D
  };
  ```
  GCOPTER 内部类型只活在 backend 内，换实现/调 cost/改 L-BFGS 不影响 ROS 接口。
- **碰撞/动力学代价函数自己写**：这部分耦合我们的 signed ESDF 和全向底盘模型，必须定制。

### 3. 运动学模型（全向底盘，比开源版简单）

- 底盘 holonomic：`vx, vy` 独立，`yaw` 与平移**解耦**。
- 轨迹设计为 **2D 位置轨迹 `(x(t), y(t))`**（MINCO 2 维）。**yaw 不进 MINCO 优化**，由单独的 yaw policy 生成，架构上预留多模式：
  ```
  enum YawMode {
    TANGENT,       # yaw = atan2(ẏ, ẋ)，跟随速度方向（第一版默认）
    FIXED,         # 固定朝向
    TARGET_TRACK,  # 朝向某目标点（自瞄/盯敌）
    EXTERNAL,      # 上层直接覆写 yaw
  };
  ```
  - **第一版只实现 `TANGENT`**，后续切 `TARGET_TRACK` 不需重做 MINCO（yaw 与轨迹解耦）。
  - 这样哨兵"边走边瞄"的需求只是换 yaw policy，平移轨迹复用。
- 因 holonomic 无非完整约束 → **不需要曲率约束**，优化问题比差速车显著简单。

### 4. 优化问题设计（代价函数）

无约束总代价（软约束 + L-BFGS）：

```
J = w_smooth · J_smooth + w_time · J_time + w_obs · J_obs + w_dyn · J_dyn
```

| 项 | 形式 | 梯度来源 |
|----|------|----------|
| `J_smooth` | `∫ ‖jerk‖² dt`（minimum jerk，s=3） | GCOPTER 闭式 |
| `J_time` | `Σ T_i` | GCOPTER 闭式 |
| `J_obs` | 轨迹按 `κ` 个采样点/段，查 **signed ESDF** 得 `d_signed`，`Σ ρ(d_safe − d_signed)` | 自写：双线性插值 signed ESDF 梯度 ∇d |
| `J_dyn` | `Σ max(0, ‖v‖²−v_max²)² + max(0, ‖a‖²−a_max²)²` | 自写：对采样点速度/加速度 |

- **碰撞代价用平方 hinge loss**：`ρ(x) = x² if x>0 else 0`，`x = d_safe − d_signed`。因用 signed ESDF，轨迹落入障碍内部时 `d_signed<0` → `x` 为大正值 → 强推力出障碍。
- **采样积分**：每段按弧长或固定 `κ=8~16` 个点采样（FAST-Lab 标准做法），约束惩罚在采样点累加，梯度按链式回传到 `(q,T)`。
- **惩罚而非硬约束**：所有不等式约束软化成 `max(0,·)²`，保证目标函数 C¹，L-BFGS 可用。
- **第一版不加额外项**：不加朝向项/敌人危险项/贴墙奖励项，那些会让调参空间爆炸。先跟通 4 项再说。

### 5. 关键参数配置

| 参数 | 建议初值 | 说明 / 整定注意 |
|------|----------|-----------------|
| `d_safe` | 0.35m | = 外接半径 0.25 + 0.1 margin，**和 costmap 膨胀值对齐**（ESDF 未膨胀，所以这里必须含半径） |
| `v_max` | 4.0 m/s | 复用 3.3 调好值；软约束，实际峰值可能略超，留 10% 裕量 |
| `a_max` | 4.0 m/s² | 同上 |
| `w_smooth` | 1.0 | 基准权重，其他相对它调 |
| `w_time` | 实验整定 | 太大→激进超速啃约束；太小→磨蹭。先小后大 |
| `w_obs` | 较大（如 1e3） | 安全优先；太大→轨迹被障碍"推"得震荡 |
| `w_dyn` | 中等（如 1e2） | 保证 v/a 不爆 |
| 段数 | = JPS 航点数−1 | JPS 简化后航点直接作 MINCO 中间路点。航点过密时先抽稀 |
| `κ`（每段采样） | 8~16 | 太少→约束漏检穿障；太多→优化慢 |
| L-BFGS max_iter | 64~128 | 保证 5Hz 下单次 < 200ms |
| ESDF 平滑 σ | 1 cell | 求梯度前轻度高斯，缓解 0.1m 台阶 |

### 5.5 JPS → MINCO 后处理管线（关键中间层，不可略）

JPS 原始路径**不能直接喂 MINCO**（原始点太密 → 段数爆炸、过约束；太稀 → 可能穿障碍）。中间必须一条后处理管线：

```
JPS raw path
  → line-of-sight pruning   (去除共线冗余点)
  → waypoint spacing control (控制相邻航点间距)
  → time allocation         (每段时间分配 T_i)
  → MINCO initialization     (初始路点 q_init + 边界状态)
```

**航点间距控制规则（第一版，保守优先）**：
- 相邻 waypoint 距离控制在 **0.3–0.8m**
- **转角处保点**（不能删，否则丢拐角）
- **直线段尽量删点**（减段数、加速优化）
- **窄通道入口/出口保点**（避免轨迹切到门框）
- 时间分配：每段 `T_i = dist_i / v_max` 粗略分配，设下限避免 `T→0`。第一版不需完美，保守稳定即可。

### 6. 函数 / 模块设计

`EsdfMap2D` 类（C++，独立可测）：

| 函数 | 职责 |
|------|------|
| `update(costmap)` | 收 raw costmap → 对 obstacle_mask 与 ~obstacle_mask 各跑一次距离变换，相减得 **signed ESDF**，预算梯度场 |
| `getDistance(x,y)` | 双线性插值返回 `d_signed`，越界返回 `+max_dist` |
| `getDistanceAndGradient(x,y)` | 返回 `(d_signed, ∇d)`，梯度用 central difference 或预算梯度图再插值 |

`minco_planner_node`（C++）核心函数：

| 函数 | 职责 |
|------|------|
| `onCostmap()` | 调 `EsdfMap2D::update()` 刷新 signed ESDF |
| `onJpsPath()` | 调 §5.5 后处理管线 → `q_init` + 时间初值 |
| `costFunction(x, grad)` | L-BFGS 回调：组装 4 项代价 + 梯度（核心，约 150 行），碰撞项调 `EsdfMap2D::getDistanceAndGradient` |
| `MincoBackend::optimize()` | 封装 GCOPTER + L-BFGS，输出 `Trajectory2D` |
| `publishTraj()` | 发 `/planner/path_vis`（Path）+ `/planner/minco_traj`（Trajectory2D） |
| `applyYawPolicy()` | 按 `YawMode` 生成 `yaw(t)`，第一版 TANGENT |
| 定时器 5Hz | 重规划循环，复用最新 JPS path + ESDF；失败时走 fallback |

`traj_tracker`（Python，改造自 `path_tracker`）：

| 函数 | 职责 |
|------|------|
| `onTraj()` | 收轨迹，记录起始时间 `t0` |
| `controlLoop()` | `t=now−t0`，按时间采样轨迹得 `(x*,y*,yaw*,vx*,vy*)`，前馈 + 位置 PID 反馈 → `/cmd_vel_chassis` |
| yaw 跟踪 | `wz = k_yaw·(yaw* − yaw_chassis)`，**连续控制**（替代 3.3 的阈值切换，根治转向抖动） |

### 7. 接口定义（提前定死，避免返工）

- **输入**：`/planner/path`（JPS, `nav_msgs/Path`）+ `/perception/costmap_2d`（`OccupancyGrid`）
- **输出：双 topic，不二选一**（修正：原计划“先用 Path”会返工，因 Path 缺时间戳/速度/加速度/系数，后续 MPC/重规划拼接会被迫改 msg）：
  - **`/planner/path_vis`**（`nav_msgs/Path`，密集采样）——只给 RViz/debug
  - **`/planner/minco_traj`**（自定义 `Trajectory2D`）——正式控制接口
- **自定义 msg `Trajectory2D`**（最低限度也要带系数 + 段时间，即使第一版跟踪只用 samples）：
  ```
  std_msgs/Header header
  uint8 yaw_mode                # YawMode 枚举
  float64[] durations           # 每段 T_i
  float64[] coef_x              # 每段 N 阶多项式系数（N+1 个/段）
  float64[] coef_y
  TrajectoryPoint2D[] samples   # 可选：采样点，方便调试和低速控制器
  ```
  `TrajectoryPoint2D`: `float64 t, x, y, vx, vy, ax, ay, yaw, yaw_rate`
  - **第一版跟踪只读 `samples`**，但 `coef_x/coef_y/durations` 必须在 msg 里预留，后续接 MPC/重规划拼接不用改 msg。
- frame 一律 `lidar_odom`，与 JPS/ESDF 一致。
- msg 包位置：新建 `sentry_msgs`（或复用现有 interfaces 包），供 planner 与 controller 共享。

### 8. 风险与注意事项

1. **Signed ESDF + 梯度质量是成败关键**：必须 signed（障碍内部有梯度）；0.1m 分辨率 + 高斯平滑 + 双线性插值。若优化不收敛/轨迹抖，**首先怀疑梯度**，用 §9 第 2 步的可视化 ∇d 排查。
2. **时间初值**：`T_i` 离谱会陷局部最优。按 `dist/v_max` 给，并设下限避免 T→0。
3. **JPS 后处理不可略**（§5.5）：航点过密会让段数爆炸、优化慢；过疏会漏掉拐角/穿障。剪枝 + 间距控制 0.3–0.8m + 转角/窄通道保点。
4. **C++ 引入构建成本**：`minco_planner_node` 是本项目首个自写 C++ 节点，需配 `CMakeLists.txt` + ament_cmake。GCOPTER 依赖 Eigen（已有）。
5. **5Hz 实时性**：单次优化必须 < 200ms。先测单次耗时再开重规划。
6. **sim 转向伪影根治验证**：3.3 的转向抖动若在 `traj_tracker` 连续 yaw 控制下消失，则确认是阈值切换问题而非 frame 问题；若仍在，需回到 `/odom` yaw 时间同步排查。
7. **Sim2Real 参数分离**：`v_max/a_max/d_safe` 在 sim 和 real 应分离配置（real 底盘横纵向能力可能不同，holonomic 的 vy 上限可能 < vx）。

### 9. 实施步骤（已按“先打基础、隔离调试”重排）

1. **`EsdfMap2D` 类**：输入 costmap，输出 `getDistance(x,y)` 和 `getDistanceAndGradient(x,y)`（signed ESDF + 双线性插值，梯度 central difference）
2. **ESDF 梯度 RViz 验证**：发 `/perception/esdf_grad_2d` marker 箭头，随机采点打印 `(d, ∂d/∂x, ∂d/∂y)`，确认方向/尺度/边界/障碍内部表现。**这一步不过，后面 MINCO 都是玄学调参。**
3. **JPS path 后处理**（§5.5）：剪枝、重采样、时间分配
4. **接 MINCO，先关 obstacle cost**：只验证轨迹能从 JPS waypoints 生成平滑轨迹，速度/加速度合理
5. **开 obstacle cost**：简单绕障、穿门、窄通道三类场景测试
6. **发双接口**：`/planner/path_vis` 给 RViz，`/planner/minco_traj` 给控制器；实现 `traj_tracker`（连续 yaw 控制）闭环
7. **加 fallback**：MINCO 失败时退回 JPS 简化路径低速跟踪，而不是直接无输出
8. 开 5Hz 重规划，测实时性 + 动态避障；参数整定，更新文档

## 阶段 3 注意事项

1. **膨胀过度堵路**：膨胀值必须验证通道连通性，宁可小不可大。≤0.5m 硬上限。
2. **frame 一致性**：JPS 输入图、goal、控制器目标必须同 frame（`lidar_odom`）。RViz fixed frame 要对齐。
3. **控制器饱和**：速度/加速度限幅是"不漂移"论证的前提，不可省。
4. **接口前置定死**：JPS 输出 `nav_msgs/Path`，MINCO 吃此格式 + ESDF，提前定死避免返工。
5. **重规划锚定**：5Hz 重规划后必须最近点锚定，否则目标乱跳。
6. **符号链接连带**：planner/controller 若放 real_ws 包，sim_ws 同步生效，注意 Sim2Real 参数分离。

---

## 4.0 Tracking Controller / MPC 实施计划

### 0. 目标与总体判断

当前 `path_tracker`（lookahead 纯追踪）在 90° 转弯处存在明显过冲，即使速度已偏慢（cruise=1.5 m/s）仍冲出轨迹。

根因分析（已确认）：

1. **无 lateral error correction**：lookahead 纯追踪只追前方目标点方向，不关心机器人离路径多远，偏离后无力拉回
2. **分轴加速度限幅**：`_apply_acc_lim` 对 vx/vy 独立限幅，90° 弯时合加速度达 `√2 × acc_lim`，比预期大 41%
3. **cruise_speed 不随曲率自适应**：弯道处仍 1.5 m/s，加速度需求 `v²/r` 远超 `acc_lim`
4. **lookahead 可能跨弯道**：lookahead_dist=0.5m 在密集采样路径上可能跨过 90° 弯顶点

推荐路线：不直接替换为完整 MPC，而是分阶段演进：

```
阶段 1：Frenet tracking baseline（横向误差反馈 + 合速度限幅）
阶段 2：速度规划与前瞻减速（曲率自适应 + 制动距离约束）
阶段 3：Linear tracking MPC（QP 求解，OSQP）
阶段 4：Delay compensation
阶段 5：SE(2) MPC / 轮系约束（后续升级）
```

### 1. 系统边界

MPC/tracker 只负责轨迹跟踪，不负责避障。避障仍由 JPS + MINCO + ESDF 完成。

```
JPS / MINCO Planner → reference trajectory → Tracking Controller / MPC → cmd_vel
```

**关键前提确认**（当前系统已满足）：

| 项 | 当前状态 |
|---|---|
| MINCO 输出时间参数化 | ✅ `sample_trajectory` 返回 `(t, x, y, vx, vy, ax, ay, yaw)` |
| 位置+yaw 来源 | ✅ `/odom`（Gazebo ground truth, `use_raw_odom=True`） |
| 速度反馈 | ⚠️ 当前仅用 position，未用 twist（阶段 1 需补充） |
| 坐标系 | ✅ world frame = map frame = Gazebo odom frame（`use_raw_odom` 模式下） |
| cmd_vel frame | body frame（`vx_body = cos(α)·v, vy_body = sin(α)·v`） |

### 2. 阶段 1：Frenet Tracking Baseline

#### 2.1 核心思路

从"追一个点"升级为"贴合一条线"：

1. **轨迹投影**：在参考路径上找最近投影点，得到切向 `t`、法向 `n`、横向误差 `e_n`
2. **控制律**：

   ```
   v_cmd_world = v_ref · t  -  k_p · e_n · n  -  k_d · v_n · n
   ```

   - 沿轨迹切向前进（`v_ref · t`）
   - 偏离轨迹时横向拉回（`k_p · e_n · n`）
   - 横向速度过大时抑制（`k_d · v_n · n`）

3. **合速度/合加速度限幅**：

   ```python
   dv = v_des - v_last
   if norm(dv) > acc_lim * dt:
       dv = dv / norm(dv) * acc_lim * dt
   v_cmd = v_last + dv
   ```

   替代当前分轴限幅，避免对角方向加速度超限。

#### 2.2 实现要点

- 复用现有 `path_tracker.py` 框架，替换 `_control_loop` 核心逻辑
- 参考 trajectory 来自 `/planner/path_vis`（MINCO 密集采样，已带 vx/vy/yaw）
- 投影：遍历路径点找最近点（路径 ≤ 200 点，O(n) 足够）
- 切向：`t = normalize(p[i+1] - p[i])`
- 法向：`n = (-t_y, t_x)`（左手法向）
- 横向误差：`e_n = (robot_pos - p_ref) · n`

#### 2.3 初始参数

```yaml
control_rate: 30.0        # Hz（保持当前）
kp_lat: 2.0               # 横向位置误差增益
kd_lat: 0.5               # 横向速度阻尼
cruise_speed: 1.0         # m/s（先保守，阶段 2 再提速）
max_acc: 1.5              # m/s²（合加速度上限）
max_yaw_rate: 2.0         # rad/s
```

调参顺序：先调 `kp_lat` 让车能回轨迹 → 再调 `kd_lat` 压振荡 → 最后提 `cruise_speed`。

#### 2.4 验收标准

- 直线跟踪不明显蛇形
- 90° 弯横向误差明显小于原 tracker
- 偏离轨迹后能主动拉回
- cmd_vel 不出现方向突变

### 3. 阶段 2：速度规划与前瞻减速

#### 3.1 曲率自适应限速

根据轨迹曲率限制速度：

```
v_κ = sqrt(a_lat_max / (|κ| + ε))
v_ref = min(cruise_speed, v_κ)
```

不只看当前曲率，看前方 0.5~1.0m 内最大曲率，提前减速。

#### 3.2 制动距离约束

从弯道低速点向前反推速度上限：

```
v_i² ≤ v_{i+1}² + 2 · a_brake · Δs
```

避免机器人到弯心才减速。

#### 3.3 合加速度限幅（已在阶段 1 实现，此处强化）

#### 3.4 验收标准

- 入弯前速度自动下降
- 合速度与合加速度曲线连续
- 不再出现切弯或冲出轨迹
- cruise_speed 稍微提高也不会突然恶化

### 4. 阶段 3：Linear Tracking MPC

#### 4.1 状态与控制量

```
状态 x = [px, py, vx, vy, ψ, ω]^T
控制 u = [ax, ay, α]^T
```

#### 4.2 离散模型（线性）

```
p_{k+1} = p_k + v_k·dt + 0.5·a_k·dt²
v_{k+1} = v_k + a_k·dt
ψ_{k+1} = ψ_k + ω_k·dt + 0.5·α·dt²
ω_{k+1} = ω_k + α·dt
```

#### 4.3 优化目标

```
J = Σ_k |p_k - p_ref,k|_Q² + |v_k - v_ref,k|_Qv² + |ψ_k - ψ_ref,k|_Qψ²
    + |u_k|_R² + |u_k - u_{k-1}|_Rd²
```

权重设计：lateral 误差权重 > tangential 误差权重；control rate penalty 偏大防抖。

#### 4.4 约束

第一版用 box constraint 近似圆约束：

```
|vx|, |vy| ≤ v_max
|ax|, |ay| ≤ a_max
|ω| ≤ ω_max
|α| ≤ α_max
```

#### 4.5 参数建议

```yaml
mpc:
  rate: 30.0              # Hz
  dt: 0.05                # s
  horizon_N: 15           # 0.75s 预测
  solver: osqp            # Python: osqp, C++: qpOASES

limits:
  v_max: 1.5              # m/s
  a_max: 1.5              # m/s²
  omega_max: 2.0          # rad/s
  alpha_max: 4.0          # rad/s²

weights:
  q_pos_tangent: 1.0
  q_pos_normal: 5.0
  q_vel_tangent: 0.5
  q_vel_normal: 3.0
  q_yaw: 0.5
  q_yaw_rate: 0.2
  r_acc: 0.1
  r_alpha: 0.1
  r_delta_acc: 0.5
  r_delta_alpha: 0.5
```

#### 4.6 Reference Sampler

MINCO 已是时间参数化轨迹，直接按时间采样：

```python
for k in range(N):
    t_k = t_now + k * dt
    x_ref, y_ref, vx_ref, vy_ref, yaw_ref = minco.evaluate(t_k)
```

`minco_solver_2d.py` 的 `evaluate(coeffs_x, coeffs_y, durations, t)` 已提供此能力。

#### 4.7 输出

MPC 控制量是 `(ax, ay, α)`，但底盘接收速度命令，取第一步预测速度：

```
v_cmd_world = [vx_1, vy_1]
ω_cmd = ω_1
→ body frame: v_body = R(ψ)^T · v_world
→ cmd_vel.linear.x = vx_body, cmd_vel.linear.y = vy_body, cmd_vel.angular.z = ω_cmd
```

#### 4.8 实现语言选择

- **第一版（阶段 1-2）**：Python，复用现有 `path_tracker.py` 框架，快速迭代
- **MPC 版（阶段 3）**：Python + OSQP（pip install osqp），若 solver_time > 15ms 再考虑 C++ 移植

### 5. 阶段 4：Delay Compensation

用上一次命令把当前状态 rollout 到未来：

```python
x_pred = x_now
for dt in delay_window:
    x_pred = dynamics_step(x_pred, last_cmd, dt)
mpc_initial_state = x_pred
```

初始 delay 估计 100ms，调试时 sweep 0/50/100/150/200ms。

### 6. Fallback 机制

```
1. solver success → MPC output
2. solver fail 但上一帧有效 → 沿用上一帧命令 × 0.8 衰减
3. 连续 fail ≥ 3 次 → 切换 Frenet baseline controller
4. 状态异常 / 定位跳变 → 刹停
```

### 7. 调试与可视化

#### 必须发布 debug 量

```
/mpc/predicted_trajectory    (Marker, 预测轨迹)
/mpc/reference_points        (Marker, 未来参考点)
/mpc/debug_error             (lateral_error, tangent_error, yaw_error)
/mpc/debug_solver_status     (solve_time_ms, status)
```

#### RViz 可视化

- 实际 odom 轨迹
- MINCO reference trajectory
- MPC predicted trajectory
- 最近投影点 + lateral error vector

#### 判断问题来源

| 现象 | 问题来源 |
|------|----------|
| cmd_vel 已提前转向但实际速度沿旧方向 | 底盘低层延迟 |
| cmd_vel 未提前减速/转向 | tracker / MPC |
| odom 轨迹跳变 | 定位 |
| solver fail 后车辆突变 | fallback 缺失 |

### 8. 阶段验收指标

在同一条测试轨迹上对比 old tracker / Frenet baseline / MPC：

| 指标 | 初期目标 | 后续目标 |
|------|----------|----------|
| max lateral error | < 0.20m | < 0.10~0.15m |
| RMS lateral error | < 0.08m | < 0.05m |
| 90° turn overshoot | 明显小于原 tracker | 进一步减小 |
| solver avg time | < 5ms | < 5ms |
| solver max time | < 15ms | < 15ms |
| solver fail count | 0 | 0 |

### 9. 实施顺序

| Step | 内容 | 产出 |
|------|------|------|
| 1 | Reference sampler（投影 + 切向/法向 + 曲率） | `reference_sampler.py` + debug marker |
| 2 | Frenet baseline controller（横向误差反馈 + 合速度限幅） | 替换 `_control_loop` + debug topic |
| 3 | 速度规划（曲率自适应 + 制动距离约束） | `speed_profile.py` |
| 4 | Linear MPC prototype（OSQP QP） | `tracking_mpc.py` + `mpc_node.py` |
| 5 | MPC 与 baseline 并行对比 | rosbag + plot script + 对比表 |
| 6 | Delay compensation | `delay_compensator.py` + sweep test |
| 7 | 真车前检查 | 坐标系 / odom / solver / fallback / 急停 |

### 10. 推荐最终架构

```
JPS / MINCO Planner
        │
        ▼
  Reference Sampler (投影 + 切向/法向 + 曲率 + 速度规划)
        │
        ├── Frenet Baseline Controller  ← fallback
        │
        ▼
  Tracking MPC (OSQP QP, horizon=15, dt=0.05s)
        │
        ▼
  Delay Compensation / Output Formatter
        │
        ▼
  world-frame velocity → body-frame velocity
        │
        ▼
  cmd_vel / chassis command
```

### 11. 当前最推荐的下一步

先做 Step 1-2（reference sampler + Frenet baseline），不立即写 MPC：

1. 轨迹投影 + tangent/normal 计算
2. lateral error 反馈
3. 合速度加速度限幅
4. debug 可视化

完成后若 90° 弯过冲明显下降，确认根因判断正确。Frenet baseline 随后成为 MPC 的 debug baseline 和 fallback。

---

## 4.0 Step 1-2 实施记录

### 已完成改动

#### path_tracker.py — Frenet 轨迹追踪控制器

**替换内容**：旧 pure pursuit lookahead 控制器 → Frenet 投影 + 横向误差反馈控制器

**新增方法**：
- `_project_to_path(idx)`：投影机器人到路径，返回 (ref_x, ref_y, tx, ty, nx, ny, e_n, curvature)
- `_max_curvature_ahead(idx, lookahead_m)`：前瞻距离内最大曲率
- `_brake_speed_limit(idx, v_target)`：从终点反推速度上限（制动距离约束）

**控制律**：
```
v_world = v_ref · t  −  kp_lat · e_n · n  −  kd_lat · v_n · n
```
- `v_ref` = min(cruise_speed, v_κ, brake_limit, v_max)
- `v_κ = sqrt(a_lat_max / (|κ| + ε))` 曲率自适应减速
- 合速度/合加速度限幅（替代旧 per-axis 限幅，消除对角方向 41% 超额加速度）

**速度反馈**：
- `/odom` twist 是 body frame (child_frame_id=base_link)，需旋转到 world frame
- `vx_world = cos(yaw)·vx_body − sin(yaw)·vy_body`
- `vy_world = sin(yaw)·vx_body + cos(yaw)·vy_body`

**坐标系说明**：
- 当前 `use_raw_odom=True`，直接用 Gazebo `/odom`（odom frame ≈ map frame，偏差 ~0.1m）
- `use_raw_odom=False` 模式已修复（通过 `map → lidar_odom` TF 变换），但 LIO 存在累积漂移
- 后续 MPC 阶段需解决持续定位问题

#### sim_planner.launch.py — 参数更新

当前参数（速度翻倍后）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `v_max` | 3.0 | 合速度上限 (m/s) |
| `cruise_speed` | 2.0 | 巡航速度 (m/s) |
| `acc_lim` | 3.0 | 合加速度上限 (m/s²) |
| `w_max` | 4.0 | 角速度上限 (rad/s) |
| `acc_lim_yaw` | 8.0 | 角加速度上限 (rad/s²) |
| `kp_lat` | 4.0 | 横向位置误差增益 |
| `kd_lat` | 1.0 | 横向速度阻尼 |
| `a_lat_max` | 3.0 | 侧向加速度上限 (m/s²) |
| `curvature_lookahead` | 0.5 | 曲率前瞻距离 (m) |
| `brake_decel` | 2.0 | 制动减速度 (m/s²) |
| `lookahead_dist` | 0.0 | 0 = Frenet 模式 |
| `use_raw_odom` | True | 用 Gazebo ground truth |

### 测试结果

| 指标 | 旧 pure pursuit | Frenet baseline |
|------|----------------|-----------------|
| max lateral error | 0.244m | 0.020m (↓12×) |
| 稳态 lateral error | 0.18-0.23m | < 0.02m |
| 弯道 k=0.43 过冲 | 严重 | e_n=0.019m |
| 到达精度 | 0.185m | 0.135m |
| 撞墙 | 是 | 否 |

### 已知问题

1. **LIO 累积漂移**：relocalization 只在启动时做一次 ICP，运动后 FAST-LIO2 里程计漂移增大，影响 `use_raw_odom=False` 模式
2. **坐标系分离**：TF 树中 `map → lidar_odom` 和 `odom → base_link` 是两个独立分支，无 `map → odom` 连接
3. **后续需要**：持续 SLAM 定位或周期性 ICP 重校正，才能切换到 `use_raw_odom=False` 在 map frame 精确跟踪

### 2026-06-23 更新：控制主线调整为 traj_tracker → MPC

#### 现象

在当前高速配置（`cruise_speed=2.0`, `v_max=3.0`）下，Frenet baseline 相比 pure pursuit 改变了过冲形态，但没有根治：
- pure pursuit：90° 弯呈大弧线切弯
- Frenet baseline：冲出轨迹后优先横向回线，再继续前进

这说明横向误差反馈已生效，但控制器仍是 **reactive path following**，不是严格的时间参数化轨迹跟踪。

#### 根因判断

当前 `/planner/path_vis` 是 `nav_msgs/Path`，只包含密集位置点和 yaw，不包含 MINCO 已经计算出的 `t/v/a`。因此 `path_tracker` 实际只是在几何路径上按 `cruise_speed` 自行生成速度；弯道前馈、制动时序和参考加速度都没有进入控制器。

Frenet baseline 可以作为 fallback/debug baseline 保留，但不适合作为高速过弯的最终主控。

#### 新决策

先实现 **time-parameterized `traj_tracker`**，不要直接上 MPC。

路线：
1. `minco_planner_node` 在 `/planner/path_vis` 外，新增发布 `/planner/traj_samples`
2. 第一版用 `std_msgs/Float64MultiArray`，每个 sample 8 个 float：
   ```
   [t, x, y, vx, vy, ax, ay, yaw]
   ```
   后续再升级为正式 `Trajectory2D` 自定义 msg
3. 新增 `sentry_controller/traj_tracker.py`：
   ```
   v_cmd_world = v_ref + kp_pos * (p_ref - p_now) + kd_vel * (v_ref - v_now)
   ```
   再做合速度/合加速度限幅，转 body frame 后发布 `/cmd_vel_chassis`
4. 若 `traj_tracker` 仍无法压住 90° 弯过冲，再进入 Linear Tracking MPC。届时 MPC 只替换控制律，不再同时重搭轨迹接口。

#### 验收目标

- 同一路径上对比 Frenet baseline 与 traj_tracker
- `traj_tracker` 能利用 MINCO 的速度剖面，入弯前自然降速
- 90° 弯不再出现“先冲出去再回轨迹”的明显反应式形态
- 若仍过冲，记录 `p_ref/v_ref/a_ref` 与实际 odom 误差，作为 MPC 权重和延迟补偿输入

### 2026-06-23 更新：控制频率与加速度限幅修正

#### 现象

新版 `traj_tracker` 的偏离限速逻辑已将长 S 弯峰值过冲从旧版约 `1.61m / 1.21m` 降到约 `1.05m / 0.63m`，但仍存在入弯反应滞后。

进一步排查发现：
- `traj_tracker` 参数 `rate_hz=30.0`
- 实测 `/clock`、`/odom`、`/cmd_vel_chassis` 墙钟频率约 `6.3-6.6Hz`
- Gazebo `real_time_factor≈0.65`
- `mecanum_controller` 插件中 `<publish_rate>10</publish_rate>`，因此仿真时间下 `/odom` 最高约 `10Hz`

这说明瓶颈不是 Python 控制器算不动，而是仿真时钟和底盘 odom 发布率限制。更重要的是，旧版 `traj_tracker` 用固定 `dt=1/rate_hz=0.033s` 做加速度限幅；当实际控制回调约 `0.1s` 一次时，等效加速度限幅被压低约 3 倍，导致入弯减速和横向纠偏都晚半拍。

#### 已修改

1. `sentry_controller/traj_tracker.py`
   - 加速度限幅改用真实控制周期 `control_dt = now - last_control_time`
   - `control_dt` 做 `[1ms, 250ms]` 夹紧，避免仿真暂停/时间回退造成异常跃变
   - CSV debug 新增 `control_dt` 字段
   - 日志新增 `dt=...`

2. Gazebo planar move 插件配置
   - `rm_nav_bringup/urdf/sentry_robot_sim.xacro`
   - `rm_simulation/pb_rm_simulation/urdf/simulation_waking_robot.xacro`
   - 将 `<publish_rate>` 从 `10` 提高到 `50`

#### 后续验证

重启 Gazebo 后重新测：
```bash
ros2 topic hz /clock
ros2 topic hz /odom
ros2 topic hz /cmd_vel_chassis
```

期望：
- `/odom` 在仿真时间内不再被 10Hz 硬限制
- 若 `real_time_factor` 仍约 `0.65`，墙钟 `/odom` 频率应接近 `30Hz` 量级，而不是 `6Hz`
- `/tmp/traj_tracker_debug.csv` 中 `control_dt` 应接近实际回调间隔

若频率修正后仍有明显高速过冲，再继续做曲率预瞄限速或 Frenet 分解版 tracker；MPC 仍作为下一阶段选项，而不是当前立即切换。

### 2026-06-23 更新：高速档参数试验

#### 背景

真实 `control_dt` 修正后，长 S 弯最大 `nearest_d` 已降到约 `0.15-0.18m`，说明此前的大过冲主要来自等效加速度不足，而不是 `traj_tracker` 控制结构本身失效。

当前速度仍低于预期，主要原因是 MINCO 时间分配仍使用 `v_alloc=1.5m/s`。即使 tracker 允许更高 `v_max`，控制器拿到的参考速度也不会自然翻倍。

#### 已修改为 fast profile

`sentry_planner/launch/sim_planner.launch.py`：

| 模块 | 参数 | 旧值 | 新值 | 说明 |
|------|------|------|------|------|
| MINCO | `v_alloc` | 1.5 | 3.0 | 时间分配速度翻倍，直接缩短轨迹时长 |
| MINCO | `v_max` | 4.0 | 6.0 | 给优化器更高速度余量 |
| MINCO | `a_max` | 4.0 | 16.0 | 同一曲率速度翻倍时横向加速度需求约 4 倍 |
| traj_tracker | `v_max` | 3.0 | 6.0 | 控制器合速度上限翻倍 |
| traj_tracker | `acc_lim` | 3.0 | 12.0 | 允许高速档下快速建立/回收速度 |
| traj_tracker | `max_feedback_speed` | 1.0 | 2.0 | 偏离轨迹时的纠偏速度上限同步提高 |
| traj_tracker | `offtrack_v_max` | 1.2 | 2.0 | 保留离轨保护，但不把高速测试压得过慢 |

#### 验证重点

下一轮长 S 弯重点看：
- `/tmp/traj_tracker_debug.csv` 中 `vref_mag/cmd_mag` 是否接近翻倍
- `nearest_d` 是否仍维持在 `0.2-0.3m` 以内
- 若出现新过冲，优先判断是 `a_max/acc_lim` 不足、轨迹曲率太激进，还是 odom/control 频率仍不足

若高速档下只在急弯出现局部过冲，下一步应做曲率预瞄限速，而不是立刻整体降速。

#### 二阶段：降低 MINCO 最短段时间

第一次 fast profile 验证显示控制误差仍较小：
- `nearest_d` 峰值约 `0.16-0.20m`
- `cmd_mag` 最大约 `2.2-2.3m/s`

但 MINCO 日志仍显示长 S 弯 `T≈14.0s`、`max_v≈1.9m/s`。原因是 JPS/MINCO 路径被拆成 14 段，而 launch 中 `t_min=1.0` 造成总时长下限 `14 × 1.0s = 14s`，抵消了 `v_alloc=3.0` 的提速效果。

已将 `sentry_planner/launch/sim_planner.launch.py` 中：
```yaml
t_min: 1.0
```
调整为：
```yaml
t_min: 0.5
```

下一轮重点观察：
- MINCO 日志中的 `T` 是否显著低于 `14s`
- `max_v` 是否提升到 `3m/s` 量级
- `nearest_d` 是否仍能维持在 `0.3m` 以内

#### 二阶段结论：回退 `t_min=1.0`

`t_min=0.5` 试验确实将轨迹时长压到约 `6.5s`，速度明显提高，但暴露出规划侧问题：

```text
MINCO final check FAILED: min_d=0.190 < d_hard=0.250, falling back to JPS path
JPS fallback min_d=0.270
MINCO: T=6.54s, max_v=4.23, max_a=23.13, a_viol=14
```

这说明短时长使平滑轨迹在直角/贴墙区域切入障碍；final check 失败后退回 JPS fallback，而 fallback 只比硬安全距离多约 `2cm`，视觉上接近折线并贴边，导致撞墙风险上升。

因此先将 launch 中 `t_min` 回退到 `1.0`。后续若继续提速，不应单纯降低 `t_min`，而应实现：
- 动态/碰撞检查失败后自动放大 durations 并重算
- 或曲率/障碍感知的局部时间分配
- 或在 fallback 发布前强制满足更大的安全距离

### 2026-06-23 更新：MINCO 软参考改造

#### 动机

不希望通过增大 JPS inflation 来获得安全裕度，因为这会把部分真实可行通道误判为不可行；也暂不切换到带 clearance cost 的 A*，避免引入搜索开销和额外调参。

目标是让 JPS 只提供拓扑参考，MINCO 在保持同一通道拓扑的前提下，利用 ESDF clearance 把轨迹从贴边 JPS 往更安全的位置推。

#### 已实现

`sentry_planner/minco_solver_2d.py`：
- 新增 `w_ref`
- 新增 `waypoint_bound_m`
- L-BFGS 优化变量仍为中间 waypoint 位置
- 中间 waypoint 不再被视为必须严格贴原 JPS 点，而是在原点附近有限范围内优化
- 代价函数新增：

```text
J_ref = w_ref * Σ ||p_i - p_i_jps||²
```

其中 ESDF clearance cost 负责推离障碍，`J_ref` 只负责防止 waypoint 跑出原拓扑太远。

`sentry_planner/minco_planner_node.py`：
- 声明并传入 `w_ref`
- 声明并传入 `waypoint_bound_m`
- MINCO 日志新增 `wp_shift=max/avg ... m`，用于观察优化后的 waypoint 相对 JPS 移动量

`sentry_planner/launch/sim_planner.launch.py` 当前试验参数：

```yaml
w_ref: 20.0
waypoint_bound_m: 0.60
```

#### 验证重点

下一轮看：
- `MINCO final check OK` 是否稳定
- `min_d` 是否相比 JPS fallback 更接近/超过 `d_soft=0.50`
- `wp_shift=max/avg` 是否有实际移动，而不是仍为 0
- RViz 中 `/planner/path_vis` 是否从贴边 JPS 向通道内部偏移
- `/tmp/traj_tracker_debug.csv` 中 `nearest_d` 是否仍维持在可接受范围

若 waypoint 移动不足，可降低 `w_ref` 或提高 `d_soft/w_obs`；若移动过度或切错拓扑，可降低 `waypoint_bound_m` 或提高 `w_ref`。

#### 2026-06-23 参数放宽

初次软参考试验中，RViz 观察到整体路径更平滑，但部分 90° 角仍有贴边/磕碰风险。为给 MINCO 更多空间从 JPS 折点向通道内部圆角化，先保持 `w_ref=20.0` 不变，仅将 waypoint 局部活动半径放宽：

```yaml
waypoint_bound_m: 0.60 -> 1.00
```

下一轮重点观察：
- 90° 角处是否明显远离障碍
- `wp_shift=max/avg` 是否接近 1.0m 上限；若经常打满，说明还可能需要降低 `w_ref` 或增大 `d_soft`
- 是否出现切错通道或穿越障碍；若出现，回收 `waypoint_bound_m` 到 `0.8`
