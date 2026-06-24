# sentry_perception

RoboMaster 哨兵感知包。基于 FAST-LIO2 输出的机体系点云做障碍物提取。

## 节点

### obstacle_detector（第一版）

```
订阅： /cloud_registered_body  (sensor_msgs/PointCloud2, frame=body)
发布： /perception/obstacle_cloud (sensor_msgs/PointCloud2, frame=body)
```

流水线：

1. **Raw bytes → numpy (N, 3) float32** — 直接读 `msg.data` 切片，比 `read_points` 迭代快 ~10×
2. **ROI 盒子过滤**（body 系）：默认 `|x|<5, |y|<5, -1<z<2`
3. **体素下采样**（hash 法，默认 0.10 m）：每格保留一个点
4. **地面阈值**：`z > ground_z_thresh`（默认 −0.05 m，上车后应改到 −0.25 m）
5. 输出 PointCloud2，直接 `xyz.tobytes()`

性能目标：MID360 每帧约 2 万点、10 Hz，单帧总耗时 < 15 ms。

参数见 `config/obstacle_detector.yaml`。

## 启动

仅感知节点（FAST-LIO2 已在别处跑）：
```bash
ros2 launch sentry_perception perception.launch.py use_sim_time:=true
```

一键起「bag 回放 + FAST-LIO2 + perception + RViz」：
```bash
# 终端 A
ros2 launch sentry_perception replay_perception.launch.py

# 终端 B
./src/sentry_bringup/scripts/play_bag.sh data/bags/static_empty_90s 1.0
```

## 下一步规划

- [ ] DBSCAN / 欧氏聚类 → BoundingBoxArray
- [ ] 障碍物稳定跟踪（按最近邻 ID 保持）
- [ ] RANSAC 地面分割替代 z 阈值（倾斜地面鲁棒）
- [ ] 输出 2D 占据栅格 `/perception/local_costmap`，喂给 planner
- [ ] 动态/静态障碍物分离（利用累积 ikd-tree 地图做差）
