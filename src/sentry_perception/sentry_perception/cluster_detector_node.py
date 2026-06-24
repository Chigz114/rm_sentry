#!/usr/bin/env python3
"""欧式聚类节点（实现上等价于 DBSCAN 的核心部分）。

订阅 obstacle_detector 输出的 /perception/obstacle_cloud，
按欧式距离阈值把点聚类成若干个体（人、桌子、墙块……），
每个 cluster 输出：
  - 中心点 (centroid)
  - 轴对齐 3D 包围盒 (AABB)
  - 点数

发布话题：
  /perception/obstacle_markers    visualization_msgs/MarkerArray（RViz 可视化，绿色框 + 中心小球）
  /perception/obstacle_centroids  geometry_msgs/PoseArray          （给下游规划用）

算法：
  cKDTree.query_pairs 一次性拿到所有近邻对 (i,j)，Union-Find 合并。
  对 1600-3000 点的规模，单帧 < 10 ms。
"""

import time
import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg


# -------- 辅助：raw bytes → (N,3) float32，和 obstacle_detector 里用的是同一段逻辑 --------
def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    field_off = {f.name: f.offset for f in msg.fields}
    if not all(k in field_off for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    ox, oy, oz = field_off['x'], field_off['y'], field_off['z']
    step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if oy == ox + 4 and oz == ox + 8 and step >= ox + 12:
        rows = raw.reshape(n, step)
        xyz = rows[:, ox:ox + 12].copy().view(np.float32).reshape(n, 3)
        return xyz
    # 兼容路径
    f32 = raw.reshape(n, step)
    def col(offset):
        return f32[:, offset:offset + 4].copy().view(np.float32).reshape(n)
    return np.stack([col(ox), col(oy), col(oz)], axis=-1)


# -------- 辅助：Union-Find，O(α(n)) 近线性 --------
class UnionFind:
    def __init__(self, n):
        self.p = np.arange(n)
    def find(self, x):
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x
    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb


# 一些固定颜色用于 ID 染色（RGB 0-1）
_COLOR_PALETTE = np.array([
    [0.2, 1.0, 0.2],   # 绿
    [1.0, 0.85, 0.1],  # 黄
    [0.3, 0.8, 1.0],   # 青
    [1.0, 0.4, 0.8],   # 粉
    [0.7, 0.5, 1.0],   # 紫
    [1.0, 0.6, 0.1],   # 橙
    [0.4, 1.0, 0.7],   # 薄荷
    [1.0, 1.0, 1.0],   # 白
], dtype=np.float32)


class ClusterDetector(Node):
    def __init__(self):
        super().__init__('cluster_detector')

        self.declare_parameter('input_topic',        '/perception/obstacle_cloud')
        self.declare_parameter('markers_topic',      '/perception/obstacle_markers')
        self.declare_parameter('centroids_topic',    '/perception/obstacle_centroids')

        # 欧式聚类
        self.declare_parameter('cluster_tolerance', 0.30)   # m
        self.declare_parameter('min_cluster_size',  5)
        self.declare_parameter('max_cluster_size',  2000)

        # 渲染
        self.declare_parameter('box_line_width', 0.02)      # m
        self.declare_parameter('marker_lifetime_s', 0.3)    # s
        self.declare_parameter('color_by_id', True)

        self.declare_parameter('log_every_n', 20)

        self._load_params()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb, qos)
        self.pub_markers   = self.create_publisher(MarkerArray, self.markers_topic,   qos)
        self.pub_centroids = self.create_publisher(PoseArray,   self.centroids_topic, qos)

        self.frame_count = 0
        self.sum_dt_ms = 0.0

        self.get_logger().info(
            f'ClusterDetector 启动\n'
            f'  in : {self.input_topic}\n'
            f'  out: {self.markers_topic}, {self.centroids_topic}\n'
            f'  tolerance={self.cluster_tol:.2f}m  '
            f'min_size={self.min_size}  max_size={self.max_size}'
        )

    def _load_params(self):
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.input_topic     = g('input_topic')
        self.markers_topic   = g('markers_topic')
        self.centroids_topic = g('centroids_topic')
        self.cluster_tol     = float(g('cluster_tolerance'))
        self.min_size        = int(g('min_cluster_size'))
        self.max_size        = int(g('max_cluster_size'))
        self.box_line_width  = float(g('box_line_width'))
        self.marker_lifetime = float(g('marker_lifetime_s'))
        self.color_by_id     = bool(g('color_by_id'))
        self.log_every_n     = int(g('log_every_n'))

    def cb(self, msg: PointCloud2):
        t0 = time.perf_counter()
        xyz = pc2_to_xyz(msg)
        n = len(xyz)
        if n < self.min_size:
            # 还是要发一个空 MarkerArray 来清除 RViz 上的旧框
            self._publish_clear_only(msg.header)
            return

        # 1) Build KDTree and get all near-neighbor pairs
        tree = cKDTree(xyz)
        pairs = tree.query_pairs(r=self.cluster_tol, output_type='ndarray')

        # 2) Union-Find 合并
        uf = UnionFind(n)
        if len(pairs):
            for i, j in pairs:
                uf.union(int(i), int(j))

        # 3) 抽 cluster ids
        # find root for each point（向量化）
        roots = np.array([uf.find(i) for i in range(n)], dtype=np.int64)
        unique_roots, inverse, counts = np.unique(roots, return_inverse=True, return_counts=True)

        # 过滤大小
        ok_mask = (counts >= self.min_size) & (counts <= self.max_size)
        ok_indices = np.where(ok_mask)[0]

        clusters = []
        for ci in ok_indices:
            pts_idx = np.where(inverse == ci)[0]
            pts = xyz[pts_idx]
            centroid = pts.mean(axis=0)
            bb_min = pts.min(axis=0)
            bb_max = pts.max(axis=0)
            clusters.append({
                'n': len(pts_idx),
                'centroid': centroid,
                'min': bb_min,
                'max': bb_max,
            })

        # 发布
        self._publish_markers(msg.header, clusters)
        self._publish_centroids(msg.header, clusters)

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        self.sum_dt_ms += dt_ms
        if self.log_every_n > 0 and self.frame_count % self.log_every_n == 0:
            avg = self.sum_dt_ms / self.frame_count
            self.get_logger().info(
                f'#{self.frame_count:4d}  '
                f'pts={n:4d}  pairs={len(pairs):5d}  '
                f'clusters={len(clusters):2d}  '
                f'dt={dt_ms:5.1f}ms avg={avg:5.1f}ms'
            )

    def _publish_clear_only(self, header):
        arr = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header = header
        arr.markers.append(delete_all)
        self.pub_markers.publish(arr)

        pa = PoseArray()
        pa.header = header
        self.pub_centroids.publish(pa)

    def _publish_markers(self, header, clusters):
        arr = MarkerArray()

        # 先清空上一帧
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header = header
        arr.markers.append(delete_all)

        life = DurationMsg(sec=int(self.marker_lifetime),
                           nanosec=int((self.marker_lifetime % 1) * 1e9))

        for i, c in enumerate(clusters):
            col = _COLOR_PALETTE[i % len(_COLOR_PALETTE)] if self.color_by_id else np.array([0.2, 1.0, 0.2])

            # --- 包围盒（LINE_LIST，12 条边）---
            box = Marker()
            box.header = header
            box.ns = 'bbox'
            box.id = i
            box.type = Marker.LINE_LIST
            box.action = Marker.ADD
            box.scale.x = self.box_line_width
            box.color.r = float(col[0])
            box.color.g = float(col[1])
            box.color.b = float(col[2])
            box.color.a = 1.0
            box.lifetime = life
            box.pose.orientation.w = 1.0

            xmin, ymin, zmin = [float(v) for v in c['min']]
            xmax, ymax, zmax = [float(v) for v in c['max']]
            corners = [
                (xmin, ymin, zmin), (xmax, ymin, zmin),
                (xmax, ymax, zmin), (xmin, ymax, zmin),
                (xmin, ymin, zmax), (xmax, ymin, zmax),
                (xmax, ymax, zmax), (xmin, ymax, zmax),
            ]
            edges = [
                (0,1),(1,2),(2,3),(3,0),   # bottom
                (4,5),(5,6),(6,7),(7,4),   # top
                (0,4),(1,5),(2,6),(3,7),   # verticals
            ]
            from geometry_msgs.msg import Point
            for a, b in edges:
                p1 = Point(x=corners[a][0], y=corners[a][1], z=corners[a][2])
                p2 = Point(x=corners[b][0], y=corners[b][1], z=corners[b][2])
                box.points.append(p1)
                box.points.append(p2)
            arr.markers.append(box)

            # --- 中心小球 ---
            sphere = Marker()
            sphere.header = header
            sphere.ns = 'centroid'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(c['centroid'][0])
            sphere.pose.position.y = float(c['centroid'][1])
            sphere.pose.position.z = float(c['centroid'][2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.12
            sphere.scale.y = 0.12
            sphere.scale.z = 0.12
            sphere.color.r = float(col[0])
            sphere.color.g = float(col[1])
            sphere.color.b = float(col[2])
            sphere.color.a = 1.0
            sphere.lifetime = life
            arr.markers.append(sphere)

            # --- 文本标签（点数）---
            text = Marker()
            text.header = header
            text.ns = 'label'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(c['centroid'][0])
            text.pose.position.y = float(c['centroid'][1])
            text.pose.position.z = float(zmax) + 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"#{i}  n={c['n']}"
            text.lifetime = life
            arr.markers.append(text)

        self.pub_markers.publish(arr)

    def _publish_centroids(self, header, clusters):
        pa = PoseArray()
        pa.header = header
        for c in clusters:
            p = Pose()
            p.position.x = float(c['centroid'][0])
            p.position.y = float(c['centroid'][1])
            p.position.z = float(c['centroid'][2])
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_centroids.publish(pa)


def main():
    rclpy.init()
    node = ClusterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
