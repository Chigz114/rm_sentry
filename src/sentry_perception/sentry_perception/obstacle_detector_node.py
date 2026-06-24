#!/usr/bin/env python3
"""障碍物检测第一版：对 FAST-LIO2 发布的机体系点云 /cloud_registered_body 做
ROI 裁剪 + 体素下采样 + 地面阈值过滤，输出 /perception/obstacle_cloud。

处理全部在 body 坐标系（frame_id=body）：
  - ROI 是以机器人为中心的长方体
  - 地面在 body 系下 z ≈ -LiDAR 安装高度，阈值 ground_z_thresh 默认 -0.25 m
  - 输出保留 frame_id=body，下游如需 world 系可订阅后 TF 变换或让 costmap 直接订 body

性能目标：MID360 每帧约 2 万点、10 Hz，整条流水线单帧 < 15 ms（纯 numpy 够用）。
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N,3) float32，尽量走快速路径：直接 memoryview 解析 raw bytes。"""
    field_off = {f.name: f.offset for f in msg.fields}
    if not all(k in field_off for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    ox, oy, oz = field_off['x'], field_off['y'], field_off['z']
    step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    # 快速路径：xyz 是 float32 且内存连续 (ox, ox+4, ox+8)
    if oy == ox + 4 and oz == ox + 8 and step >= ox + 12:
        rows = raw.reshape(n, step)
        xyz = rows[:, ox:ox + 12].copy().view(np.float32).reshape(n, 3)
        return xyz
    # 兼容路径：逐字段抽取
    f32 = raw.view(np.uint8).reshape(n, step)
    def col(offset):
        return f32[:, offset:offset + 4].copy().view(np.float32).reshape(n)
    return np.stack([col(ox), col(oy), col(oz)], axis=-1)


def xyz_to_pc2(header: Header, xyz: np.ndarray) -> PointCloud2:
    """numpy (N,3) float32 → PointCloud2，直接拼 raw bytes 避免 Python 循环。"""
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    n = len(xyz)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False
    from sensor_msgs.msg import PointField
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.data = xyz.tobytes()
    return msg


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # --- 话题 ---
        self.declare_parameter('input_topic',  '/cloud_registered_body')
        self.declare_parameter('output_topic', '/perception/obstacle_cloud')
        # --- ROI (body 坐标系) ---
        self.declare_parameter('roi_x_min', -5.0)
        self.declare_parameter('roi_x_max',  5.0)
        self.declare_parameter('roi_y_min', -5.0)
        self.declare_parameter('roi_y_max',  5.0)
        self.declare_parameter('roi_z_min', -1.0)
        self.declare_parameter('roi_z_max',  2.0)
        # --- 下采样 / 地面 ---
        self.declare_parameter('voxel_size',       0.10)
        self.declare_parameter('ground_z_thresh', -0.25)
        self.declare_parameter('min_points_out',   10)
        # --- 日志 ---
        self.declare_parameter('log_every_n', 20)

        self._load_params()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb, qos)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)

        self.frame_count = 0
        self.sum_dt_ms = 0.0

        self.get_logger().info(
            f'ObstacleDetector 启动\n'
            f'  in : {self.input_topic}\n'
            f'  out: {self.output_topic}\n'
            f'  ROI body frame: '
            f'x[{self.roi_x_min:+.1f},{self.roi_x_max:+.1f}] '
            f'y[{self.roi_y_min:+.1f},{self.roi_y_max:+.1f}] '
            f'z[{self.roi_z_min:+.1f},{self.roi_z_max:+.1f}]\n'
            f'  voxel={self.voxel_size:.2f}m  ground z>{self.ground_z_thresh:+.2f}m'
        )

    def _load_params(self):
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.input_topic    = g('input_topic')
        self.output_topic   = g('output_topic')
        self.roi_x_min      = g('roi_x_min')
        self.roi_x_max      = g('roi_x_max')
        self.roi_y_min      = g('roi_y_min')
        self.roi_y_max      = g('roi_y_max')
        self.roi_z_min      = g('roi_z_min')
        self.roi_z_max      = g('roi_z_max')
        self.voxel_size     = g('voxel_size')
        self.ground_z_thresh= g('ground_z_thresh')
        self.min_points_out = g('min_points_out')
        self.log_every_n    = g('log_every_n')

    def cb(self, msg: PointCloud2):
        t0 = time.perf_counter()

        xyz = pc2_to_xyz(msg)
        n_in = len(xyz)
        if n_in == 0:
            return

        # 1) 过滤 NaN/Inf
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        n_finite = len(xyz)

        # 2) ROI 盒子
        m = ((xyz[:, 0] > self.roi_x_min) & (xyz[:, 0] < self.roi_x_max) &
             (xyz[:, 1] > self.roi_y_min) & (xyz[:, 1] < self.roi_y_max) &
             (xyz[:, 2] > self.roi_z_min) & (xyz[:, 2] < self.roi_z_max))
        xyz = xyz[m]
        n_roi = len(xyz)

        # 3) 体素下采样（hash 法，每格保留一个点）
        if n_roi > 0 and self.voxel_size > 0:
            keys = np.floor(xyz / self.voxel_size).astype(np.int64)
            keys -= keys.min(axis=0)
            dy = int(keys[:, 0].max()) + 1
            dz = dy * (int(keys[:, 1].max()) + 1)
            hashed = keys[:, 0] + keys[:, 1] * dy + keys[:, 2] * dz
            _, uniq = np.unique(hashed, return_index=True)
            xyz = xyz[uniq]
        n_voxel = len(xyz)

        # 4) 地面阈值
        xyz = xyz[xyz[:, 2] > self.ground_z_thresh]
        n_out = len(xyz)

        # 5) 发布
        if n_out >= self.min_points_out:
            out_msg = xyz_to_pc2(msg.header, xyz)
            self.pub.publish(out_msg)

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        self.sum_dt_ms += dt_ms
        if self.log_every_n > 0 and self.frame_count % self.log_every_n == 0:
            avg = self.sum_dt_ms / self.frame_count
            self.get_logger().info(
                f'#{self.frame_count:4d}  '
                f'in={n_in:5d} finite={n_finite:5d} roi={n_roi:5d} '
                f'voxel={n_voxel:4d} out={n_out:4d}  '
                f'dt={dt_ms:5.1f}ms avg={avg:5.1f}ms'
            )


def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
