"""
感知前端第 4 层：2D ESDF 节点

输入：/perception/costmap_2d  (nav_msgs/OccupancyGrid)
         第 3 层投影得到的 2D 占据图，cell ∈ {100=occ, 0=free, -1=unknown}
输出：/perception/esdf_2d     (sensor_msgs/PointCloud2 with intensity)
         每个 cell 一个点，intensity = 2D 欧氏距离到最近障碍 + 10.0 (m)
         （+10 偏移和第 2 层 ROG/ESDF 的 intensity 语义保持一致，方便 RViz 共用色方案）

算法：scipy.ndimage.distance_transform_edt（Felzenszwalb 线性 2D 距离变换）
      对 100×100 网格，耗时 < 1 ms。

语义选择：
  - treat_unknown_as_obstacle=False (默认)：未探测区当 free 处理，距离自由扩散
    适合：规划器希望穿越未知区（允许 exploration）
  - treat_unknown_as_obstacle=True：unknown 也当 occupied，距离被截断
    适合：保守规划，不穿越未知区

和第 2 层 ROG/ESDF 的区别：
  - 第 2 层：3D 欧氏距离在整个空间算，可视化时切 z=robot_z 一层 —— 本质是 3D 数据
  - 第 4 层（本节点）：2D 欧氏距离在 xy 平面算 —— 本质就是 2D 数据
  - 规划器需要的是 2D 数据（机器人平面运动），所以第 4 层才是规划器真正用的 ESDF
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from scipy.ndimage import distance_transform_edt


class ESDF2DNode(Node):
    def __init__(self):
        super().__init__('esdf2d_node')

        self.declare_parameter('costmap_topic', '/perception/costmap_2d')
        self.declare_parameter('esdf_topic',    '/perception/esdf_2d')
        self.declare_parameter('demo_copy_topic', '/perception/esdf_2d_demo')
        self.declare_parameter('demo_copy_enable', False)
        self.declare_parameter('demo_copy_x_offset', 0.0)
        self.declare_parameter('demo_copy_y_offset', 0.0)
        self.declare_parameter('demo_copy_z_offset', 0.0)
        self.declare_parameter('treat_unknown_as_obstacle', False)
        self.declare_parameter('max_distance_m', 5.0)   # 截断，避免颜色被拉伸

        in_topic  = self.get_parameter('costmap_topic').value
        out_topic = self.get_parameter('esdf_topic').value
        demo_topic = self.get_parameter('demo_copy_topic').value
        self.demo_copy_enable = bool(self.get_parameter('demo_copy_enable').value)
        self.demo_offset = np.array([
            float(self.get_parameter('demo_copy_x_offset').value),
            float(self.get_parameter('demo_copy_y_offset').value),
            float(self.get_parameter('demo_copy_z_offset').value),
        ], dtype=np.float32)
        self.unknown_is_obstacle = self.get_parameter('treat_unknown_as_obstacle').value
        self.max_dist = float(self.get_parameter('max_distance_m').value)

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(OccupancyGrid, in_topic, self.on_costmap, sub_qos)
        self.pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self.demo_pub = None
        if self.demo_copy_enable:
            self.demo_pub = self.create_publisher(PointCloud2, demo_topic, pub_qos)

        self.get_logger().info(
            f"2D ESDF 启用: in={in_topic}, out={out_topic}, "
            f"unknown_as_obstacle={self.unknown_is_obstacle}, max_dist={self.max_dist}m, "
            f"demo_copy={self.demo_copy_enable}, demo_topic={demo_topic}, "
            f"demo_offset=({self.demo_offset[0]}, {self.demo_offset[1]}, {self.demo_offset[2]})"
        )

    def on_costmap(self, msg: OccupancyGrid):
        H = msg.info.height
        W = msg.info.width
        res = msg.info.resolution

        # OccupancyGrid.data 是 row-major (H, W)；原点在 info.origin（左下角）
        # 但 RViz 把 origin.y + j * res 当第 j 行中心，所以 (i=0, j=0) 是左下
        grid = np.asarray(msg.data, dtype=np.int8).reshape(H, W)

        if self.unknown_is_obstacle:
            obstacle_mask = (grid == 100) | (grid == -1)
        else:
            obstacle_mask = (grid == 100)

        # --- 2D 欧氏距离变换 ---
        # distance_transform_edt 计算每个 True 元素到最近 False 元素的距离（单元数）
        # 我们要每个 free 元素到最近 obstacle 的距离，所以 invert 掩膜
        if not obstacle_mask.any():
            # 全 free：距离设为 max_dist
            dist_m = np.full((H, W), self.max_dist, dtype=np.float32)
        else:
            dist_cells = distance_transform_edt(~obstacle_mask).astype(np.float32)
            dist_m = dist_cells * res
            # 截断
            np.minimum(dist_m, self.max_dist, out=dist_m)

        # --- 打包成 PointCloud2 ---
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # xs: (H, W) x 方向索引，ys: y 方向索引
        ys, xs = np.meshgrid(np.arange(H), np.arange(W), indexing='ij')
        x_world = (origin_x + (xs + 0.5) * res).astype(np.float32)
        y_world = (origin_y + (ys + 0.5) * res).astype(np.float32)
        z_world = np.zeros_like(x_world, dtype=np.float32)

        # intensity = dist + 10.0，和 ROG/ESDF 的约定对齐
        intensity = (dist_m + 10.0).astype(np.float32)

        # (N, 4) 打平
        points = np.stack(
            [x_world.ravel(), y_world.ravel(), z_world.ravel(), intensity.ravel()],
            axis=1,
        )

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud)

        if self.demo_pub is not None:
            demo_points = points.copy()
            demo_points[:, :3] += self.demo_offset
            demo_cloud = pc2.create_cloud(header, fields, demo_points)
            self.demo_pub.publish(demo_cloud)


def main():
    rclpy.init()
    node = ESDF2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
