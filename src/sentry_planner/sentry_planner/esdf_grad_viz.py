"""
ESDF gradient verification node.

Subscribes to /perception/costmap_2d, builds a signed ESDF via EsdfMap2D,
and publishes:
  - /perception/signed_esdf_2d  (MarkerArray CUBE_LIST, color blocks: blue=free, red=obstacle)
  - /perception/esdf_grad_2d    (MarkerArray ARROW, yellow gradient arrows on subsampled grid)

Also logs sample queries at key points (robot origin, near obstacles, inside obstacles)
to verify signed distance values and gradient directions.

Usage:
  ros2 run sentry_planner esdf_grad_viz
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion

from .esdf_map_2d import EsdfMap2D


class EsdfGradVizNode(Node):
    def __init__(self):
        super().__init__('esdf_grad_viz')

        self.declare_parameter('costmap_topic', '/perception/costmap_2d')
        self.declare_parameter('esdf_topic', '/perception/signed_esdf_2d')
        self.declare_parameter('grad_marker_topic', '/perception/esdf_grad_2d')
        self.declare_parameter('max_distance_m', 5.0)
        self.declare_parameter('smooth_sigma', 1.0)
        self.declare_parameter('grad_arrow_step', 4)       # sample every N cells
        self.declare_parameter('grad_arrow_scale', 0.3)     # arrow length scale
        self.declare_parameter('log_samples', True)

        in_topic = self.get_parameter('costmap_topic').value
        out_esdf = self.get_parameter('esdf_topic').value
        out_grad = self.get_parameter('grad_marker_topic').value
        max_dist = float(self.get_parameter('max_distance_m').value)
        smooth_sigma = float(self.get_parameter('smooth_sigma').value)
        self.arrow_step = int(self.get_parameter('grad_arrow_step').value)
        self.arrow_scale = float(self.get_parameter('grad_arrow_scale').value)
        self.log_samples = bool(self.get_parameter('log_samples').value)

        self.esdf_map = EsdfMap2D(max_distance_m=max_dist, smooth_sigma=smooth_sigma)

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
        self.esdf_pub = self.create_publisher(MarkerArray, out_esdf, pub_qos)
        self.grad_pub = self.create_publisher(MarkerArray, out_grad, pub_qos)

        self._frame_id = 'lidar_odom'
        self._first_cb = True

        self.get_logger().info(
            f"EsdfGradViz: in={in_topic}, esdf_out={out_esdf}, grad_out={out_grad}, "
            f"max_dist={max_dist}, sigma={smooth_sigma}, arrow_step={self.arrow_step}"
        )

    def on_costmap(self, msg: OccupancyGrid):
        self._frame_id = msg.header.frame_id
        self.esdf_map.update(
            grid_data=msg.data,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            frame_id=msg.header.frame_id,
        )

        self._publish_esdf_cloud(msg.header.stamp)
        self._publish_grad_markers(msg.header.stamp)

        if self._first_cb and self.log_samples:
            self._log_sample_queries()
            self._first_cb = False

    def _publish_esdf_cloud(self, stamp):
        """Publish signed ESDF as CUBE_LIST color blocks (each cell = one cube)."""
        esdf = self.esdf_map.get_signed_esdf_array()
        H, W = esdf.shape
        res = self.esdf_map.resolution
        ox, oy = self.esdf_map.origin_x, self.esdf_map.origin_y

        markers = MarkerArray()

        # Delete previous
        del_m = Marker()
        del_m.header.frame_id = self._frame_id
        del_m.header.stamp = stamp
        del_m.action = Marker.DELETEALL
        markers.markers.append(del_m)

        cube = Marker()
        cube.header.frame_id = self._frame_id
        cube.header.stamp = stamp
        cube.ns = "signed_esdf"
        cube.id = 0
        cube.type = Marker.CUBE_LIST
        cube.action = Marker.ADD
        cube.scale = Vector3(x=res * 0.95, y=res * 0.95, z=0.02)
        cube.pose.orientation.w = 1.0

        for row in range(H):
            for col in range(W):
                wx = ox + (col + 0.5) * res
                wy = oy + (row + 0.5) * res
                d = esdf[row, col]

                # Color map: free=blue (d>0), obstacle=red (d<0), boundary=white
                # Blue intensity proportional to distance in free space
                # Red intensity proportional to depth in obstacle
                if d > 0.05:
                    # Free: blue → cyan → white as distance increases
                    t = min(d / 2.0, 1.0)
                    color = ColorRGBA(r=0.2 * t, g=0.4 * t, b=1.0, a=0.5)
                elif d < -0.05:
                    # Obstacle: red, darker as deeper inside
                    t = min(-d / 1.0, 1.0)
                    color = ColorRGBA(r=1.0, g=0.1 * (1 - t), b=0.1 * (1 - t), a=0.7)
                else:
                    # Boundary: yellow
                    color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)

                cube.points.append(Point(x=float(wx), y=float(wy), z=0.0))
                cube.colors.append(color)

        markers.markers.append(cube)
        self.esdf_pub.publish(markers)

    def _publish_grad_markers(self, stamp):
        """Publish gradient arrows on a subsampled grid as MarkerArray.

        Uses individual ARROW markers (with arrowhead shape), unified yellow color.
        """
        gx, gy = self.esdf_map.get_gradient_arrays()
        esdf = self.esdf_map.get_signed_esdf_array()
        H, W = esdf.shape
        res = self.esdf_map.resolution
        ox, oy = self.esdf_map.origin_x, self.esdf_map.origin_y

        markers = MarkerArray()

        # Delete-all marker first
        delete_marker = Marker()
        delete_marker.header.frame_id = self._frame_id
        delete_marker.header.stamp = stamp
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        step = self.arrow_step
        arrow_id = 0
        arrow_color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.9)  # bright yellow

        for row in range(0, H, step):
            for col in range(0, W, step):
                wx = ox + (col + 0.5) * res
                wy = oy + (row + 0.5) * res
                d = esdf[row, col]
                dgx = gx[row, col]
                dgy = gy[row, col]

                gmag = np.hypot(dgx, dgy)
                if gmag < 1e-6:
                    continue

                # Unit gradient direction
                ux = dgx / gmag
                uy = dgy / gmag
                angle = math.atan2(uy, ux)

                arrow = Marker()
                arrow.header.frame_id = self._frame_id
                arrow.header.stamp = stamp
                arrow.ns = "esdf_gradient"
                arrow.id = arrow_id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                # ARROW scale: x=shaft length, y=shaft diameter, z=head diameter
                arrow.scale = Vector3(
                    x=self.arrow_scale,
                    y=0.03,
                    z=0.06,
                )
                arrow.pose.position = Point(x=float(wx), y=float(wy), z=0.05)
                arrow.pose.orientation = Quaternion(
                    x=0.0,
                    y=0.0,
                    z=float(math.sin(angle / 2.0)),
                    w=float(math.cos(angle / 2.0)),
                )
                arrow.color = arrow_color

                markers.markers.append(arrow)
                arrow_id += 1

        self.grad_pub.publish(markers)

    def _log_sample_queries(self):
        """Log sample (d, grad) at key world points for verification."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("ESDF sample queries (world → d_signed, grad):")
        self.get_logger().info(f"  Grid: {self.esdf_map.width}x{self.esdf_map.height} "
                               f"res={self.esdf_map.resolution} "
                               f"origin=({self.esdf_map.origin_x:.1f},{self.esdf_map.origin_y:.1f})")

        # Robot origin (0,0) — should be free, positive distance
        # Near obstacle — check a few points
        # Inside obstacle — find an occupied cell and query it
        esdf = self.esdf_map.get_signed_esdf_array()
        H, W = esdf.shape
        res = self.esdf_map.resolution
        ox, oy = self.esdf_map.origin_x, self.esdf_map.origin_y

        test_points = [
            (0.0, 0.0, "robot origin"),
            (1.0, 0.0, "1m ahead"),
            (-1.0, 0.0, "1m behind"),
            (0.0, 2.0, "2m left"),
            (0.0, -2.0, "2m right"),
        ]

        # Find an obstacle cell for inside-obstacle test
        grid_data = esdf  # just use the array to find negative regions
        neg_indices = np.argwhere(esdf < -0.05)
        if len(neg_indices) > 0:
            mid_idx = neg_indices[len(neg_indices) // 2]
            obs_wx = ox + (mid_idx[1] + 0.5) * res
            obs_wy = oy + (mid_idx[0] + 0.5) * res
            test_points.append((float(obs_wx), float(obs_wy), "inside obstacle"))

        for wx, wy, label in test_points:
            d, gx, gy = self.esdf_map.get_distance_and_gradient(wx, wy)
            gmag = np.hypot(gx, gy)
            sign = "FREE" if d > 0 else ("OBSTACLE" if d < 0 else "BOUNDARY")
            self.get_logger().info(
                f"  ({wx:+.2f}, {wy:+.2f}) [{label}]: "
                f"d_signed={d:+.3f}m ({sign}), "
                f"grad=({gx:+.4f}, {gy:+.4f}), |grad|={gmag:.4f}"
            )

        # Statistics
        free_count = int(np.sum(esdf > 0.05))
        occ_count = int(np.sum(esdf < -0.05))
        boundary_count = int(np.sum(np.abs(esdf) <= 0.05))
        self.get_logger().info(
            f"  Stats: free={free_count}, obstacle={occ_count}, boundary={boundary_count}, "
            f"total={H*W}"
        )
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    node = EsdfGradVizNode()
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
