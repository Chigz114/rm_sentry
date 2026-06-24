"""
Global relocalization node — publishes map → lidar_odom TF.

Strategy (Phase A: birth-zone seed + ICP refinement):
  1. On startup, publish a static map → lidar_odom TF from seed pose
     (birth zone position in field coordinates).
  2. Accumulate N scans of /cloud_registered (in lidar_odom frame).
  3. Parse the Gazebo world file to build a 2D prior point cloud (in map frame).
  4. Run 2D ICP to refine the seed transform.
  5. Publish the refined map → lidar_odom TF continuously.

Subscribes:
  /cloud_registered (sensor_msgs/PointCloud2) — FAST-LIO registered cloud

Publishes:
  TF: map → lidar_odom

Parameters:
  seed_x, seed_y, seed_yaw — birth zone pose in map (world) frame
  world_file — path to Gazebo .world file for prior generation
  accumulate_count — number of scans to accumulate before ICP (default 20)
  icp_max_iter — ICP max iterations (default 50)
  icp_tol — ICP convergence tolerance (default 1e-4)
  icp_max_dist — ICP max correspondence distance in meters (default 1.0)
  voxel_size — downsample voxel size for accumulated cloud (default 0.1)
  refine_with_icp — whether to run ICP after seed (default True)
  tf_rate_hz — TF publishing rate (default 50.0)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import xml.etree.ElementTree as ET


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RelocalizationNode(Node):
    def __init__(self):
        super().__init__('relocalization_node')

        # Parameters
        self.declare_parameter('seed_x', 6.0)
        self.declare_parameter('seed_y', 4.0)
        self.declare_parameter('seed_yaw', 0.0)
        self.declare_parameter('world_file', '')
        self.declare_parameter('accumulate_count', 20)
        self.declare_parameter('icp_max_iter', 50)
        self.declare_parameter('icp_tol', 1e-4)
        self.declare_parameter('icp_max_dist', 2.0)
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('refine_with_icp', True)
        self.declare_parameter('tf_rate_hz', 50.0)
        self.declare_parameter('cloud_topic', '/cloud_registered')

        self.seed_x = self.get_parameter('seed_x').value
        self.seed_y = self.get_parameter('seed_y').value
        self.seed_yaw = self.get_parameter('seed_yaw').value
        self.world_file = self.get_parameter('world_file').value
        self.accumulate_count = int(self.get_parameter('accumulate_count').value)
        self.icp_max_iter = int(self.get_parameter('icp_max_iter').value)
        self.icp_tol = float(self.get_parameter('icp_tol').value)
        self.icp_max_dist = float(self.get_parameter('icp_max_dist').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.refine_with_icp = self.get_parameter('refine_with_icp').value
        tf_rate = float(self.get_parameter('tf_rate_hz').value)
        cloud_topic = self.get_parameter('cloud_topic').value

        # State
        self._accumulated_points = []
        self._cloud_count = 0
        self._icp_done = False
        self._prior_pc = None

        # Current transform: map ← lidar_odom (3x3 2D homogeneous)
        self._T_map_lidar = self._seed_to_matrix(
            self.seed_x, self.seed_y, self.seed_yaw)

        # Load prior from world file
        if self.world_file:
            self._prior_pc = self._load_prior_from_world(self.world_file)
            self.get_logger().info(
                f'Prior point cloud: {len(self._prior_pc)} points from {self.world_file}')
        else:
            self.get_logger().warn('No world_file specified, ICP disabled')
            self.refine_with_icp = False

        # TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_timer = self.create_timer(1.0 / tf_rate, self._publish_tf)

        # Status publisher (for monitoring ICP progress)
        self._status_pub = self.create_publisher(String, '/relocalization/status', 10)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        # Cloud subscription for ICP
        if self.refine_with_icp and self._prior_pc is not None:
            qos_sensor = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=5,
            )
            self._cloud_sub = self.create_subscription(
                PointCloud2, cloud_topic, self._on_cloud, qos_sensor)
            self.get_logger().info(
                f'Relocalization: seed=({self.seed_x:.2f}, {self.seed_y:.2f}, '
                f'{math.degrees(self.seed_yaw):.1f}°), accumulating {self.accumulate_count} scans for ICP')
        else:
            self.get_logger().info(
                f'Relocalization: seed-only mode, seed=({self.seed_x:.2f}, '
                f'{self.seed_y:.2f}, {math.degrees(self.seed_yaw):.1f}°)')

    def _publish_status(self):
        msg = String()
        if self._icp_done:
            x = self._T_map_lidar[0, 2]
            y = self._T_map_lidar[1, 2]
            yaw = math.atan2(self._T_map_lidar[1, 0], self._T_map_lidar[0, 0])
            msg.data = (f'ICP_DONE: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}° '
                        f'(seed was x={self.seed_x:.3f} y={self.seed_y:.3f} '
                        f'yaw={math.degrees(self.seed_yaw):.1f}°)')
        elif self.refine_with_icp and self._prior_pc is not None:
            msg.data = (f'ACCUMULATING: {self._cloud_count}/{self.accumulate_count} '
                        f'(seed x={self.seed_x:.3f} y={self.seed_y:.3f} '
                        f'yaw={math.degrees(self.seed_yaw):.1f}°)')
        else:
            msg.data = 'SEED_ONLY (no ICP)'
        self._status_pub.publish(msg)

    # ── TF publishing ──────────────────────────────────────────

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'lidar_odom'
        t.transform.translation.x = self._T_map_lidar[0, 2]
        t.transform.translation.y = self._T_map_lidar[1, 2]
        t.transform.translation.z = 0.0
        # Convert yaw to quaternion
        yaw = math.atan2(self._T_map_lidar[1, 0], self._T_map_lidar[0, 0])
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_broadcaster.sendTransform(t)

    # ── Cloud accumulation + ICP ───────────────────────────────

    def _on_cloud(self, msg: PointCloud2):
        if self._icp_done:
            return

        # Extract XYZ points and filter to obstacle-like points (z > 0.1m)
        raw_pts = np.array([
            (p[0], p[1], p[2]) for p in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ], dtype=np.float64)

        if len(raw_pts) < 10:
            return

        # Keep only points above ground (obstacles, walls, etc.)
        obstacle_mask = raw_pts[:, 2] > 0.1
        pts = raw_pts[obstacle_mask][:, :2]

        if len(pts) < 10:
            return

        self._accumulated_points.append(pts)
        self._cloud_count += 1

        if self._cloud_count % 5 == 0:
            self.get_logger().info(
                f'Accumulating scans: {self._cloud_count}/{self.accumulate_count}')

        if self._cloud_count >= self.accumulate_count:
            self._run_icp()

    def _run_icp(self):
        self.get_logger().info('Running 2D ICP refinement...')

        # Merge and downsample accumulated cloud
        all_pts = np.vstack(self._accumulated_points)
        all_pts = self._voxel_downsample_2d(all_pts, self.voxel_size)
        self.get_logger().info(
            f'Accumulated cloud: {len(all_pts)} points after voxel filter')
        self.get_logger().info(
            f'Source (lidar_odom) range: x=[{all_pts[:,0].min():.2f},{all_pts[:,0].max():.2f}] '
            f'y=[{all_pts[:,1].min():.2f},{all_pts[:,1].max():.2f}]')
        self.get_logger().info(
            f'Target (prior/map) range: x=[{self._prior_pc[:,0].min():.2f},{self._prior_pc[:,0].max():.2f}] '
            f'y=[{self._prior_pc[:,1].min():.2f},{self._prior_pc[:,1].max():.2f}]')

        # Initial transform from seed
        T = self._T_map_lidar.copy()

        # Run ICP
        T_refined, final_error, n_iter = self._icp_2d(
            all_pts, self._prior_pc, T,
            max_iter=self.icp_max_iter,
            tol=self.icp_tol,
            max_dist=self.icp_max_dist,
        )

        # Compute improvement
        delta = np.linalg.norm(
            T_refined[:2, 2] - T[:2, 2])
        yaw_refined = math.atan2(T_refined[1, 0], T_refined[0, 0])
        delta_yaw = abs(yaw_refined - self.seed_yaw)

        self.get_logger().info(
            f'ICP converged in {n_iter} iterations: '
            f'translation delta={delta:.4f}m, yaw delta={math.degrees(delta_yaw):.2f}°, '
            f'RMSE={final_error:.4f}m')

        # Sanity check: reject if ICP moved too far from seed (> 2m or > 30°)
        if delta > 2.0 or delta_yaw > math.radians(30):
            self.get_logger().warn(
                f'ICP result too far from seed (delta={delta:.2f}m, '
                f'{math.degrees(delta_yaw):.1f}°), keeping seed transform')
        else:
            self._T_map_lidar = T_refined
            self.get_logger().info('ICP refinement accepted')

        self._icp_done = True
        # Stop subscribing to cloud
        self.destroy_subscription(self._cloud_sub)

    # ── ICP implementation ─────────────────────────────────────

    def _icp_2d(self, source, target, init_T, max_iter=50, tol=1e-4, max_dist=1.0):
        """2D point-to-point ICP.

        Args:
            source: (N, 2) points in lidar_odom frame
            target: (M, 2) points in map frame (prior)
            init_T: (4, 4) initial transform mapping source → target
            max_iter: max iterations
            tol: convergence tolerance on transform change
            max_dist: max correspondence distance

        Returns:
            T_refined (4x4), final_rmse, n_iterations
        """
        from scipy.spatial import cKDTree

        T = init_T.copy()
        tree = cKDTree(target)

        prev_error = float('inf')

        for i in range(max_iter):
            # Transform source points (3x3 2D homogeneous)
            src_h = np.hstack([source, np.ones((len(source), 1))])
            src_t = (T @ src_h.T).T[:, :2]

            # Find nearest neighbors
            dists, idxs = tree.query(src_t, k=1)

            # Filter by max distance
            mask = dists < max_dist
            if mask.sum() < 10:
                self.get_logger().warn(
                    f'ICP iter {i}: only {mask.sum()} correspondences, stopping')
                break

            src_orig_f = source[mask]
            tgt_f = target[idxs[mask]]

            # Compute full transform (original source → target) via Umeyama 2D.
            # Correspondences are found using transformed points (src_t) but the
            # fit uses original points, so T_new is the complete lidar_odom→map
            # transform and can replace T directly.
            T_new = self._umeyama_2d(src_orig_f, tgt_f)

            # Check convergence
            delta = np.linalg.norm(T_new[:2, 2] - T[:2, 2])
            T = T_new

            # Compute RMSE
            src_h2 = np.hstack([source, np.ones((len(source), 1))])
            src_t2 = (T @ src_h2.T).T[:, :2]
            dists2, _ = tree.query(src_t2, k=1)
            rmse = np.sqrt(np.mean(dists2 ** 2))

            if abs(prev_error - rmse) < tol:
                self.get_logger().info(
                    f'ICP converged at iter {i}: RMSE={rmse:.4f}')
                return T, rmse, i + 1

            prev_error = rmse

        return T, prev_error, max_iter

    @staticmethod
    def _umeyama_2d(src, dst):
        """Compute optimal 2D rigid transform (rotation + translation)
        mapping src → dst using SVD.

        Args:
            src: (N, 2) source points
            dst: (N, 2) target points

        Returns:
            3x3 2D homogeneous transform
        """
        mu_src = src.mean(axis=0)
        mu_dst = dst.mean(axis=0)

        src_c = src - mu_src
        dst_c = dst - mu_dst

        # 2x2 covariance
        H = src_c.T @ dst_c / len(src)

        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Handle reflection
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = mu_dst - R @ mu_src

        T = np.eye(3)
        T[:2, :2] = R
        T[:2, 2] = t
        return T

    # ── Prior generation from world file ───────────────────────

    def _load_prior_from_world(self, world_file):
        """Parse Gazebo world file and generate 2D prior point cloud.

        Samples points along the perimeter of each box obstacle's footprint
        at a fixed resolution. Returns (N, 2) array in map (world) coordinates.
        """
        tree = ET.parse(world_file)
        root = tree.getroot()

        # Find world element
        world_elem = root.find('world')
        if world_elem is None:
            self.get_logger().error('No <world> element in world file')
            return np.zeros((0, 2))

        skip_names = {'spawn', 'control', 'light', 'floor'}
        sample_res = 0.1  # meters between sampled points
        points = []

        for model in world_elem.findall('model'):
            name = model.get('name', '')
            if any(s in name.lower() for s in skip_names):
                continue

            pose_elem = model.find('pose')
            link = model.find('link')
            if pose_elem is None or link is None:
                continue

            # Parse pose: "x y z roll pitch yaw"
            pose_vals = pose_elem.text.strip().split()
            px = float(pose_vals[0])
            py = float(pose_vals[1])

            # Find box geometry
            collision = link.find('collision')
            if collision is None:
                continue
            geometry = collision.find('geometry')
            if geometry is None:
                continue
            box = geometry.find('box')
            if box is None:
                continue
            size_elem = box.find('size')
            if size_elem is None:
                continue

            size_vals = size_elem.text.strip().split()
            sx = float(size_vals[0])
            sy = float(size_vals[1])

            # Sample points filling the box footprint (interior + perimeter)
            hx, hy = sx / 2.0, sy / 2.0
            n_x = max(2, int(sx / sample_res))
            n_y = max(2, int(sy / sample_res))
            for i in range(n_x + 1):
                for j in range(n_y + 1):
                    x = px - hx + i * sx / n_x
                    y = py - hy + j * sy / n_y
                    points.append((x, y))

        return np.array(points, dtype=np.float64)

    # ── Utilities ──────────────────────────────────────────────

    @staticmethod
    def _seed_to_matrix(x, y, yaw):
        """Build 3x3 2D homogeneous transform from seed pose."""
        T = np.eye(3)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        T[0, 0] = cos_y
        T[0, 1] = -sin_y
        T[1, 0] = sin_y
        T[1, 1] = cos_y
        T[0, 2] = x
        T[1, 2] = y
        return T

    @staticmethod
    def _voxel_downsample_2d(points, voxel_size):
        """Downsample 2D points using a voxel grid."""
        keys = np.floor(points / voxel_size).astype(np.int64)
        # Use unique keys to find one representative per voxel
        _, idx = np.unique(keys, axis=0, return_index=True)
        return points[idx]


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
