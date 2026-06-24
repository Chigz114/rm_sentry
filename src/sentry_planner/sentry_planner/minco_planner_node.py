"""
MINCO planner node — Stage 3.4

Subscribes:
  /planner/path          (nav_msgs/Path) — JPS raw path
  /Odometry              (Odometry) — robot current state for boundary conditions
  /perception/costmap_2d (OccupancyGrid) — for ESDF (obstacle cost, step 5)

Publishes:
  /planner/path_vis      (nav_msgs/Path) — dense smooth trajectory for RViz
  /planner/minco_traj    (MarkerArray) — trajectory + waypoints visualization
  /planner/minco_info    (Marker) — text info (cost, max_v, max_a)

Pipeline:
  1. Receive JPS path
  2. Post-process: prune → spacing → time allocation
  3. MINCO optimize (smoothness + time, no obstacle cost for step 4)
  4. Sample trajectory and publish for visualization

Usage:
  ros2 run sentry_planner minco_planner_node
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from .esdf_map_2d import EsdfMap2D
from .path_postprocess import postprocess_jps_path
from .minco_solver_2d import MincoSolver2D


class MincoPlannerNode(Node):
    def __init__(self):
        super().__init__('minco_planner_node')

        # Parameters
        self.declare_parameter('path_topic', '/planner/path')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('costmap_topic', '/perception/costmap_2d')
        self.declare_parameter('path_vis_topic', '/planner/path_vis')
        self.declare_parameter('traj_samples_topic', '/planner/traj_samples')
        self.declare_parameter('traj_marker_topic', '/planner/minco_traj')
        self.declare_parameter('info_marker_topic', '/planner/minco_info')
        self.declare_parameter('v_max', 4.0)
        self.declare_parameter('a_max', 4.0)
        self.declare_parameter('v_alloc', 2.0)   # conservative velocity for time allocation
        self.declare_parameter('w_smooth', 1.0)
        self.declare_parameter('w_time', 50.0)
        self.declare_parameter('w_obs', 0.0)
        self.declare_parameter('w_collision', 0.0)
        self.declare_parameter('d_soft', 0.36)
        self.declare_parameter('d_hard', 0.28)
        self.declare_parameter('w_ref', 0.0)
        self.declare_parameter('waypoint_bound_m', 0.0)
        self.declare_parameter('w_dyn', 100.0)
        self.declare_parameter('min_spacing', 1.5)
        self.declare_parameter('max_spacing', 5.0)
        self.declare_parameter('sample_dt', 0.05)
        self.declare_parameter('max_iter', 200)
        self.declare_parameter('t_min', 0.5)

        path_topic = self.get_parameter('path_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value
        self.path_vis_topic = self.get_parameter('path_vis_topic').value
        self.traj_samples_topic = self.get_parameter('traj_samples_topic').value
        self.traj_marker_topic = self.get_parameter('traj_marker_topic').value
        self.info_marker_topic = self.get_parameter('info_marker_topic').value
        v_max = float(self.get_parameter('v_max').value)
        a_max = float(self.get_parameter('a_max').value)
        w_smooth = float(self.get_parameter('w_smooth').value)
        w_time = float(self.get_parameter('w_time').value)
        w_obs = float(self.get_parameter('w_obs').value)
        w_collision = float(self.get_parameter('w_collision').value)
        d_soft = float(self.get_parameter('d_soft').value)
        d_hard = float(self.get_parameter('d_hard').value)
        w_ref = float(self.get_parameter('w_ref').value)
        waypoint_bound_m = float(self.get_parameter('waypoint_bound_m').value)
        w_dyn = float(self.get_parameter('w_dyn').value)
        self.min_spacing = float(self.get_parameter('min_spacing').value)
        self.max_spacing = float(self.get_parameter('max_spacing').value)
        self.sample_dt = float(self.get_parameter('sample_dt').value)
        self.max_iter = int(self.get_parameter('max_iter').value)
        self.t_min = float(self.get_parameter('t_min').value)
        self.v_alloc = float(self.get_parameter('v_alloc').value)

        self.solver = MincoSolver2D(
            v_max=v_max, a_max=a_max,
            w_smooth=w_smooth, w_time=w_time, w_obs=w_obs, w_dyn=w_dyn,
            w_collision=w_collision, d_soft=d_soft, d_hard=d_hard,
            w_ref=w_ref, waypoint_bound_m=waypoint_bound_m,
        )

        self.esdf_map = EsdfMap2D(max_distance_m=5.0, smooth_sigma=1.0)

        # State
        self._odom_received = False
        self._current_vel = (0.0, 0.0)
        self._current_acc = (0.0, 0.0)
        self._frame_id = 'lidar_odom'

        # QoS
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        self.path_sub = self.create_subscription(Path, path_topic, self.on_path, sub_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, sub_qos)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, costmap_topic, self.on_costmap, sub_qos
        )

        self.path_vis_pub = self.create_publisher(Path, self.path_vis_topic, pub_qos)
        self.traj_samples_pub = self.create_publisher(
            Float64MultiArray, self.traj_samples_topic, pub_qos)
        self.traj_marker_pub = self.create_publisher(MarkerArray, self.traj_marker_topic, pub_qos)
        self.info_marker_pub = self.create_publisher(Marker, self.info_marker_topic, pub_qos)

        self.get_logger().info(
            f"MincoPlannerNode: path={path_topic}, odom={odom_topic}, "
            f"v_max={v_max}, w_smooth={w_smooth}, w_time={w_time}, w_obs={w_obs}, "
            f"w_collision={w_collision}, d_soft={d_soft}, d_hard={d_hard}, "
            f"w_ref={w_ref}, waypoint_bound={waypoint_bound_m}"
        )

    def on_odom(self, msg: Odometry):
        self._current_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._odom_received = True
        self._frame_id = msg.header.frame_id

    def on_costmap(self, msg: OccupancyGrid):
        self.esdf_map.update(
            grid_data=msg.data,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            frame_id=msg.header.frame_id,
        )

    def on_path(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn("Path too short, skipping")
            return

        # Extract waypoints from Path message
        raw_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._frame_id = msg.header.frame_id

        # Step 1: Post-process JPS path
        waypoints, durations = postprocess_jps_path(
            raw_points,
            v_max=self.v_alloc,
            min_spacing=self.min_spacing,
            max_spacing=self.max_spacing,
            t_min=self.t_min,
        )

        if len(waypoints) < 2:
            self.get_logger().warn("Post-processed path too short")
            return

        # Step 2: Set boundary conditions from current odom
        boundary = {
            'start_vel': self._current_vel if self._odom_received else (0.0, 0.0),
            'start_acc': self._current_acc,
            'end_vel': (0.0, 0.0),
            'end_acc': (0.0, 0.0),
        }

        # Step 3: Inject ESDF and MINCO optimize
        self.solver.set_esdf(self.esdf_map)
        _t_opt0 = time.perf_counter()
        opt_wps, opt_durations, cx, cy, cost = self.solver.optimize(
            waypoints, durations, boundary=boundary, max_iter=self.max_iter
        )
        _opt_ms = (time.perf_counter() - _t_opt0) * 1000.0
        max_shift, avg_shift = self._waypoint_shift_stats(waypoints, opt_wps)

        # Step 3b: Final collision check
        min_d, min_d_x, min_d_y = self.solver.check_clearance(cx, cy, opt_durations)
        d_hard = self.solver.d_hard
        if min_d < d_hard:
            self.get_logger().warn(
                f"MINCO final check FAILED: min_d={min_d:.3f} < d_hard={d_hard:.3f} "
                f"at ({min_d_x:.2f}, {min_d_y:.2f}), falling back to JPS path"
            )
            # Fall back to unoptimized JPS waypoints
            opt_wps = waypoints
            cx, cy = self.solver.compute_coefficients(opt_wps, opt_durations, boundary)
            cost = self.solver.total_cost(cx, cy, opt_durations)
            min_d, min_d_x, min_d_y = self.solver.check_clearance(cx, cy, opt_durations)
            self.get_logger().warn(
                f"JPS fallback min_d={min_d:.3f} at ({min_d_x:.2f}, {min_d_y:.2f})"
            )
        else:
            self.get_logger().info(
                f"MINCO final check OK: min_d={min_d:.3f} >= d_hard={d_hard:.3f}"
            )

        # Step 4: Check dynamics
        max_v, max_a, v_viol, a_viol = self.solver.check_dynamics(cx, cy, opt_durations)

        self.get_logger().info(
            f"MINCO: {len(raw_points)} JPS pts → {len(opt_wps)} wps, "
            f"{len(opt_durations)} segs, T={sum(opt_durations):.2f}s, "
            f"cost={cost:.4f}, max_v={max_v:.2f}, max_a={max_a:.2f}, "
            f"v_viol={v_viol}, a_viol={a_viol}, min_d={min_d:.3f}, "
            f"wp_shift=max/avg {max_shift:.2f}/{avg_shift:.2f}m, "
            f"opt_time={_opt_ms:.1f}ms"
        )

        # Step 5: Publish visualization
        samples = self.solver.sample_trajectory(cx, cy, opt_durations, dt=self.sample_dt)
        self._publish_path_vis(samples, msg.header.stamp)
        self._publish_traj_samples(samples)
        self._publish_traj_markers(opt_wps, cx, cy, opt_durations, msg.header.stamp)
        self._publish_info(cost, max_v, max_a, v_viol, a_viol,
                          sum(opt_durations), len(opt_wps), msg.header.stamp)

    def _waypoint_shift_stats(self, ref_wps, opt_wps):
        if len(ref_wps) != len(opt_wps) or len(ref_wps) <= 2:
            return 0.0, 0.0
        shifts = []
        for ref, opt in zip(ref_wps[1:-1], opt_wps[1:-1]):
            shifts.append(math.hypot(opt[0] - ref[0], opt[1] - ref[1]))
        if not shifts:
            return 0.0, 0.0
        return max(shifts), sum(shifts) / len(shifts)

    def _publish_path_vis(self, samples, stamp):
        """Publish dense smooth trajectory as nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.frame_id = self._frame_id
        path_msg.header.stamp = stamp

        for t, x, y, vx, vy, ax, ay, yaw in samples:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            # Orientation from velocity direction
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=float(qz), w=float(qw))
            path_msg.poses.append(pose)

        self.path_vis_pub.publish(path_msg)

    def _publish_traj_samples(self, samples):
        """Publish sampled trajectory as Float64MultiArray rows.

        Layout: N rows x 8 columns:
          [t, x, y, vx, vy, ax, ay, yaw]

        This is the first control-facing trajectory interface. It intentionally
        stays simple until a formal Trajectory2D msg package is introduced.
        """
        msg = Float64MultiArray()
        n = len(samples)
        msg.layout.dim = [
            MultiArrayDimension(label='samples', size=n, stride=n * 8),
            MultiArrayDimension(label='fields_t_x_y_vx_vy_ax_ay_yaw', size=8, stride=8),
        ]
        msg.layout.data_offset = 0
        data = []
        for t, x, y, vx, vy, ax, ay, yaw in samples:
            data.extend([
                float(t), float(x), float(y), float(vx), float(vy),
                float(ax), float(ay), float(yaw),
            ])
        msg.data = data
        self.traj_samples_pub.publish(msg)

    def _publish_traj_markers(self, waypoints, cx, cy, durations, stamp):
        """Publish trajectory markers: waypoints + trajectory line + velocity arrows."""
        markers = MarkerArray()

        # Delete previous
        del_m = Marker()
        del_m.header.frame_id = self._frame_id
        del_m.header.stamp = stamp
        del_m.action = Marker.DELETEALL
        markers.markers.append(del_m)

        # 1. Trajectory line (cyan)
        line = Marker()
        line.header.frame_id = self._frame_id
        line.header.stamp = stamp
        line.ns = "minco_traj"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale = Vector3(x=0.03, y=0.0, z=0.0)
        line.pose.orientation.w = 1.0
        line.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)

        samples = self.solver.sample_trajectory(cx, cy, durations, dt=self.sample_dt)
        for t, x, y, vx, vy, ax, ay, yaw in samples:
            line.points.append(Point(x=float(x), y=float(y), z=0.03))

        markers.markers.append(line)

        # 2. Waypoints (spheres: green=intermediate, yellow=start, red=end)
        for i, (wx, wy) in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = stamp
            m.ns = "minco_waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale = Vector3(x=0.12, y=0.12, z=0.12)
            m.pose.position = Point(x=float(wx), y=float(wy), z=0.1)
            m.pose.orientation.w = 1.0

            if i == 0:
                m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # yellow start
            elif i == len(waypoints) - 1:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # red end
            else:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # green intermediate

            markers.markers.append(m)

        # 3. Velocity arrows at sampled points (orange)
        arrow = Marker()
        arrow.header.frame_id = self._frame_id
        arrow.header.stamp = stamp
        arrow.ns = "minco_velocity"
        arrow.id = 0
        arrow.type = Marker.LINE_LIST
        arrow.action = Marker.ADD
        arrow.scale = Vector3(x=0.02, y=0.0, z=0.0)
        arrow.pose.orientation.w = 1.0
        arrow.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.5)

        for t, x, y, vx, vy, ax, ay, yaw in samples[::3]:  # every 3rd sample
            v_mag = math.hypot(vx, vy)
            if v_mag < 0.01:
                continue
            scale = 0.2 / max(v_mag, 0.1)  # normalize for visibility
            dx = vx * scale
            dy = vy * scale
            arrow.points.append(Point(x=float(x), y=float(y), z=0.04))
            arrow.points.append(Point(x=float(x + dx), y=float(y + dy), z=0.04))

        markers.markers.append(arrow)

        self.traj_marker_pub.publish(markers)

    def _publish_info(self, cost, max_v, max_a, v_viol, a_viol, total_T, n_wps, stamp):
        """Publish text info marker."""
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = stamp
        m.ns = "minco_info"
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.scale = Vector3(x=0.0, y=0.0, z=0.3)
        m.pose.position = Point(x=0.0, y=0.0, z=2.0)
        m.pose.orientation.w = 1.0
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)

        status = "OK" if v_viol == 0 and a_viol == 0 else "VIOLATION"
        m.text = (f"MINCO: {n_wps} wps, T={total_T:.2f}s\n"
                  f"cost={cost:.3f}, max_v={max_v:.2f}, max_a={max_a:.2f}\n"
                  f"v_viol={v_viol}, a_viol={a_viol} [{status}]")

        self.info_marker_pub.publish(m)


def main():
    rclpy.init()
    node = MincoPlannerNode()
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
