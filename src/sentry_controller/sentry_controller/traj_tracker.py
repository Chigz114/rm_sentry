"""
Time-parameterized trajectory tracker.

Subscribes:
  /planner/traj_samples (Float64MultiArray, rows [t,x,y,vx,vy,ax,ay,yaw])
  /odom                 (Odometry, Gazebo ground truth)

Publishes:
  /cmd_vel_chassis      (Twist, body frame)
"""
import bisect
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp_mag(x, y, limit):
    mag = math.hypot(x, y)
    if mag > limit > 0.0:
        scale = limit / mag
        return x * scale, y * scale
    return x, y


class TrajTracker(Node):
    def __init__(self):
        super().__init__('traj_tracker')

        self.declare_parameter('traj_topic', '/planner/traj_samples')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_chassis')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('acc_lim', 3.0)
        self.declare_parameter('kp_pos', 2.0)
        self.declare_parameter('kd_vel', 0.8)
        self.declare_parameter('k_yaw', 0.0)
        self.declare_parameter('w_max', 0.0)
        self.declare_parameter('goal_tol', 0.18)
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('progress_anchor', True)
        self.declare_parameter('lookahead_time', 0.25)
        self.declare_parameter('max_feedback_speed', 1.0)
        self.declare_parameter('track_slow_start', 0.25)
        self.declare_parameter('track_slow_stop', 0.80)
        self.declare_parameter('offtrack_v_max', 1.2)
        self.declare_parameter('debug_traj', True)
        self.declare_parameter('debug_csv_path', '/tmp/traj_tracker_debug.csv')

        traj_topic = self.get_parameter('traj_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        rate = float(self.get_parameter('rate_hz').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.acc_lim = float(self.get_parameter('acc_lim').value)
        self.kp_pos = float(self.get_parameter('kp_pos').value)
        self.kd_vel = float(self.get_parameter('kd_vel').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.time_scale = float(self.get_parameter('time_scale').value)
        self.progress_anchor = bool(self.get_parameter('progress_anchor').value)
        self.lookahead_time = float(self.get_parameter('lookahead_time').value)
        self.max_feedback_speed = float(self.get_parameter('max_feedback_speed').value)
        self.track_slow_start = float(self.get_parameter('track_slow_start').value)
        self.track_slow_stop = float(self.get_parameter('track_slow_stop').value)
        self.offtrack_v_max = float(self.get_parameter('offtrack_v_max').value)
        self.debug_traj = bool(self.get_parameter('debug_traj').value)
        self.debug_csv_path = str(self.get_parameter('debug_csv_path').value)

        self.dt = 1.0 / rate
        self.samples = []
        self.sample_times = []
        self.traj_start_time = None
        self.current_idx = 0
        self.traj_rcv = False
        self.at_goal = False

        self.rx = 0.0
        self.ry = 0.0
        self.yaw = 0.0
        self.vx_world = 0.0
        self.vy_world = 0.0
        self.odom_rcv = False

        self.last_vx_body = 0.0
        self.last_vy_body = 0.0
        self.last_wz = 0.0
        self.last_control_time = None

        self.debug_csv_file = None
        self.debug_csv = None
        if self.debug_csv_path:
            try:
                self.debug_csv_file = open(self.debug_csv_path, 'w', newline='')
                self.debug_csv = csv.writer(self.debug_csv_file)
                self.debug_csv.writerow([
                    'ros_time', 'control_dt', 't_ref', 'idx', 'nearest_d',
                    'err_x', 'err_y', 'err_mag',
                    'vref_x', 'vref_y', 'vref_mag',
                    'fb_x', 'fb_y', 'fb_mag',
                    'ff_scale', 'speed_cap',
                    'vworld_x', 'vworld_y', 'vworld_mag',
                    'cmd_x', 'cmd_y', 'cmd_mag',
                    'end_dist',
                ])
                self.debug_csv_file.flush()
            except OSError as exc:
                self.get_logger().warn(
                    f'Failed to open debug_csv_path={self.debug_csv_path}: {exc}')
                self.debug_csv_file = None
                self.debug_csv = None

        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_traj = self.create_subscription(
            Float64MultiArray, traj_topic, self._on_traj, qos_rel)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_sensor)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, qos_rel)
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'TrajTracker: traj={traj_topic} odom={odom_topic} '
            f'v_max={self.v_max} acc={self.acc_lim} '
            f'kp={self.kp_pos} kd={self.kd_vel} rate={rate}Hz '
            f'fb_cap={self.max_feedback_speed} offtrack_v={self.offtrack_v_max}')

    def _on_traj(self, msg):
        if len(msg.data) < 16 or len(msg.data) % 8 != 0:
            self.get_logger().warn(f'Invalid traj_samples length={len(msg.data)}')
            return
        rows = []
        data = msg.data
        for i in range(0, len(data), 8):
            rows.append(tuple(float(v) for v in data[i:i + 8]))
        if len(rows) < 2:
            return

        self.samples = rows
        self.sample_times = [row[0] for row in rows]
        self.traj_start_time = self.get_clock().now()
        self.last_control_time = None
        self.traj_rcv = True
        self.at_goal = False
        self.current_idx = 0

        if self.odom_rcv:
            cy = math.cos(self.yaw)
            sy = math.sin(self.yaw)
            self.last_vx_body = cy * self.vx_world + sy * self.vy_world
            self.last_vy_body = -sy * self.vx_world + cy * self.vy_world
        else:
            self.last_vx_body = 0.0
            self.last_vy_body = 0.0
        self.last_wz = 0.0

        if self.odom_rcv:
            self.current_idx, _ = self._find_nearest_idx(0)

        total_t = rows[-1][0]
        self.get_logger().info(
            f'New trajectory: {len(rows)} samples, T={total_t:.2f}s, '
            f'start=({rows[0][1]:.2f},{rows[0][2]:.2f}) '
            f'end=({rows[-1][1]:.2f},{rows[-1][2]:.2f}) '
            f'anchor_idx={self.current_idx}')

    def _on_odom(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

        vx_body = msg.twist.twist.linear.x
        vy_body = msg.twist.twist.linear.y
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        self.vx_world = cy * vx_body - sy * vy_body
        self.vy_world = sy * vx_body + cy * vy_body
        self.odom_rcv = True

    def _sample_ref(self, t):
        if not self.samples:
            return None
        if t <= self.sample_times[0]:
            return self.samples[0]
        if t >= self.sample_times[-1]:
            return self.samples[-1]
        j = bisect.bisect_right(self.sample_times, t)
        a = self.samples[j - 1]
        b = self.samples[j]
        ta = a[0]
        tb = b[0]
        u = (t - ta) / max(tb - ta, 1e-6)
        return tuple(a[k] + u * (b[k] - a[k]) for k in range(8))

    def _find_nearest_idx(self, start_idx):
        if not self.samples:
            return 0, float('inf')
        best_i = max(0, min(start_idx, len(self.samples) - 1))
        best_d = float('inf')
        # Search forward with a small look-behind. The robot should not move
        # backward along a newly received MINCO trajectory, but a look-behind
        # absorbs odom noise and command latency.
        lo = max(0, best_i - 8)
        for i in range(lo, len(self.samples)):
            _, x, y, *_ = self.samples[i]
            d = math.hypot(self.rx - x, self.ry - y)
            if d < best_d:
                best_d = d
                best_i = i
            elif i > best_i + 30 and d > best_d + 0.5:
                break
        return best_i, best_d

    def _control_loop(self):
        if not self.traj_rcv or not self.odom_rcv or self.traj_start_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.traj_start_time).nanoseconds * 1e-9
        if self.last_control_time is None:
            control_dt = self.dt
        else:
            control_dt = (now - self.last_control_time).nanoseconds * 1e-9
            if control_dt <= 0.0:
                control_dt = self.dt
        control_dt = max(1e-3, min(control_dt, 0.25))
        self.last_control_time = now

        if self.progress_anchor:
            self.current_idx, nearest_d = self._find_nearest_idx(self.current_idx)
            t_ref = min(
                self.sample_times[-1],
                self.sample_times[self.current_idx] + self.lookahead_time)
        else:
            nearest_d = 0.0
            t_ref = elapsed / max(self.time_scale, 1e-3)
        ref = self._sample_ref(t_ref)
        if ref is None:
            return

        _, px, py, vx_ref, vy_ref, _ax, _ay, yaw_ref = ref
        err_x = px - self.rx
        err_y = py - self.ry
        vel_err_x = vx_ref - self.vx_world
        vel_err_y = vy_ref - self.vy_world

        err_mag = math.hypot(err_x, err_y)
        slow_span = max(self.track_slow_stop - self.track_slow_start, 1e-3)
        ff_scale = 1.0 - max(
            0.0,
            min(1.0, (err_mag - self.track_slow_start) / slow_span))

        fb_x = self.kp_pos * err_x + self.kd_vel * vel_err_x
        fb_y = self.kp_pos * err_y + self.kd_vel * vel_err_y
        fb_x, fb_y = clamp_mag(fb_x, fb_y, self.max_feedback_speed)

        vx_world = ff_scale * vx_ref + fb_x
        vy_world = ff_scale * vy_ref + fb_y
        speed_cap = self.offtrack_v_max + ff_scale * (self.v_max - self.offtrack_v_max)
        vx_world, vy_world = clamp_mag(vx_world, vy_world, speed_cap)

        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        vx_body_des = cy * vx_world + sy * vy_world
        vy_body_des = -sy * vx_world + cy * vy_world

        dvx = vx_body_des - self.last_vx_body
        dvy = vy_body_des - self.last_vy_body
        max_dv = self.acc_lim * control_dt
        dvx, dvy = clamp_mag(dvx, dvy, max_dv)
        vx_cmd = self.last_vx_body + dvx
        vy_cmd = self.last_vy_body + dvy

        yaw_err = math.atan2(math.sin(yaw_ref - self.yaw), math.cos(yaw_ref - self.yaw))
        wz_cmd = 0.0
        if self.k_yaw > 0.0 and self.w_max > 0.0:
            wz_cmd = max(-self.w_max, min(self.w_max, self.k_yaw * yaw_err))

        end = self.samples[-1]
        end_dist = math.hypot(end[1] - self.rx, end[2] - self.ry)
        if end_dist < self.goal_tol:
            if not self.at_goal:
                self.get_logger().info(f'Reached trajectory end: dist={end_dist:.3f}m')
                self.at_goal = True
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        self._publish_cmd(vx_cmd, vy_cmd, wz_cmd)

        if self.debug_csv is not None:
            now_sec = now.nanoseconds * 1e-9
            self.debug_csv.writerow([
                f'{now_sec:.6f}', f'{control_dt:.4f}',
                f'{t_ref:.3f}', self.current_idx, f'{nearest_d:.4f}',
                f'{err_x:.4f}', f'{err_y:.4f}', f'{err_mag:.4f}',
                f'{vx_ref:.4f}', f'{vy_ref:.4f}', f'{math.hypot(vx_ref, vy_ref):.4f}',
                f'{fb_x:.4f}', f'{fb_y:.4f}', f'{math.hypot(fb_x, fb_y):.4f}',
                f'{ff_scale:.4f}', f'{speed_cap:.4f}',
                f'{vx_world:.4f}', f'{vy_world:.4f}', f'{math.hypot(vx_world, vy_world):.4f}',
                f'{vx_cmd:.4f}', f'{vy_cmd:.4f}', f'{math.hypot(vx_cmd, vy_cmd):.4f}',
                f'{end_dist:.4f}',
            ])
            self.debug_csv_file.flush()

        if self.debug_traj:
            self.get_logger().info(
                f't={t_ref:.2f}/{self.sample_times[-1]:.2f} '
                f'idx={self.current_idx}/{len(self.samples)-1} nd={nearest_d:.2f} '
                f'err=({err_x:.2f},{err_y:.2f}) '
                f'vref=({vx_ref:.2f},{vy_ref:.2f}) '
                f'fb=({fb_x:.2f},{fb_y:.2f}) ff={ff_scale:.2f} cap={speed_cap:.2f} '
                f'vworld=({vx_world:.2f},{vy_world:.2f}) '
                f'cmd=({vx_cmd:.2f},{vy_cmd:.2f}) dt={control_dt:.3f} end={end_dist:.2f}',
                throttle_duration_sec=0.3)

    def _publish_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub_cmd.publish(msg)
        self.last_vx_body = vx
        self.last_vy_body = vy
        self.last_wz = wz

    def destroy_node(self):
        if self.debug_csv_file is not None:
            self.debug_csv_file.close()
            self.debug_csv_file = None
            self.debug_csv = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
