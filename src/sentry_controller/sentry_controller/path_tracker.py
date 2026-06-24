"""
Frenet 轨迹追踪控制器 —— 阶段 4.0 Step 1-2

订阅：/planner/path_vis (nav_msgs/Path, MINCO 密集采样)
     /odom           (Odometry, Gazebo ground truth, position + yaw + twist)
发布：/cmd_vel_chassis (Twist → Gazebo)

行为：Frenet 投影 + 横向误差反馈 + 合速度/合加速度限幅 + 曲率自适应减速
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros
from tf2_ros import TransformException


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def dist2d(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        self.declare_parameter('v_max',       1.0)
        self.declare_parameter('w_max',       2.0)
        self.declare_parameter('k_p',         1.5)
        self.declare_parameter('k_yaw',       2.0)
        self.declare_parameter('acc_lim',     1.5)
        self.declare_parameter('acc_lim_yaw', 4.0)
        self.declare_parameter('switch_radius', 0.15)
        self.declare_parameter('goal_tol',    0.20)
        self.declare_parameter('lookahead_dist', 0.0)   # 0 = Frenet mode
        self.declare_parameter('cruise_speed', 1.0)
        self.declare_parameter('rate_hz',     30.0)
        self.declare_parameter('odom_topic',  '/odom')
        self.declare_parameter('chassis_odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_chassis')
        self.declare_parameter('path_topic',  '/planner/path_vis')
        self.declare_parameter('use_raw_odom', True)
        # Frenet controller parameters
        self.declare_parameter('kp_lat',      2.0)    # 横向位置误差增益
        self.declare_parameter('kd_lat',      0.5)    # 横向速度阻尼
        self.declare_parameter('max_lat_speed', 1.0)  # 横向纠偏速度上限 (m/s)
        self.declare_parameter('min_tracking_speed', 0.4)  # 低速纠偏保底速度
        self.declare_parameter('lat_slow_start', 0.20)  # 横向误差超过此值开始降低切向速度
        self.declare_parameter('lat_slow_stop', 0.70)   # 横向误差超过此值切向速度降到 0
        self.declare_parameter('a_lat_max',   1.0)    # 侧向加速度上限 (m/s²)
        self.declare_parameter('curvature_lookahead', 0.5)  # 曲率前瞻距离 (m)
        self.declare_parameter('brake_decel',  1.0)    # 制动减速度 (m/s²)
        self.declare_parameter('debug_frenet', False)

        self.v_max       = self.get_parameter('v_max').value
        self.w_max       = self.get_parameter('w_max').value
        self.k_p         = self.get_parameter('k_p').value
        self.k_yaw       = self.get_parameter('k_yaw').value
        self.acc_lim     = self.get_parameter('acc_lim').value
        self.acc_lim_yaw = self.get_parameter('acc_lim_yaw').value
        self.switch_r    = self.get_parameter('switch_radius').value
        self.goal_tol    = self.get_parameter('goal_tol').value
        self.lookahead   = self.get_parameter('lookahead_dist').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        odom_topic       = self.get_parameter('odom_topic').value
        chassis_odom_topic = self.get_parameter('chassis_odom_topic').value
        cmd_vel_topic    = self.get_parameter('cmd_vel_topic').value
        path_topic       = self.get_parameter('path_topic').value
        self.use_raw_odom = self.get_parameter('use_raw_odom').value
        rate             = self.get_parameter('rate_hz').value
        self.kp_lat      = self.get_parameter('kp_lat').value
        self.kd_lat      = self.get_parameter('kd_lat').value
        self.max_lat_speed = self.get_parameter('max_lat_speed').value
        self.min_tracking_speed = self.get_parameter('min_tracking_speed').value
        self.lat_slow_start = self.get_parameter('lat_slow_start').value
        self.lat_slow_stop = self.get_parameter('lat_slow_stop').value
        self.a_lat_max   = self.get_parameter('a_lat_max').value
        self.curv_la     = self.get_parameter('curvature_lookahead').value
        self.brake_decel = self.get_parameter('brake_decel').value
        self.debug_frenet = self.get_parameter('debug_frenet').value

        self.dt = 1.0 / rate

        self.rx = 0.0
        self.ry = 0.0
        self.odom_rcv = False
        self.chassis_yaw = 0.0
        self.chassis_rcv = False
        self.vx_world = 0.0
        self.vy_world = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_frame = 'map'
        self.odom_frame = 'lidar_odom'

        self.path: list = []
        self.speed_profile: list = []
        self.current_idx = 0
        self.path_rcv = False

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0
        self.at_goal = False

        qos = QoSProfile(
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

        self.sub_path = self.create_subscription(
            Path, path_topic, self._on_path, qos_sensor)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_sensor)
        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'PathTracker: v_max={self.v_max} cruise={self.cruise_speed} '
            f'kp_lat={self.kp_lat} kd_lat={self.kd_lat} '
            f'acc_lim={self.acc_lim} rate={rate}Hz path={path_topic}')

    def _on_path(self, msg: Path):
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(new_path) < 2:
            self.get_logger().warn(f'Path too short: {len(new_path)} pts')
            return
        self.path = new_path
        self.speed_profile = self._compute_speed_profile(new_path)
        self.path_rcv = True
        self.at_goal = False
        # 最近点锚定：找到离机器人最近的路径点作为起始追踪点
        if self.odom_rcv:
            # 新路径可能在车辆仍有速度时到来；限加速度状态应从真实速度开始，
            # 否则会从旧路径的上一条命令继续积分，导致换路瞬间横向甩动。
            cy = math.cos(self.chassis_yaw)
            sy = math.sin(self.chassis_yaw)
            self.last_vx = cy * self.vx_world + sy * self.vy_world
            self.last_vy = -sy * self.vx_world + cy * self.vy_world

            best_d = float('inf')
            for i, (px, py) in enumerate(self.path):
                d = dist2d(self.rx, self.ry, px, py)
                if d < best_d:
                    best_d = d
                    self.current_idx = i
        else:
            self.current_idx = 0
        self.get_logger().info(
            f'New path: {len(self.path)} pts, anchored idx={self.current_idx} '
            f'(dist={best_d:.2f}m)' if self.odom_rcv else
            f'New path: {len(self.path)} pts, start idx=0')
        if self.speed_profile:
            sample_ids = sorted(set([
                0,
                len(self.speed_profile) // 4,
                len(self.speed_profile) // 2,
                3 * len(self.speed_profile) // 4,
                len(self.speed_profile) - 1,
            ]))
            profile_text = ', '.join(
                f'{i}:{self.speed_profile[i]:.2f}' for i in sample_ids)
            self.get_logger().info(f'Speed profile samples: {profile_text}')

    def _on_odom(self, msg: Odometry):
        if self.use_raw_odom:
            self.rx = msg.pose.pose.position.x
            self.ry = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.chassis_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            # 速度反馈：twist 是 body frame (child_frame_id=base_link)，需转到 world frame
            cy = math.cos(self.chassis_yaw)
            sy = math.sin(self.chassis_yaw)
            vx_body = msg.twist.twist.linear.x
            vy_body = msg.twist.twist.linear.y
            self.vx_world = cy * vx_body - sy * vy_body
            self.vy_world = sy * vx_body + cy * vy_body
            self.odom_rcv = True
            self.chassis_rcv = True
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rclpy.time.Time())
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            t = tf.transform.translation
            q = tf.transform.rotation
            tf_yaw = 2.0 * math.atan2(q.z, q.w)
            cy = math.cos(tf_yaw)
            sy = math.sin(tf_yaw)
            self.rx = cy * px - sy * py + t.x
            self.ry = sy * px + cy * py + t.y
            # 机器人 heading in map frame = TF rotation + odom orientation
            odom_q = msg.pose.pose.orientation
            odom_yaw = euler_from_quaternion(odom_q.x, odom_q.y, odom_q.z, odom_q.w)
            self.chassis_yaw = tf_yaw + odom_yaw
            # twist is in body frame (child_frame_id=livox_frame), rotate body→lidar_odom→map
            vx_lo = math.cos(odom_yaw) * msg.twist.twist.linear.x - math.sin(odom_yaw) * msg.twist.twist.linear.y
            vy_lo = math.sin(odom_yaw) * msg.twist.twist.linear.x + math.cos(odom_yaw) * msg.twist.twist.linear.y
            self.vx_world = cy * vx_lo - sy * vy_lo
            self.vy_world = sy * vx_lo + cy * vy_lo
            self.odom_rcv = True
            self.chassis_rcv = True
        except TransformException:
            pass

    def _find_nearest_idx(self):
        """找到离机器人最近的路径点索引。"""
        best_d = float('inf')
        best_i = self.current_idx
        for i in range(self.current_idx, len(self.path)):
            d = dist2d(self.rx, self.ry, self.path[i][0], self.path[i][1])
            if d < best_d:
                best_d = d
                best_i = i
            elif d > best_d + 0.5:
                break
        return best_i, best_d

    def _path_curvature_at(self, idx):
        """三点圆近似曲率。"""
        if idx <= 0 or idx >= len(self.path) - 1:
            return 0.0
        p0 = self.path[idx - 1]
        p1 = self.path[idx]
        p2 = self.path[idx + 1]
        a = dist2d(p0[0], p0[1], p1[0], p1[1])
        b = dist2d(p1[0], p1[1], p2[0], p2[1])
        c = dist2d(p0[0], p0[1], p2[0], p2[1])
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            return 0.0
        s = (a + b + c) / 2.0
        area_sq = s * (s - a) * (s - b) * (s - c)
        if area_sq <= 1e-12:
            return 0.0
        return 4.0 * math.sqrt(area_sq) / (a * b * c)

    def _compute_speed_profile(self, path):
        """反向生成速度包络，保证能在弯道/终点前按 brake_decel 降速。"""
        n = len(path)
        if n == 0:
            return []
        point_limits = [min(self.cruise_speed, self.v_max) for _ in path]
        old_path = self.path
        self.path = path
        for i in range(n):
            k = abs(self._max_curvature_ahead(i, self.curv_la))
            if k > 1e-4:
                point_limits[i] = min(point_limits[i], math.sqrt(self.a_lat_max / k))
        self.path = old_path

        profile = point_limits[:]
        profile[-1] = 0.0
        for i in range(n - 2, -1, -1):
            seg = dist2d(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1])
            brake_limit = math.sqrt(profile[i + 1] ** 2 + 2.0 * self.brake_decel * seg)
            profile[i] = min(point_limits[i], brake_limit)
        return profile

    def _project_to_path(self, idx):
        """投影到路径上，返回 (ref_x, ref_y, tx, ty, nx, ny, e_n, curvature).

        t: 切向单位向量, n: 法向单位向量 (左转正)
        e_n: 横向误差 (正=机器人在路径左侧)
        curvature: 当前点曲率 (1/m)
        """
        if len(self.path) < 2:
            px, py = self.path[idx]
            return px, py, 1.0, 0.0, 0.0, 1.0, self.ry - py, 0.0

        # 在附近线段上做连续投影，避免用离散采样点导致横向误差跳动。
        start = max(0, idx - 3)
        end = min(len(self.path) - 2, idx + 3)
        best = None
        for i in range(start, end + 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            dx = bx - ax
            dy = by - ay
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < 1e-12:
                continue
            u = ((self.rx - ax) * dx + (self.ry - ay) * dy) / seg_len2
            u = max(0.0, min(1.0, u))
            px = ax + u * dx
            py = ay + u * dy
            d = dist2d(self.rx, self.ry, px, py)
            if best is None or d < best[0]:
                best = (d, i, px, py, dx, dy)

        if best is None:
            px, py = self.path[idx]
            tx, ty = 1.0, 0.0
            proj_idx = idx
        else:
            _, proj_idx, px, py, dx, dy = best
            seg_len = math.hypot(dx, dy)
            tx, ty = dx / seg_len, dy / seg_len

        # 法向 (左转正)
        nx, ny = -ty, tx
        # 横向误差
        rx_off = self.rx - px
        ry_off = self.ry - py
        e_n = rx_off * nx + ry_off * ny
        curvature = self._path_curvature_at(proj_idx)
        return px, py, tx, ty, nx, ny, e_n, curvature

    def _max_curvature_ahead(self, idx, lookahead_m):
        """返回前方 lookahead_m 内最大曲率。"""
        max_k = 0.0
        accum = 0.0
        for i in range(idx, len(self.path) - 1):
            seg = dist2d(self.path[i][0], self.path[i][1],
                         self.path[i+1][0], self.path[i+1][1])
            if accum > lookahead_m:
                break
            accum += seg
            if i > 0 and i < len(self.path) - 1:
                p0 = self.path[i - 1]
                p1 = self.path[i]
                p2 = self.path[i + 1]
                a = dist2d(p0[0], p0[1], p1[0], p1[1])
                b = dist2d(p1[0], p1[1], p2[0], p2[1])
                c = dist2d(p0[0], p0[1], p2[0], p2[1])
                if a > 1e-6 and b > 1e-6 and c > 1e-6:
                    s = (a + b + c) / 2.0
                    area_sq = s * (s - a) * (s - b) * (s - c)
                    if area_sq > 1e-12:
                        k = 4.0 * math.sqrt(area_sq) / (a * b * c)
                        if abs(k) > abs(max_k):
                            max_k = k
        return max_k

    def _brake_speed_limit(self, idx, v_target):
        """查询预计算速度包络，避免到弯心/终点才减速。"""
        if not self.speed_profile:
            return v_target
        idx = max(0, min(idx, len(self.speed_profile) - 1))
        return min(v_target, self.speed_profile[idx])

    def _control_loop(self):
        if not self.odom_rcv or not self.chassis_rcv or not self.path_rcv:
            return

        if len(self.path) == 0:
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        # 检查是否到达终点
        end_x, end_y = self.path[-1]
        end_dist = dist2d(self.rx, self.ry, end_x, end_y)
        if end_dist < self.goal_tol:
            if not self.at_goal:
                self.get_logger().info(
                    f'Reached final goal: dist={end_dist:.3f}m')
                self.at_goal = True
            self._publish_cmd(0.0, 0.0, 0.0)
            return
        self.at_goal = False

        # 找最近点并更新索引
        nearest_idx, nearest_d = self._find_nearest_idx()
        self.current_idx = nearest_idx

        # === Frenet 投影 ===
        ref_x, ref_y, tx, ty, nx, ny, e_n, curvature = self._project_to_path(nearest_idx)

        # === 速度规划 ===
        # 基础速度
        v_ref = self.cruise_speed
        # 终点减速
        if end_dist < 1.0:
            v_ref = min(v_ref, self.k_p * end_dist)
        # 曲率自适应限速
        k_ahead = self._max_curvature_ahead(nearest_idx, self.curv_la)
        if abs(k_ahead) > 1e-4:
            v_curv = math.sqrt(self.a_lat_max / abs(k_ahead))
            v_ref = min(v_ref, v_curv)
        # 制动距离约束
        v_ref = min(v_ref, self._brake_speed_limit(nearest_idx, v_ref))
        # 速度上限
        v_ref = min(v_ref, self.v_max)

        # === Frenet 控制律 (world frame) ===
        # 当前速度的法向分量
        v_n = self.vx_world * nx + self.vy_world * ny
        # 切向分量
        v_t = self.vx_world * tx + self.vy_world * ty

        # 横向误差过大时，停止“赶进度”，优先回到轨迹附近。
        e_abs = abs(e_n)
        if self.lat_slow_stop > self.lat_slow_start:
            if e_abs >= self.lat_slow_stop:
                v_ref *= 0.0
            elif e_abs > self.lat_slow_start:
                ratio = (self.lat_slow_stop - e_abs) / (self.lat_slow_stop - self.lat_slow_start)
                v_ref *= max(0.0, min(1.0, ratio))

        # v_cmd = v_ref * t + v_lat * n
        # 横向纠偏必须单独限速；否则 e_n 一大，反馈项会直接把合速度打到 v_max，
        # 使弯道减速/制动包络失效。
        v_lat_cmd = -self.kp_lat * e_n - self.kd_lat * v_n
        if abs(v_lat_cmd) > self.max_lat_speed:
            v_lat_cmd = math.copysign(self.max_lat_speed, v_lat_cmd)
        vx_world_des = v_ref * tx + v_lat_cmd * nx
        vy_world_des = v_ref * ty + v_lat_cmd * ny

        # === 合速度限幅 ===
        v_des_mag = math.hypot(vx_world_des, vy_world_des)
        speed_cap = min(self.v_max, max(self.min_tracking_speed, v_ref, abs(v_lat_cmd)))
        if v_des_mag > speed_cap:
            scale = speed_cap / v_des_mag
            vx_world_des *= scale
            vy_world_des *= scale

        # === world → body frame ===
        cy = math.cos(self.chassis_yaw)
        sy = math.sin(self.chassis_yaw)
        vx_body_des = cy * vx_world_des + sy * vy_world_des
        vy_body_des = -sy * vx_world_des + cy * vy_world_des

        # === 合加速度限幅 (body frame) ===
        dvx = vx_body_des - self.last_vx
        dvy = vy_body_des - self.last_vy
        dv_mag = math.hypot(dvx, dvy)
        max_dv = self.acc_lim * self.dt
        if dv_mag > max_dv:
            scale = max_dv / dv_mag
            dvx *= scale
            dvy *= scale
        vx_cmd = self.last_vx + dvx
        vy_cmd = self.last_vy + dvy

        # === 角速度 ===
        # 轨迹方向与当前航向的偏差
        traj_yaw = math.atan2(ty, tx)
        yaw_err = traj_yaw - self.chassis_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
        if end_dist > self.goal_tol * 3:
            wz_des = 0.0
        else:
            wz_des = max(-self.w_max, min(self.w_max, self.k_yaw * yaw_err * 0.3))
        wz_cmd = self._apply_acc_lim(self.last_wz, wz_des, self.acc_lim_yaw)

        self._publish_cmd(vx_cmd, vy_cmd, wz_cmd)

        if self.debug_frenet:
            self.get_logger().info(
                f'idx={nearest_idx}/{len(self.path)-1} '
                f'pos=({self.rx:.2f},{self.ry:.2f}) ref=({ref_x:.2f},{ref_y:.2f}) '
                f'end={end_dist:.2f} yaw={self.chassis_yaw:.2f} '
                f't=({tx:.2f},{ty:.2f}) n=({nx:.2f},{ny:.2f}) '
                f'e_n={e_n:.3f} v_n={v_n:.2f} k={curvature:.3f} '
                f'v_ref={v_ref:.2f} v_lat={v_lat_cmd:.2f} cap={speed_cap:.2f} '
                f'vdes_w=({vx_world_des:.2f},{vy_world_des:.2f}) '
                f'cmd_b=({vx_cmd:.2f},{vy_cmd:.2f})',
                throttle_duration_sec=0.2)
        else:
            self.get_logger().info(
                f'idx={nearest_idx}/{len(self.path)-1} '
                f'end={end_dist:.2f}m '
                f'e_n={e_n:.3f}m k={curvature:.3f} '
                f'v_ref={v_ref:.2f} v={math.hypot(vx_cmd,vy_cmd):.2f} '
                f'vx={vx_cmd:.2f} vy={vy_cmd:.2f}',
                throttle_duration_sec=0.5)

    def _apply_acc_lim(self, last, des, lim):
        delta = des - last
        max_delta = lim * self.dt
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        return last + delta

    def _publish_cmd(self, vx, vy, wz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)
        self.last_vx = vx
        self.last_vy = vy
        self.last_wz = wz


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
