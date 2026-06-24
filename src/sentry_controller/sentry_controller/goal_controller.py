"""
单目标控制器 —— 阶段 3.2

订阅：/goal_pose  (PoseStamped, RViz 2D Nav Goal)
     /Odometry   (Odometry, FAST-LIO2, position in lidar_odom)
     /odom       (Odometry, Gazebo ground truth, chassis yaw)
发布：/cmd_vel_chassis (Twist → Gazebo)

注意：仿真中 base_link (LiDAR mount) 持续旋转，FAST-LIO2 的 yaw 是旋转顶的 yaw。
底盘 (base_link_fake) 的 yaw 从 Gazebo /odom 获取。
位置仍用 FAST-LIO2 /Odometry（与 goal 同在 lidar_odom 系）。

行为：直接驱动小车朝目标点运动（全向底盘，空旷区，无避障）
约束：速度饱和 + 加速度/角速度限幅，保证动力学可行
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalController(Node):
    def __init__(self):
        super().__init__('goal_controller')

        self.declare_parameter('v_max',       0.5)    # m/s
        self.declare_parameter('w_max',       1.5)    # rad/s
        self.declare_parameter('k_p',         1.5)    # 比例增益 v = min(v_max, k_p * dist)
        self.declare_parameter('k_yaw',       2.0)    # 角速度比例增益
        self.declare_parameter('acc_lim',     0.5)    # 线加速度限幅 m/s²
        self.declare_parameter('acc_lim_yaw', 2.0)    # 角加速度限幅 rad/s²
        self.declare_parameter('goal_tol',    0.15)   # 到点容差 m
        self.declare_parameter('rate_hz',     20.0)   # 控制频率
        self.declare_parameter('odom_topic',  '/Odometry')   # FAST-LIO2 position (lidar_odom frame)
        self.declare_parameter('chassis_odom_topic', '/odom')  # Gazebo ground truth (chassis yaw)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_chassis')

        self.v_max       = self.get_parameter('v_max').value
        self.w_max       = self.get_parameter('w_max').value
        self.k_p         = self.get_parameter('k_p').value
        self.k_yaw       = self.get_parameter('k_yaw').value
        self.acc_lim     = self.get_parameter('acc_lim').value
        self.acc_lim_yaw = self.get_parameter('acc_lim_yaw').value
        self.goal_tol    = self.get_parameter('goal_tol').value
        odom_topic       = self.get_parameter('odom_topic').value
        chassis_odom_topic = self.get_parameter('chassis_odom_topic').value
        cmd_vel_topic    = self.get_parameter('cmd_vel_topic').value
        rate             = self.get_parameter('rate_hz').value

        self.dt = 1.0 / rate

        self.rx = 0.0
        self.ry = 0.0
        self.ryaw = 0.0
        self.odom_rcv = False
        self.chassis_yaw = 0.0
        self.chassis_rcv = False

        self.goal_x = None
        self.goal_y = None

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

        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, qos)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_sensor)
        self.sub_chassis_odom = self.create_subscription(
            Odometry, chassis_odom_topic, self._on_chassis_odom, qos)
        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'GoalController: v_max={self.v_max} w_max={self.w_max} '
            f'k_p={self.k_p} acc_lim={self.acc_lim} goal_tol={self.goal_tol} '
            f'rate={rate}Hz pos_odom={odom_topic} chassis_odom={chassis_odom_topic} cmd={cmd_vel_topic}')

    def _on_goal(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.at_goal = False
        self.get_logger().info(
            f'New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def _on_odom(self, msg: Odometry):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        self.odom_rcv = True

    def _on_chassis_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.chassis_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.chassis_rcv = True

    def _control_loop(self):
        if not self.odom_rcv or not self.chassis_rcv or self.goal_x is None:
            return

        dx = self.goal_x - self.rx
        dy = self.goal_y - self.ry
        dist = math.hypot(dx, dy)

        if dist < self.goal_tol:
            if not self.at_goal:
                self.get_logger().info(
                    f'Reached goal: dist={dist:.3f}m < tol={self.goal_tol}m')
                self.at_goal = True
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        self.at_goal = False

        # 目标方向在 chassis body frame 下的角度（用底盘 yaw，非 LiDAR yaw）
        alpha = math.atan2(dy, dx) - self.chassis_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # 速度饱和：v = min(v_max, k_p * dist)
        v_des = min(self.v_max, self.k_p * dist)

        # 全向底盘：直接将速度向量投影到 body frame
        vx_des = v_des * math.cos(alpha)
        vy_des = v_des * math.sin(alpha)

        # 角速度：全向底盘不需要转向到目标方向，直接平移即可
        # 仅在接近目标时缓慢对齐航向，避免轨道效应
        if dist > self.goal_tol * 3:
            wz_des = 0.0
        else:
            wz_des = max(-self.w_max, min(self.w_max, -self.k_yaw * alpha * 0.3))

        # 加速度限幅
        vx = self._apply_acc_lim(self.last_vx, vx_des, self.acc_lim)
        vy = self._apply_acc_lim(self.last_vy, vy_des, self.acc_lim)
        wz = self._apply_acc_lim(self.last_wz, wz_des, self.acc_lim_yaw)

        self._publish_cmd(vx, vy, wz)

        self.get_logger().info(
            f'dist={dist:.2f}m alpha={math.degrees(alpha):.1f}° '
            f'vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}',
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
    node = GoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
