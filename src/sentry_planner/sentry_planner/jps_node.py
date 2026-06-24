#!/usr/bin/env python3
"""JPS (Jump Point Search) path planner — Stage 3.1

Subscribes:
  /planner/costmap_inflated (OccupancyGrid) — inflated costmap
  /goal_pose (PoseStamped) — from RViz "2D Nav Goal"
  /Odometry (Odometry) — robot current position

Publishes:
  /planner/path (nav_msgs/Path) — waypoint sequence for controller

JPS is an optimized variant of A* on uniform grids that skips
expanding symmetric nodes in open areas, producing the same optimal
path as A* but significantly faster.
"""
import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException
import math as _math


# 8-direction movement: (dx, dy, cost)
DIRS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, 1.41421356), (1, -1, 1.41421356),
    (-1, 1, 1.41421356), (-1, -1, 1.41421356),
]

# Direction lookup by (dx, dy) → index
DIR_MAP = {(d[0], d[1]): i for i, d in enumerate(DIRS)}


class JPSNode(Node):
    def __init__(self):
        super().__init__('jps_node')

        self.declare_parameter('costmap_topic', '/planner/costmap_inflated')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('odom_frame', 'lidar_odom')
        self.declare_parameter('use_raw_odom', False)  # bypass TF, use odom position directly (for sim when LIO diverges)
        self.declare_parameter('path_topic', '/planner/path')
        self.declare_parameter('max_iter', 200000)
        self.declare_parameter('goal_tolerance_m', 0.15)

        costmap_topic = self.get_parameter('costmap_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.use_raw_odom = self.get_parameter('odom_topic').value
        self.use_raw_odom = self.get_parameter('use_raw_odom').value
        path_topic = self.get_parameter('path_topic').value
        self.max_iter = self.get_parameter('max_iter').value
        self.goal_tol = self.get_parameter('goal_tolerance_m').value

        self.grid: np.ndarray = None
        self.grid_w = 0
        self.grid_h = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame_id = 'map'
        self.odom_frame = self.get_parameter('odom_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal_x = None
        self.goal_y = None

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.sub_costmap = self.create_subscription(
            OccupancyGrid, costmap_topic, self._on_costmap, qos_sensor)
        self.sub_goal = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal, 10)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_sensor)

        self.pub_path = self.create_publisher(Path, path_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/planner/jps_viz', 10)

        self.get_logger().info(
            f'JPSNode: costmap={costmap_topic} goal={goal_topic} '
            f'odom={odom_topic} path={path_topic}')

    # ── callbacks ──────────────────────────────────────────────

    def _on_costmap(self, msg: OccupancyGrid):
        self.grid_w = msg.info.width
        self.grid_h = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.frame_id = msg.header.frame_id
        self.grid = np.array(msg.data, dtype=np.int8).reshape((self.grid_h, self.grid_w))
        # occupied = 100, free = 0
        self._occupied = (self.grid == 100)

    def _on_odom(self, msg: Odometry):
        if self.use_raw_odom:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.odom_frame, rclpy.time.Time())
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = 2.0 * _math.atan2(q.z, q.w)
            cy = _math.cos(yaw)
            sy = _math.sin(yaw)
            self.robot_x = cy * px - sy * py + t.x
            self.robot_y = sy * px + cy * py + t.y
        except TransformException:
            pass

    def _on_goal(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(
            f'Goal received: ({self.goal_x:.2f}, {self.goal_y:.2f}) frame={msg.header.frame_id}')
        self._plan_and_publish()

    # ── coordinate conversion ──────────────────────────────────

    def _world_to_grid(self, x, y):
        gx = int(round((x - self.origin_x) / self.resolution))
        gy = int(round((y - self.origin_y) / self.resolution))
        return gx, gy

    def _grid_to_world(self, gx, gy):
        wx = gx * self.resolution + self.origin_x
        wy = gy * self.resolution + self.origin_y
        return wx, wy

    # ── grid helpers ───────────────────────────────────────────

    def _is_free(self, x, y):
        if x < 0 or x >= self.grid_w or y < 0 or y >= self.grid_h:
            return False
        return not self._occupied[y, x]

    def _is_blocked(self, x, y):
        if x < 0 or x >= self.grid_w or y < 0 or y >= self.grid_h:
            return True
        return self._occupied[y, x]

    # ── JPS core ───────────────────────────────────────────────

    def _jump(self, x, y, dx, dy, gx, gy):
        """Jump from (x,y) in direction (dx,dy). Returns (jx, jy) or None."""
        nx, ny = x + dx, y + dy
        if self._is_blocked(nx, ny):
            return None

        if (nx, ny) == (gx, gy):
            return (nx, ny)

        # Check for forced neighbors
        if dx != 0 and dy != 0:
            # Diagonal move
            # Forced neighbor: horizontal/vertical blocked adjacent
            if (self._is_blocked(nx - dx, ny) and not self._is_blocked(nx - dx, ny + dy)):
                return (nx, ny)
            if (self._is_blocked(nx, ny - dy) and not self._is_blocked(nx + dx, ny - dy)):
                return (nx, ny)

            # Recurse diagonal + horizontal + vertical
            h = self._jump(nx, ny, dx, 0, gx, gy)
            if h is not None:
                return (nx, ny)
            v = self._jump(nx, ny, 0, dy, gx, gy)
            if v is not None:
                return (nx, ny)

        else:
            # Cardinal move
            if dx != 0:
                # Horizontal
                if (self._is_blocked(nx, ny - 1) and not self._is_blocked(nx + dx, ny - 1)):
                    return (nx, ny)
                if (self._is_blocked(nx, ny + 1) and not self._is_blocked(nx + dx, ny + 1)):
                    return (nx, ny)
            else:
                # Vertical (dy != 0)
                if (self._is_blocked(nx - 1, ny) and not self._is_blocked(nx - 1, ny + dy)):
                    return (nx, ny)
                if (self._is_blocked(nx + 1, ny) and not self._is_blocked(nx + 1, ny + dy)):
                    return (nx, ny)

        return self._jump(nx, ny, dx, dy, gx, gy)

    def _heuristic(self, x, y, gx, gy):
        dx = abs(x - gx)
        dy = abs(y - gy)
        return (dx + dy) + (1.41421356 - 2) * min(dx, dy)

    def _pruned_neighbors(self, x, y, px, py):
        """Get pruned neighbors for JPS expansion from (x,y) with parent (px,py)."""
        if px is None:
            # No parent: all 8 directions
            return [(d[0], d[1]) for d in DIRS]

        dx = x - px
        dy = y - py
        # Normalize to unit steps
        if dx != 0:
            dx = dx // abs(dx)
        if dy != 0:
            dy = dy // abs(dy)

        neighbors = []

        if dx != 0 and dy != 0:
            # Diagonal parent
            # Natural neighbor: continue diagonal
            neighbors.append((dx, dy))
            # Forced neighbors (if adjacent blocked)
            if self._is_blocked(x - dx, y):
                neighbors.append((-dx, dy))
            if self._is_blocked(x, y - dy):
                neighbors.append((dx, -dy))
            # Also add straight expansions
            neighbors.append((dx, 0))
            neighbors.append((0, dy))
        else:
            # Cardinal parent
            if dx != 0:
                # Horizontal parent
                neighbors.append((dx, 0))
                if self._is_blocked(x, y - 1):
                    neighbors.append((dx, -1))
                if self._is_blocked(x, y + 1):
                    neighbors.append((dx, 1))
            else:
                # Vertical parent
                neighbors.append((0, dy))
                if self._is_blocked(x - 1, y):
                    neighbors.append((-1, dy))
                if self._is_blocked(x + 1, y):
                    neighbors.append((1, dy))

        return neighbors

    def _jps_search(self, sx, sy, gx, gy):
        """Run JPS from (sx,sy) to (gx,gy). Returns list of (x,y) grid points or None."""
        if not self._is_free(sx, sy):
            self.get_logger().warn(f'Start ({sx},{sy}) is blocked')
            return None
        if not self._is_free(gx, gy):
            self.get_logger().warn(f'Goal ({gx},{gy}) is blocked')
            return None

        open_set = []
        counter = 0
        g_score = {(sx, sy): 0.0}
        came_from = {}

        heapq.heappush(open_set, (0.0, counter, (sx, sy), None))

        closed = set()
        iterations = 0

        while open_set:
            iterations += 1
            if iterations > self.max_iter:
                self.get_logger().warn(f'JPS exceeded max_iter={self.max_iter}')
                return None

            f, _, current, parent = heapq.heappop(open_set)
            cx, cy = current

            if current in closed:
                continue
            closed.add(current)

            if cx == gx and cy == gy:
                # Reconstruct path
                path = [(cx, cy)]
                node = current
                while node in came_from:
                    node = came_from[node]
                    path.append(node)
                path.reverse()
                return path

            # Get pruned neighbors and jump
            neighbors = self._pruned_neighbors(cx, cy, parent[0] if parent else None,
                                               parent[1] if parent else None)
            for dx, dy in neighbors:
                j = self._jump(cx, cy, dx, dy, gx, gy)
                if j is None or j in closed:
                    continue

                jx, jy = j
                step_cost = math.hypot(jx - cx, jy - cy)
                tentative_g = g_score[current] + step_cost

                if j not in g_score or tentative_g < g_score[j]:
                    g_score[j] = tentative_g
                    came_from[j] = current
                    f_score = tentative_g + self._heuristic(jx, jy, gx, gy)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, j, current))

        self.get_logger().warn('JPS: no path found')
        return None

    def _simplify_path(self, grid_path):
        """Line-of-sight simplification: remove intermediate points on straight segments."""
        if len(grid_path) <= 2:
            return grid_path

        simplified = [grid_path[0]]
        for i in range(1, len(grid_path) - 1):
            prev = simplified[-1]
            nxt = grid_path[i + 1]
            if not self._line_of_sight(prev[0], prev[1], nxt[0], nxt[1]):
                simplified.append(grid_path[i])
        simplified.append(grid_path[-1])
        return simplified

    def _line_of_sight(self, x0, y0, x1, y1):
        """Bresenham line check: returns True if no obstacle on the line."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if self._is_blocked(x, y):
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # ── planning ───────────────────────────────────────────────

    def _plan_and_publish(self):
        if self.grid is None:
            self.get_logger().warn('No costmap received yet')
            return
        if self.goal_x is None:
            return

        sx, sy = self._world_to_grid(self.robot_x, self.robot_y)
        gx, gy = self._world_to_grid(self.goal_x, self.goal_y)

        self.get_logger().info(
            f'Planning: start=({sx},{sy}) goal=({gx},{gy}) '
            f'grid={self.grid_w}x{self.grid_h}')

        grid_path = self._jps_search(sx, sy, gx, gy)
        if grid_path is None:
            self.get_logger().warn('JPS failed to find path')
            self._publish_empty_path()
            return

        # Simplify with line-of-sight
        simplified = self._simplify_path(grid_path)

        # Convert to world coordinates
        world_path = [self._grid_to_world(px, py) for px, py in simplified]

        self.get_logger().info(
            f'Path found: {len(grid_path)} grid pts → {len(simplified)} waypoints')

        self._publish_path(world_path)
        self._publish_viz(world_path)

    def _publish_path(self, world_path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for wx, wy in world_path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _publish_empty_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub_path.publish(msg)

    def _publish_viz(self, world_path):
        """Publish markers for RViz visualization."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Path line strip
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.ns = 'jps_path'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        m.points = [Point(x=wx, y=wy, z=0.0) for wx, wy in world_path]
        ma.markers.append(m)

        # Waypoint spheres
        m2 = Marker()
        m2.header.stamp = stamp
        m2.header.frame_id = self.frame_id
        m2.ns = 'jps_waypoints'
        m2.id = 1
        m2.type = Marker.SPHERE_LIST
        m2.action = Marker.ADD
        m2.scale.x = 0.15
        m2.scale.y = 0.15
        m2.scale.z = 0.15
        m2.color.r = 1.0
        m2.color.g = 0.5
        m2.color.b = 0.0
        m2.color.a = 1.0
        m2.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        m2.points = [Point(x=wx, y=wy, z=0.0) for wx, wy in world_path]
        ma.markers.append(m2)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = JPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
