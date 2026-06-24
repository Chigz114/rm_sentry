#!/usr/bin/env python3
"""Costmap inflation node — Stage 3.0

Subscribes to /perception/costmap_2d (OccupancyGrid)
Performs binary circular dilation by inflation_radius_m
Publishes /planner/costmap_inflated (OccupancyGrid)

The inflation ensures the robot's physical footprint won't overlap obstacles.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class CostmapInflator(Node):
    def __init__(self):
        super().__init__('costmap_inflator')

        self.declare_parameter('inflation_radius_m', 0.35)
        self.declare_parameter('input_topic', '/perception/costmap_2d')
        self.declare_parameter('output_topic', '/planner/costmap_inflated')

        self.inflation_radius_m = self.get_parameter('inflation_radius_m').value
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            OccupancyGrid, in_topic, self._on_costmap, rclpy.qos.qos_profile_sensor_data)
        self.pub = self.create_publisher(OccupancyGrid, out_topic, 10)

        self._kernel = None
        self._kernel_size = 0

        self.get_logger().info(
            f'CostmapInflator: in={in_topic} out={out_topic} '
            f'inflation={self.inflation_radius_m}m')

    def _build_kernel(self, resolution: float):
        r_cells = max(1, int(round(self.inflation_radius_m / resolution)))
        size = 2 * r_cells + 1
        kernel = np.zeros((size, size), dtype=bool)
        cy = cx = r_cells
        for dy in range(size):
            for dx in range(size):
                if math.hypot(dx - cx, dy - cy) <= r_cells:
                    kernel[dy, dx] = True
        self._kernel = kernel
        self._kernel_size = size
        self._kernel_radius = r_cells

    def _on_costmap(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        occupied = (data == 100)

        if self._kernel is None or abs(res * self._kernel_radius - self.inflation_radius_m) > res * 0.5:
            self._build_kernel(res)

        kr = self._kernel_radius
        inflated = np.zeros_like(occupied)
        ys, xs = np.where(occupied)
        for y, x in zip(ys, xs):
            y0 = max(0, y - kr)
            y1 = min(h, y + kr + 1)
            x0 = max(0, x - kr)
            x1 = min(w, x + kr + 1)
            ky0 = max(0, kr - y)
            ky1 = ky0 + (y1 - y0)
            kx0 = max(0, kr - x)
            kx1 = kx0 + (x1 - x0)
            inflated[y0:y1, x0:x1] |= self._kernel[ky0:ky1, kx0:kx1]

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = (np.where(inflated, 100, 0).astype(np.int8).flatten().tolist())

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapInflator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
