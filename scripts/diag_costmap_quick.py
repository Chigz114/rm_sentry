#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class CM(Node):
    def __init__(self):
        super().__init__('cm')
        self.sub = self.create_subscription(OccupancyGrid, '/perception/costmap_2d', self.cb, 10)
        self.timer = self.create_timer(8.0, self.timeout)
        self.done = False
    def cb(self, msg):
        if self.done: return
        self.done = True
        w, h = msg.info.width, msg.info.height
        vals = list(msg.data)
        occ = sum(1 for v in vals if v == 100)
        free = sum(1 for v in vals if v == 0)
        other = w*h - occ - free
        print(f'Costmap: {w}x{h}={w*h} cells')
        print(f'Occupied(100): {occ}, Free(0): {free}, Other: {other}')
        # Show value distribution
        from collections import Counter
        dist = Counter(vals)
        for v in sorted(dist.keys()):
            print(f'  val={v}: {dist[v]} cells')
        rclpy.shutdown()
    def timeout(self):
        if not self.done:
            print('No costmap received!')
        rclpy.shutdown()

rclpy.init()
node = CM()
rclpy.spin(node)
