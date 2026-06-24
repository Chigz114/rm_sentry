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
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution
        occ = sum(1 for v in msg.data if v == 100)
        print(f'Costmap: {w}x{h}, occupied={occ}')
        # Highland A: lidar_odom x[-3.5,-1.5] y[-1,4]
        hla = 0
        for j in range(h):
            for i in range(w):
                if msg.data[j*w+i] == 100:
                    wx = ox + (i+0.5)*res
                    wy = oy + (j+0.5)*res
                    if -3.5 <= wx <= -1.5 and -1 <= wy <= 4:
                        hla += 1
        # Highland B: lidar_odom x[1.5,3.5] y[-4,1]
        hlb = 0
        for j in range(h):
            for i in range(w):
                if msg.data[j*w+i] == 100:
                    wx = ox + (i+0.5)*res
                    wy = oy + (j+0.5)*res
                    if 1.5 <= wx <= 3.5 and -4 <= wy <= 1:
                        hlb += 1
        print(f'Highland A occupied: {hla} cells')
        print(f'Highland B occupied: {hlb} cells')
        rclpy.shutdown()
    def timeout(self):
        if not self.done:
            print('No costmap received!')
        rclpy.shutdown()

rclpy.init()
node = CM()
rclpy.spin(node)
