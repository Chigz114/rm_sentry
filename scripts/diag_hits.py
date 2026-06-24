#!/usr/bin/env python3
import rclpy, struct, math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class Analyzer(Node):
    def __init__(self):
        super().__init__('analyzer')
        self.odom_received = False
        self.cloud_received = False
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_cb, 10)
        self.cloud_sub = self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_cb, 10)
        self.timer = self.create_timer(10.0, self.timeout)
    def odom_cb(self, msg):
        self.odom_received = True
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
    def cloud_cb(self, msg):
        if not self.odom_received or self.cloud_received: return
        self.cloud_received = True
        rx, ry = self.rx, self.ry
        print(f'Robot pos: ({rx:.3f}, {ry:.3f})')
        offsets = {}
        for f in msg.fields:
            offsets[f.name] = f.offset
        ps = msg.point_step
        data = msg.data
        npts = msg.width * msg.height
        res = 0.1; gw, gh = 130, 90; ox, oy = -6.5, -4.5
        ground_z = -0.4; h_climb = 0.10
        hit_counts = [0]*(gw*gh)
        frame_min = 1e9
        for i in range(min(npts, 100000)):
            base = i*ps
            try:
                x = struct.unpack_from('f', data, base+offsets['x'])[0]
                y = struct.unpack_from('f', data, base+offsets['y'])[0]
                z = struct.unpack_from('f', data, base+offsets['z'])[0]
            except:
                break
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < frame_min:
                frame_min = z
            gx = x-ox; gy = y-oy
            if gx<0 or gy<0: continue
            gi = int(gx/res); gj = int(gy/res)
            if gi<0 or gi>=gw or gj<0 or gj>=gh: continue
            idx = gj*gw+gi
            if z - ground_z > h_climb:
                hit_counts[idx] += 1
        print(f'Frame global min Z: {frame_min:.3f}')
        # Highland B: lidar_odom x[1.5,3.5] y[-4,1]
        maxh = 0; pass_cnt = 0; total_hits = 0
        for gj in range(5, 56):
            for gi in range(80, 101):
                idx = gj*gw+gi
                if hit_counts[idx] > 0:
                    total_hits += 1
                    wx = ox+(gi+0.5)*res; wy = oy+(gj+0.5)*res
                    dist = math.hypot(wx-rx, wy-ry)
                    n_min = 4 if dist<2 else (2 if dist<4 else 1)
                    if hit_counts[idx] >= n_min:
                        pass_cnt += 1
                    if hit_counts[idx] > maxh:
                        maxh = hit_counts[idx]
        print(f'Highland B: cells_with_hits={total_hits}, max_hits={maxh}, cells_passing_nmin={pass_cnt}')
        # Highland A: lidar_odom x[-3.5,-1.5] y[-1,4]
        maxh = 0; pass_cnt = 0; total_hits = 0
        for gj in range(35, 86):
            for gi in range(40, 61):
                idx = gj*gw+gi
                if hit_counts[idx] > 0:
                    total_hits += 1
                    wx = ox+(gi+0.5)*res; wy = oy+(gj+0.5)*res
                    dist = math.hypot(wx-rx, wy-ry)
                    n_min = 4 if dist<2 else (2 if dist<4 else 1)
                    if hit_counts[idx] >= n_min:
                        pass_cnt += 1
                    if hit_counts[idx] > maxh:
                        maxh = hit_counts[idx]
        print(f'Highland A: cells_with_hits={total_hits}, max_hits={maxh}, cells_passing_nmin={pass_cnt}')
        rclpy.shutdown()
    def timeout(self):
        if not self.cloud_received:
            print(f'Timeout! odom={self.odom_received} cloud={self.cloud_received}')
        rclpy.shutdown()

rclpy.init()
node = Analyzer()
rclpy.spin(node)
