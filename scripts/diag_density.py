#!/usr/bin/env python3
import rclpy, struct, math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class DensityAnalyzer(Node):
    def __init__(self):
        super().__init__('density_a')
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
        offsets = {}
        for f in msg.fields:
            offsets[f.name] = f.offset
        ps = msg.point_step
        data = msg.data
        npts = msg.width * msg.height
        print(f'Total points in cloud: {npts}')
        # Grid params
        res = 0.1; gw, gh = 130, 90; ox, oy = -6.5, -4.5
        # Count points per cell for highland areas
        # Highland A: lidar_odom x[-3.5,-1.5] y[-1,4]
        # Highland B: lidar_odom x[1.5,3.5] y[-4,1]
        cell_counts_a = {}
        cell_counts_b = {}
        all_pts_a = []
        all_pts_b = []
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
            # Highland A
            if -3.5 <= x <= -1.5 and -1 <= y <= 4:
                gi = int((x - ox) / res)
                gj = int((y - oy) / res)
                key = (gi, gj)
                cell_counts_a[key] = cell_counts_a.get(key, 0) + 1
                all_pts_a.append((x, y, z))
            # Highland B
            if 1.5 <= x <= 3.5 and -4 <= y <= 1:
                gi = int((x - ox) / res)
                gj = int((y - oy) / res)
                key = (gi, gj)
                cell_counts_b[key] = cell_counts_b.get(key, 0) + 1
                all_pts_b.append((x, y, z))
        print(f'\nHighland A: {len(all_pts_a)} points in {len(cell_counts_a)} cells')
        if cell_counts_a:
            vals = sorted(cell_counts_a.values())
            print(f'  Points/cell: min={vals[0]}, median={vals[len(vals)//2]}, max={vals[-1]}')
            print(f'  Cells with 0 pts: {2000 - len(cell_counts_a)} (out of ~2000 total)')
            # Z distribution
            zs = sorted([p[2] for p in all_pts_a])
            print(f'  Z range: [{zs[0]:.3f}, {zs[-1]:.3f}], median={zs[len(zs)//2]:.3f}')
        print(f'\nHighland B: {len(all_pts_b)} points in {len(cell_counts_b)} cells')
        if cell_counts_b:
            vals = sorted(cell_counts_b.values())
            print(f'  Points/cell: min={vals[0]}, median={vals[len(vals)//2]}, max={vals[-1]}')
            print(f'  Cells with 0 pts: {2000 - len(cell_counts_b)} (out of ~2000 total)')
            zs = sorted([p[2] for p in all_pts_b])
            print(f'  Z range: [{zs[0]:.3f}, {zs[-1]:.3f}], median={zs[len(zs)//2]:.3f}')
        # Show spatial coverage as a coarse grid for Highland A
        print(f'\nHighland A coverage (X: -3.5 to -1.5, Y: -1 to 4):')
        for ybin in range(-1, 5):
            row = ''
            for xbin in range(-3, 0):
                cnt = sum(1 for x,y,z in all_pts_a if xbin<=x<xbin+1 and ybin<=y<ybin+1)
                row += f'{cnt:5d}'
            print(f'  Y[{ybin}:{ybin+1}]: {row}')
        print(f'\nHighland B coverage (X: 1.5 to 3.5, Y: -4 to 1):')
        for ybin in range(-4, 2):
            row = ''
            for xbin in range(1, 4):
                cnt = sum(1 for x,y,z in all_pts_b if xbin<=x<xbin+1 and ybin<=y<ybin+1)
                row += f'{cnt:5d}'
            print(f'  Y[{ybin}:{ybin+1}]: {row}')
        rclpy.shutdown()
    def timeout(self):
        if not self.cloud_received:
            print(f'Timeout! odom={self.odom_received} cloud={self.cloud_received}')
        rclpy.shutdown()

rclpy.init()
node = DensityAnalyzer()
rclpy.spin(node)
