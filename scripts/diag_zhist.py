#!/usr/bin/env python3
import rclpy, struct, math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class ZHist(Node):
    def __init__(self):
        super().__init__('zhist')
        self.sub = self.create_subscription(PointCloud2, '/cloud_registered', self.cb, 10)
        self.timer = self.create_timer(8.0, self.timeout)
        self.done = False
    def cb(self, msg):
        if self.done: return
        self.done = True
        offsets = {}
        for f in msg.fields:
            offsets[f.name] = f.offset
        ps = msg.point_step
        data = msg.data
        npts = msg.width * msg.height
        zs = []
        for i in range(min(npts, 50000)):
            base = i * ps
            try:
                x = struct.unpack_from('f', data, base+offsets['x'])[0]
                y = struct.unpack_from('f', data, base+offsets['y'])[0]
                z = struct.unpack_from('f', data, base+offsets['z'])[0]
            except:
                break
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                zs.append(z)
        zs.sort()
        n = len(zs)
        print(f"Points: {n}")
        if n:
            print(f"Z min={zs[0]:.3f} p5={zs[n//20]:.3f} p10={zs[n//10]:.3f} p25={zs[n//4]:.3f} median={zs[n//2]:.3f} p75={zs[3*n//4]:.3f} max={zs[-1]:.3f}")
            # Histogram
            for lo in [-0.6,-0.5,-0.45,-0.4,-0.35,-0.3,-0.25,-0.2,-0.15,-0.1,-0.05,0,0.05,0.1,0.2]:
                hi = lo + 0.05
                c = sum(1 for z in zs if lo <= z < hi)
                print(f"  [{lo:.2f},{hi:.2f}): {c} ({100*c/n:.1f}%)")
        rclpy.shutdown()
    def timeout(self):
        print("No cloud!")
        rclpy.shutdown()

rclpy.init()
rclpy.spin(ZHist())
