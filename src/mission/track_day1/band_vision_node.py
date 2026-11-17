#!/usr/bin/env python3
"""Floor band edges from the D435 on the internal PC (Jetson, ROS 2 Foxy, Python 3.8).

  in   /camera/color/image_raw, /camera/depth/image_rect_raw (+ camera_info), BEST_EFFORT
  out  /track/edges  std_msgs/String JSON {ok, age, pts:[x,y,normal,strength]*N (robot frame), h, pitch, roll, ms, why}
       age = seconds from frame arrival on this PC to publish (+ sensor pipeline), so the NUC needs no clock sync
  in   /track/snap   std_msgs/Bool  -> writes camera + bird's-eye check images to ~/coshow_track/snap/

Only std_msgs leave this PC (Foxy <-> Jazzy custom messages break). No cv_bridge.
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import band_vision_core as BV  # noqa: E402

SENSOR_LAT = 0.03      # s, exposure -> ROS message on this PC (D435 at 30 fps)


def to_np(msg):
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8'):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a[..., ::-1].copy() if enc == 'rgb8' else a
    if enc in ('16uc1', 'mono16'):
        a = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return a.astype(np.float32) * 0.001
    raise ValueError('encoding %s' % msg.encoding)


class BandVisionNode(Node):
    def __init__(self):
        super().__init__('band_vision')
        p = self.declare_parameter
        self.hz = float(p('hz', 10.0).value)
        cfg = dict(cam_x=float(p('cam_x', 0.466).value), cam_y=float(p('cam_y', 0.0096).value),
                   cam_yaw=float(p('cam_yaw', 0.0).value))
        self.vis = BV.BandVision(cfg)
        self.snap_dir = os.path.expanduser(str(p('snap_dir', '~/coshow_track/snap').value))
        q = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, str(p('color_topic', '/camera/color/image_raw').value), self.cb_color, q)
        self.create_subscription(Image, str(p('depth_topic', '/camera/depth/image_rect_raw').value), self.cb_depth, q)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cb_ci, q)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.cb_di, q)
        self.create_subscription(Bool, '/track/snap', self.cb_snap, 2)
        self.pub = self.create_publisher(String, '/track/edges', QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.color = self.depth = None
        self.Kc = self.Kd = None
        self.want_snap = False
        self.ms = []
        self.n_pub = self.n_ok = 0
        self.last_why = ''
        self.create_timer(1.0 / self.hz, self.tick)
        self.create_timer(5.0, self.report)
        self.get_logger().info('band vision %.0f Hz -> /track/edges' % self.hz)

    def cb_color(self, m):
        self.color = (time.time(), m)

    def cb_depth(self, m):
        self.depth = (time.time(), m)

    def cb_ci(self, m):
        self.Kc = np.array(m.k, np.float64).reshape(3, 3)

    def cb_di(self, m):
        self.Kd = np.array(m.k, np.float64).reshape(3, 3)

    def cb_snap(self, m):
        self.want_snap = bool(m.data)

    def tick(self):
        if self.color is None or self.depth is None or self.Kc is None or self.Kd is None:
            return
        tc, mc = self.color
        td, md = self.depth
        if time.time() - tc > 0.5:
            return
        if abs(tc - td) > 0.1:
            self.last_why = 'colour/depth %.0f ms apart' % (abs(tc - td) * 1e3)
            return
        t0 = time.time()
        try:
            bgr = to_np(mc)
            dep = to_np(md)
        except ValueError as e:
            self.last_why = str(e)
            return
        out = self.vis.process(bgr, dep, self.Kc, self.Kd)
        ms = (time.time() - t0) * 1e3
        self.ms.append(ms)
        d = dict(ok=bool(out['ok']), why=out.get('why', ''), ms=round(ms, 1),
                 age=round(time.time() - min(tc, td) + SENSOR_LAT, 4))
        if 'h' in out:
            d.update(h=round(float(out['h']), 3), pitch=round(float(out['pitch']), 2), roll=round(float(out['roll']), 2))
        if out['ok']:
            d['pts'] = [round(float(v), 3) for v in out['pts'].reshape(-1)]
            self.n_ok += 1
        self.last_why = d['why']
        self.pub.publish(String(data=json.dumps(d, separators=(',', ':'))))
        self.n_pub += 1
        if self.want_snap:
            self.want_snap = False
            self.snap(bgr, out)

    def snap(self, bgr, out):
        import cv2
        os.makedirs(self.snap_dir, exist_ok=True)
        tag = time.strftime('%H%M%S')
        cv2.imwrite(os.path.join(self.snap_dir, 'cam_%s.jpg' % tag), bgr)
        if out.get('bev') is not None:
            c = self.vis.cfg
            bev = out['bev'].copy()
            bev[~out['valid']] //= 3
            big = cv2.resize(bev, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
            for xr, yr, th, mg in out['pts']:
                xf, yf = xr - c['cam_x'], yr - c['cam_y']
                row, col = (c['bev_x1'] - xf) / c['bev_res'], (c['bev_y'] - yf) / c['bev_res']
                cv2.circle(big, (int(col * 3), int(row * 3)), 2, (0, 0, 255), -1)
            cv2.imwrite(os.path.join(self.snap_dir, 'bev_%s.jpg' % tag), big)
        self.get_logger().info('snap -> %s/*_%s.jpg' % (self.snap_dir, tag))

    def report(self):
        ms = float(np.median(self.ms)) if self.ms else 0.0
        self.get_logger().info('published %d (ok %d) in 5 s · %.1f ms/frame · %s'
                               % (self.n_pub, self.n_ok, ms, self.last_why or 'ok'))
        self.ms = []
        self.n_pub = self.n_ok = 0


def main():
    rclpy.init()
    n = BandVisionNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
