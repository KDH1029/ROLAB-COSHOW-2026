#!/usr/bin/env python3
"""Hula-hoop vision on the internal PC (Jetson, ROS 2 Foxy, Python 3.8, TensorRT) — red / yellow hoop direction.

  in   /mission/hoop_active  std_msgs/Bool   the runner sets True while it stands at the look point (re-sent 1 Hz)
       /camera/color/image_raw, /camera/color/camera_info  (same PC's D435, BEST_EFFORT) — subscribed only while active
  out  /mission/hoop_state   std_msgs/String JSON  {"red": {"bearing_deg": +left, "conf": .., "u": px},
                                                    "yellow": {...}}   (a colour not seen is left out)
Model: either the hoop-only YOLOv8 (0 rh, 1 yh, insu_label 11-19, hoop_384x640_fp16.engine — the default) or the
team's 6-class tl6_1119 (2 rh, 3 yh): engine:=... red_cls:=2 yellow_cls:=3.
Bearing = atan((cx - u) / fx) of the best box of each colour (camera looks forward like the robot).
Same pre/post-processing as the traffic-light node (tl_yolo_common: letterbox 384x640, decode, NMS).
★ Python 3.8.
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tl_yolo_common as C  # noqa: E402
from trt_engine import TrtEngine  # noqa: E402


def to_bgr(msg):
    enc = msg.encoding.lower()
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return a[..., ::-1].copy() if enc == 'rgb8' else a


class HoopNode(Node):
    def __init__(self):
        super().__init__('hoop_vision')
        self.declare_parameter('engine', os.path.join(HERE, 'hoop_384x640_fp16.engine'))
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('hz', 5.0)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter('red_cls', 0)        # hoop-only model 0/1, tl6_1119 2/3
        self.declare_parameter('yellow_cls', 1)
        g = lambda n: self.get_parameter(n).value
        self.eng = TrtEngine(g('engine'))
        self.H, self.W = self.eng.input_shape[2], self.eng.input_shape[3]
        self.eng.infer(np.zeros(self.eng.input_shape, np.float32))      # warm up
        self.conf, self.dt = float(g('conf')), 1.0 / float(g('hz'))
        self.colours = {int(g('red_cls')): 'red', int(g('yellow_cls')): 'yellow'}
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.subs = []
        self.active = False
        self.K = None
        self.last = 0.0
        self.n = 0
        self.create_subscription(Bool, '/mission/hoop_active', self.cb_active, 10)
        self.pub = self.create_publisher(String, '/mission/hoop_state', 10)
        self.get_logger().info('hoop vision ready (%s, %dx%d) — waiting for /mission/hoop_active'
                               % (os.path.basename(g('engine')), self.W, self.H))

    def cb_active(self, m):
        on = bool(m.data)
        if on == self.active:
            return
        self.active = on
        if on:
            self.subs = [self.create_subscription(Image, self.get_parameter('image_topic').value, self.cb_image, self.qos),
                         self.create_subscription(CameraInfo, self.get_parameter('info_topic').value, self.cb_info, self.qos)]
        else:
            for s in self.subs:
                self.destroy_subscription(s)
            self.subs = []
        self.get_logger().info('hoop_active -> %s' % on)

    def cb_info(self, m):
        self.K = np.array(m.k, np.float64).reshape(3, 3)

    def cb_image(self, m):
        t = time.time()
        if not self.active or t - self.last < self.dt:
            return
        self.last = t
        bgr = to_bgr(m)
        blob, r, left, top = C.letterbox(bgr, (self.H, self.W))
        out = self.eng.infer(blob)
        dets = C.decode(out, r, left, top, bgr.shape[:2], conf_thr=self.conf)
        fx = self.K[0, 0] if self.K is not None else 0.5 * bgr.shape[1] / math.tan(math.radians(69.4 / 2))
        cx = self.K[0, 2] if self.K is not None else bgr.shape[1] / 2.0
        res = {}
        for c, s, x1, y1, x2, y2 in dets:                 # best (first) box of each colour
            name = self.colours.get(c)
            if name is None or name in res:
                continue
            u = 0.5 * (x1 + x2)
            res[name] = {'bearing_deg': round(math.degrees(math.atan2(cx - u, fx)), 2), 'conf': round(s, 3),
                         'u': round(u, 1), 'box': [round(x1), round(y1), round(x2), round(y2)]}
        self.pub.publish(String(data=json.dumps(res)))
        self.n += 1
        if self.n % 5 == 1:
            self.get_logger().info('hoops: %s (%.0f ms)' % (', '.join('%s %+.1f deg %.2f' % (k, v['bearing_deg'], v['conf'])
                                                                       for k, v in res.items()) or 'none', 1000 * (time.time() - t)))


def main():
    rclpy.init()
    n = HoopNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
