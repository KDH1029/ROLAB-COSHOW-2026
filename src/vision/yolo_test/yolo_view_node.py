#!/usr/bin/env python3
"""YOLO test view on the internal PC (Jetson, ROS 2 Foxy, Python 3.8, TensorRT) — test only, its own folder
(~/yolo_test), nothing shared with the competition nodes (traffic light, hula hoop) except reading an engine file.

  in   /camera/color/image_raw (BEST_EFFORT) — subscribed only while someone watches the view
  out  /yolo_test/view/compressed  sensor_msgs/CompressedImage  JPEG, half size, every box of the model drawn
       /yolo_test/dets             std_msgs/String JSON [[class, conf, x1, y1, x2, y2], ...] (full-size pixels)
Runs the engine only while /yolo_test/view/compressed has a subscriber (scripts/yolo_view.sh on the NUC), at `hz`.
Half-size JPEG at 5 Hz is ~100-150 KB/s: the raw 848x480 stream would fill the robot's 100 Mb/s link (11-05).
Parameters: engine (default the hula-hoop tl6_1119 engine), names (default by class count), conf 0.25, hz 5,
view_scale 0.5, quality 70.  ★ Python 3.8.
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tl_yolo_common as C  # noqa: E402   (copied next to this file by scripts/yolo_view.sh)
from trt_engine import TrtEngine  # noqa: E402

NAMES = {6: 'tl_red,tl_green,rh,yh,unknown,cone',          # tl6_1119 (hula hoop engine)
         4: 'tl_red,tl_green,unknown,cone',                 # tl_cone_1104 (traffic light engine)
         2: 'rh,yh'}
COLOURS = {'tl_red': (0, 0, 255), 'tl_green': (0, 200, 0), 'rh': (80, 80, 255), 'yh': (0, 220, 255),
           'unknown': (200, 200, 200), 'cone': (0, 140, 255)}


class YoloView(Node):
    def __init__(self):
        super().__init__('yolo_test_view')
        g = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.engine_path = g('engine', os.path.expanduser('~/junsu_vision/hoop_384x640_fp16.engine'))
        self.conf, self.dt = float(g('conf', 0.25)), 1.0 / float(g('hz', 5.0))
        self.scale, self.quality = float(g('view_scale', 0.5)), int(g('quality', 70))
        self.image_topic = g('image_topic', '/camera/color/image_raw')
        self.eng = TrtEngine(self.engine_path)
        self.H, self.W = self.eng.input_shape[2], self.eng.input_shape[3]
        out = self.eng.infer(np.zeros(self.eng.input_shape, np.float32))
        nc = int(np.asarray(out).shape[-2]) - 4
        names = g('names', '') or NAMES.get(nc, ','.join(str(i) for i in range(nc)))
        self.names = names.split(',')
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pub_view = self.create_publisher(CompressedImage, '/yolo_test/view/compressed', self.qos)
        self.pub_dets = self.create_publisher(String, '/yolo_test/dets', 5)
        self.sub = None
        self.last = 0.0
        self.n, self.t_fps, self.fps = 0, time.time(), 0.0
        self.create_timer(1.0, self.watchers)
        self.get_logger().info('yolo test view ready: %s (%d classes: %s) — waiting for a viewer'
                               % (os.path.basename(self.engine_path), nc, ', '.join(self.names)))

    def watchers(self):
        """Camera + engine only while someone watches."""
        on = self.pub_view.get_subscription_count() > 0
        if on and self.sub is None:
            self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, self.qos)
            self.get_logger().info('viewer connected — reading %s' % self.image_topic)
        elif not on and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.get_logger().info('no viewer — camera and engine idle')

    def cb_image(self, m):
        t = time.time()
        if t - self.last < self.dt:
            return
        self.last = t
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        bgr = a[..., ::-1].copy() if m.encoding.lower() == 'rgb8' else a.copy()
        blob, r, left, top = C.letterbox(bgr, (self.H, self.W))
        dets = C.decode(self.eng.infer(blob), r, left, top, bgr.shape[:2], conf_thr=self.conf)
        ms = 1000 * (time.time() - t)
        self.pub_dets.publish(String(data=json.dumps([[int(c), round(s, 3)] + [round(v) for v in b]
                                                      for c, s, *b in dets])))
        self.n += 1
        if t - self.t_fps >= 2.0:
            self.fps, self.n, self.t_fps = self.n / (t - self.t_fps), 0, t
        img = cv2.resize(bgr, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        for c, s, x1, y1, x2, y2 in dets:
            name = self.names[c] if 0 <= c < len(self.names) else str(c)
            col = COLOURS.get(name, (255, 255, 255))
            p1, p2 = (int(x1 * self.scale), int(y1 * self.scale)), (int(x2 * self.scale), int(y2 * self.scale))
            cv2.rectangle(img, p1, p2, col, 2 if s >= 0.5 else 1)
            cv2.putText(img, '%s %.2f' % (name, s), (p1[0], max(10, p1[1] - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1,
                        cv2.LINE_AA)
        cv2.putText(img, '%s  %d boxes  %.0f ms  %.1f Hz' % (os.path.basename(self.engine_path).split('_384')[0],
                                                          len(dets), ms, self.fps),
                    (4, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        ok, jpg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if ok:
            out = CompressedImage()
            out.header = m.header
            out.format = 'jpeg'
            out.data = jpg.tobytes()
            self.pub_view.publish(out)


def main():
    rclpy.init()
    n = YoloView()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
