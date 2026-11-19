#!/usr/bin/env python3
"""NUC window for the YOLO test view (/yolo_test/view/compressed from ~/yolo_test on the internal PC).
Keys: q / Esc quit · s save the frame (logs/yolo_test/) · space pause."""
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs', 'yolo_test')


class Viewer(Node):
    def __init__(self):
        super().__init__('yolo_test_viewer')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CompressedImage, '/yolo_test/view/compressed', self.cb, q)
        self.img, self.t, self.n = None, 0.0, 0

    def cb(self, m):
        self.img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        self.t = time.time()
        self.n += 1


def main():
    rclpy.init()
    v = Viewer()
    win = 'YOLO test (q quit, s save, space pause)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 848, 480)
    blank = np.zeros((240, 424, 3), np.uint8)
    paused, shown, t0 = False, None, time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(v, timeout_sec=0.02)
            if not paused:
                if v.img is not None and time.time() - v.t < 2.0:
                    shown = v.img
                else:
                    shown = blank.copy()
                    cv2.putText(shown, 'waiting for frames... %.0f s' % (time.time() - t0), (10, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(win, shown)
            k = cv2.waitKey(10) & 0xFF
            if k in (ord('q'), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord(' '):
                paused = not paused
            if k == ord('s') and shown is not None:
                os.makedirs(OUT, exist_ok=True)
                f = os.path.join(OUT, 'yolo_%s.jpg' % time.strftime('%H%M%S'))
                cv2.imwrite(f, shown)
                print('saved', os.path.normpath(f))
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        print('frames received: %d' % v.n)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
