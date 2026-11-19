#!/usr/bin/env python3
"""Hula-hoop vision check on the real camera — the robot does not move.

    scripts/hoop_check.sh [seconds=20]        (alias d1hc)

Turns the hoop node on the way the runner does at the look point (/mission/hoop_active True, re-sent), listens to
/mission/hoop_state and prints, every second, how many answers came and how often red / yellow were in them
(confidence, bearing + = left). At the end: a summary and hoop_active False. Point the robot at the arch first.
"""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class HoopCheck(Node):
    def __init__(self, secs):
        super().__init__('hoop_check')
        self.secs = secs
        self.pub = self.create_publisher(Bool, '/mission/hoop_active', 2)
        self.create_subscription(String, '/mission/hoop_state', self.cb, 10)
        self.t0 = time.time()
        self.rows, self.win = [], []
        self.last_print = self.t0
        self.create_timer(0.5, self.tick)

    def cb(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        self.rows.append(d)
        self.win.append(d)

    def tick(self):
        self.pub.publish(Bool(data=True))
        t = time.time()
        if t - self.last_print >= 1.0:
            self.last_print = t
            n = len(self.win)
            parts = []
            for c in ('red', 'yellow'):
                hit = [d[c] for d in self.win if c in d]
                parts.append('%s %d/%d%s' % (c, len(hit), n, (' conf %.2f bearing %+.1f deg' % (
                    np.mean([h['conf'] for h in hit]), np.mean([h['bearing_deg'] for h in hit]))) if hit else ''))
            print('%4.0f s  answers %d · %s' % (t - self.t0, n, ' · '.join(parts)), flush=True)
            self.win = []
        if t - self.t0 > self.secs:
            for _ in range(3):
                self.pub.publish(Bool(data=False))
            n = len(self.rows)
            print('── %d answers in %.0f s (node: 5 Hz expected)' % (n, self.secs))
            if n == 0:
                print('★ no answer: hoop node not running (d1hoop / day1.sh hoop start) or camera off')
            for c in ('red', 'yellow'):
                hit = [d[c] for d in self.rows if c in d]
                if hit:
                    cf = [h['conf'] for h in hit]
                    print('%-6s seen in %d/%d (%.0f %%), conf median %.2f (min %.2f), bearing %+.1f deg, box width %s px'
                          % (c, len(hit), n, 100.0 * len(hit) / max(1, n), np.median(cf), min(cf),
                             np.median([h['bearing_deg'] for h in hit]),
                             int(np.median([h['box'][2] - h['box'][0] for h in hit])) if 'box' in hit[0] else '-'))
                else:
                    print('%-6s never seen' % c)
            raise SystemExit


def main():
    secs = float(dict(a.split('=') for a in sys.argv[1:] if '=' in a).get('seconds', 20))
    rclpy.init()
    n = HoopCheck(secs)
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        for _ in range(3):
            n.pub.publish(Bool(data=False))
        time.sleep(0.2)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
