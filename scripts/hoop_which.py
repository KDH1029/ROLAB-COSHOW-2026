#!/usr/bin/env python3
"""Which branch would the runner take for RED and for YELLOW, right now? — debug tool, the robot does not move.

    scripts/hoop_which.sh [seconds=15] [route=rec105326_lap1] [pose=x,y,yaw_deg]          (alias d1which)

Stand the robot at the look point facing the arches first (d1lookhere, then space + q there: it stays standing and
the hula-hoop node is back on). This turns the hoop vision on (/mission/hoop_active, like the runner), and feeds each
colour's answers to the runner's OWN branch choice (day1_runner_node.Day1Runner.look_decide, the last 5 answers),
as if hoop=red and as if hoop=yellow. Inner / outer = the branch whose hoop is nearer / farther from the track centre
(the same rule as hoop=in|out). Robot pose = the look point and its facing (pose= to give another one).
"""
import json
import math
import os
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import day1_runner_node as R  # noqa: E402   (only its look_decide is used)


def zone_of(route):
    for d in (os.path.join(ROOT, 'logs/day1/routes'), os.path.join(ROOT, 'routes')):
        f = os.path.join(d, route + '.zones.json')
        if os.path.exists(f):
            z = [q for q in json.load(open(f)) if q.get('type') == 'look']
            if z:
                return z[0], f
    sys.exit('★ no look point in %s.zones.json' % route)


class Which(Node):
    def __init__(self, secs, z, pose):
        super().__init__('hoop_which')
        self.secs, self.z, self.pose = secs, z, pose
        order = sorted(z['branches'], key=lambda n: math.hypot(*z['branches'][n]))
        self.label = {order[0]: '인코스(안쪽)', order[-1]: '아웃코스(바깥쪽)'}
        self.seen = {'red': [], 'yellow': []}
        self.picks = {'red': [], 'yellow': []}
        self.n = 0
        self.t0 = time.time()
        self.last = self.t0
        self.pub = self.create_publisher(Bool, '/mission/hoop_active', 2)
        self.create_subscription(String, '/mission/hoop_state', self.cb, 10)
        self.create_timer(0.5, self.tick)

    def decide(self, colour):
        fake = SimpleNamespace(look=dict(zone=self.z, pose=self.pose, yaw_now=self.pose[2]),
                               hoop_seen=self.seen[colour], hoop_pick=None)
        return R.Day1Runner.look_decide(fake)

    def cb(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.n += 1
        t = time.time()
        for c in ('red', 'yellow'):
            v = d.get(c)
            if isinstance(v, dict) and 'bearing_deg' in v:          # the runner's cb_hoop, as is
                self.seen[c].append((t, self.pose[2] + math.radians(float(v['bearing_deg']))))
            elif isinstance(v, dict) and v.get('side') in ('left', 'right'):
                self.seen[c].append((t, v['side']))

    def tick(self):
        self.pub.publish(Bool(data=True))
        t = time.time()
        if t - self.last >= 1.0:
            self.last = t
            parts = []
            for c, k in (('red', '빨강'), ('yellow', '노랑')):
                name, why = self.decide(c)
                if name is None:
                    parts.append('%s: 아직 (%d번 봄, 3번 필요)' % (k, len(self.seen[c])))
                else:
                    self.picks[c].append(name)
                    b = math.degrees(math.atan2(math.sin(self.seen[c][-1][1] - self.pose[2]),
                                                math.cos(self.seen[c][-1][1] - self.pose[2]))) \
                        if isinstance(self.seen[c][-1][1], float) else self.seen[c][-1][1]
                    parts.append('%s → %s %s (보인 방향 %s)' % (k, name, self.label.get(name, ''),
                                                          ('%+.0f°' % b) if isinstance(b, float) else b))
            print('%3.0f s  답 %d  |  %s' % (t - self.t0, self.n, '  |  '.join(parts)), flush=True)
        if t - self.t0 > self.secs:
            for _ in range(3):
                self.pub.publish(Bool(data=False))
            print('── 결과 (%d초, 답 %d개)' % (self.secs, self.n))
            if self.n == 0:
                print('★ 답 없음: 훌라후프 노드가 꺼졌거나(d1hoopon) 카메라가 꺼짐')
            for c, k in (('red', '빨강'), ('yellow', '노랑')):
                p = self.picks[c]
                if p:
                    top = max(set(p), key=p.count)
                    print('%s 받으면 → %s %s  (%d/%d초 같은 결론)' % (k, top, self.label.get(top, ''), p.count(top), len(p)))
                else:
                    print('%s 받으면 → 결정 못 함 (카드가 3번 이상 안 보임) → 주행기는 10초 뒤 무작위' % k)
            raise SystemExit


def main():
    kv = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    z, f = zone_of(kv.get('route', 'rec105326_lap1'))
    if 'pose' in kv:
        x, y, yd = [float(v) for v in kv['pose'].split(',')]
    else:
        (x, y), yd = z['at'], z['yaw_deg']
    pose = (x, y, math.radians(yd))
    print('look point %s: (%.2f, %.2f) facing %.0f deg — branches:' % (os.path.basename(f), x, y, yd))
    order = sorted(z['branches'], key=lambda n: math.hypot(*z['branches'][n]))
    for n in order:
        hx, hy = z['branches'][n]
        b = math.degrees(math.atan2(math.sin(math.atan2(hy - y, hx - x) - pose[2]), math.cos(math.atan2(hy - y, hx - x) - pose[2])))
        print('  %s %s: hoop (%.2f, %.2f), %.0f deg %s of straight ahead, %.1f m'
              % (n, '인코스(안쪽)' if n == order[0] else '아웃코스(바깥쪽)', hx, hy, abs(b), 'LEFT' if b > 0 else 'RIGHT',
                 math.hypot(hx - x, hy - y)))
    print('  → the runner takes the branch whose hoop is nearest to where the card is seen')
    rclpy.init()
    n = Which(float(kv.get('seconds', 15)), z, pose)
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
