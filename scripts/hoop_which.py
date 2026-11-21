#!/usr/bin/env python3
"""Which arch is red and which is yellow, as the runner sees it: 인코스 = ?, 아웃코스 = ? — debug, robot does not move.

    scripts/hoop_which.sh [seconds=15] [route=rec105326_lap1] [pose=x,y,yaw_deg]          (alias dwhich)

Stand the robot at the look point facing the arches first (dlookhere, then space + q there: it stays standing and
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
        self.order = order
        self.label = {}
        for n in order:
            hx, hy = z['branches'][n]
            b = math.degrees(math.atan2(math.sin(math.atan2(hy - pose[1], hx - pose[0]) - pose[2]),
                                        math.cos(math.atan2(hy - pose[1], hx - pose[0]) - pose[2])))
            self.label[n] = '%s(%s, %s %.0f°)' % ('인코스' if n == order[0] else '아웃코스', n.split('_')[-1],
                                                 '왼쪽' if b > 0 else '오른쪽', abs(b))
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

    def course_line(self):
        """{branch: [colours]} from the runner's choice for each colour, + the bearings seen."""
        got, bear = {}, {}
        for c, k in (('red', '빨강'), ('yellow', '노랑')):
            name, _ = self.decide(c)
            if self.seen[c]:
                v = self.seen[c][-1][1]
                bear[k] = ('%+.0f°' % math.degrees(math.atan2(math.sin(v - self.pose[2]), math.cos(v - self.pose[2])))
                           if isinstance(v, float) else v)
            if name is not None:
                got.setdefault(name, []).append(k)
                self.picks[c].append(name)
        return got, bear

    def fmt(self, got):
        out = []
        for n in self.order:
            cs = got.get(n, [])
            out.append('%s = %s' % (self.label[n], ' + '.join(cs) if cs else '?'))
        return '   ·   '.join(out)

    def tick(self):
        self.pub.publish(Bool(data=True))
        t = time.time()
        if t - self.last >= 1.0:
            self.last = t
            got, bear = self.course_line()
            warn = ''
            for n, cs in got.items():
                if len(cs) > 1:
                    warn = '   ★ 두 색이 모두 %s — 카드 방향이 애매함' % self.label[n]
            print('%3.0f s  %s   (보인 방향 %s, 답 %d)%s' % (t - self.t0, self.fmt(got),
                                                   ', '.join('%s %s' % kv for kv in bear.items()) or '없음', self.n, warn),
                  flush=True)
        if t - self.t0 > self.secs:
            for _ in range(3):
                self.pub.publish(Bool(data=False))
            print('── 결과 (%d초, 답 %d개)' % (self.secs, self.n))
            if self.n == 0:
                print('★ 답 없음: 훌라후프 노드가 꺼졌거나(dhoopon) 카메라가 꺼짐')
            final = {}
            for c, k in (('red', '빨강'), ('yellow', '노랑')):
                p = self.picks[c]
                if p:
                    top = max(set(p), key=p.count)
                    final.setdefault(top, []).append('%s(%d/%d초)' % (k, p.count(top), len(p)))
            for n in self.order:
                print('  %s : %s' % (self.label[n], ' + '.join(final.get(n, [])) or '? (그 색 카드를 3번 이상 못 봄)'))
            if any(len(v) > 1 for v in final.values()):
                print('  ★ 두 색이 같은 코스로 나옴 — 로봇이 보는 자리·방향이 틀렸거나 카드 상자가 엉뚱한 곳')
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
