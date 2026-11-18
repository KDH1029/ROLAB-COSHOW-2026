#!/usr/bin/env python3
"""Fake robot + route sender for the ego pass-through test (no robot, loopback only).

Integrates /cmd_vel (body frame) into /Odometry (camera_init), publishes a far-away dummy cloud,
and sends a route of points. mode=single: one /goal_pose per point, next only after arrival
(what a runner without pass-through does). mode=pass: /goal_path (goal + following points) then
/goal_pose for the first point only. Prints RESULT with the minimum speed while passing each
intermediate point and the total time.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
import struct

ROUTE = [(2.0, 0.0), (4.0, 0.0), (4.0, 2.0), (2.0, 2.0)]
ARRIVE = 0.30      # last point
PASS = 0.60        # intermediate points: a smoothed corner passes within this


class Fake(Node):
    def __init__(self):
        super().__init__('fake_route')
        self.mode = self.declare_parameter('mode', 'pass').value
        self.t_max = float(self.declare_parameter('t_max', 60.0).value)
        self.x = self.y = self.th = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.last_cmd = 0.0
        self.pub_odom = self.create_publisher(Odometry, '/Odometry', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_path = self.create_publisher(Path, '/goal_path', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.cloud)
        self.t0 = time.time()
        self.idx = 0          # route point being approached
        self.sent = -1
        self.min_v = [9.0] * len(ROUTE)
        self.done_t = None
        self.stopped_s = 0.0   # time below 0.05 m/s while the route is not finished (excluding the start)
        self.moved = False

    def cb_cmd(self, m):
        self.vx, self.vy, self.wz = m.linear.x, m.linear.y, m.angular.z
        self.last_cmd = time.time()

    def pose(self, x, y):
        p = PoseStamped(); p.header.frame_id = 'camera_init'; p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y; p.pose.orientation.w = 1.0
        return p

    def send(self, i):
        if self.mode == 'pass' and i == 0:
            path = Path(); path.header.frame_id = 'camera_init'; path.header.stamp = self.get_clock().now().to_msg()
            path.poses = [self.pose(*q) for q in ROUTE[i:]]
            self.pub_path.publish(path)
            time.sleep(0.05)
        self.pub_goal.publish(self.pose(*ROUTE[i]))
        self.sent = i

    def cloud(self):
        pts = [(15.0, 15.0, 0.3), (-15.0, 15.0, 0.3)]
        m = PointCloud2(); m.header.frame_id = 'camera_init'; m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name=n, offset=4 * k, datatype=7, count=1) for k, n in enumerate('xyz')]
        m.point_step, m.row_step, m.is_dense = 12, 12 * len(pts), True
        m.data = b''.join(struct.pack('fff', *p) for p in pts)
        self.pub_cloud.publish(m)

    def step(self):
        t = time.time() - self.t0
        if time.time() - self.last_cmd > 0.5:
            self.vx = self.vy = self.wz = 0.0
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += (c * self.vx - s * self.vy) * self.dt
        self.y += (s * self.vx + c * self.vy) * self.dt
        self.th += self.wz * self.dt
        o = Odometry(); o.header.frame_id = 'camera_init'; o.child_frame_id = 'body'
        o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        # twist left empty like FAST-LIO /Odometry: ego estimates the (world-frame) velocity from the poses
        self.pub_odom.publish(o)
        if t < 3.0 or self.done_t is not None:
            if self.done_t is not None or t > self.t_max:
                self.finish(t)
            return
        sp = math.hypot(self.vx, self.vy)
        if int(t * 2) != getattr(self, '_tr', -1):
            self._tr = int(t * 2)
            print('TRACE t=%.1f x=%.2f y=%.2f th=%.0f vx=%.2f vy=%.2f wz=%.2f idx=%d' % (t, self.x, self.y, math.degrees(self.th), self.vx, self.vy, self.wz, self.idx), flush=True)
        for i, q in enumerate(ROUTE):
            if math.hypot(q[0] - self.x, q[1] - self.y) < PASS:
                self.min_v[i] = min(self.min_v[i], sp)
        # single: the next goal only after ego has reached this one (its goal_tol 0.25), like a plain runner
        if sp > 0.2:
            self.moved = True
        if self.moved and sp < 0.05 and self.idx < len(ROUTE):
            self.stopped_s += self.dt
        lim = ARRIVE if (self.idx == len(ROUTE) - 1 or self.mode == 'single') else PASS
        if self.idx < len(ROUTE) and math.hypot(ROUTE[self.idx][0] - self.x, ROUTE[self.idx][1] - self.y) < lim:
            self.idx += 1
        if self.idx >= len(ROUTE):
            if sp < 0.02:
                self.done_t = t
            return
        if self.mode == 'single' and self.sent != self.idx:
            self.send(self.idx)
        if self.mode == 'pass' and self.sent < 0:
            self.send(0)
        if t > self.t_max:
            self.finish(t)

    def finish(self, t):
        mids = ['%.2f' % v for v in self.min_v[:-1]]
        print('RESULT mode=%s reached=%d/%d time=%.1fs stopped=%.2fs min_speed_at_intermediate=%s end=(%.2f,%.2f)' % (
            self.mode, self.idx, len(ROUTE), (self.done_t or t) - 3.0, self.stopped_s, mids, self.x, self.y), flush=True)
        raise SystemExit(0)


def main():
    rclpy.init(args=sys.argv)
    n = Fake()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
