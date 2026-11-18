"""Lane centre line of the rulebook track and the track <-> odom transform (no ROS, numpy, Python 3.8).

Track frame (track_spec): origin = track centre, x = long axis, counter-clockwise travel; station s = arc length
along the lane centre, 0 at the left end of the bottom straight (x = -STRAIGHT/2, y < 0).
The start pose comes from wall_align while standing; the odometry (FAST-LIO) carries it through the lap.
Rectangle and track are both symmetric under a 180 deg turn, so the wall twin does not matter: either one
gives the same lane, the same path relative to the robot and the same lap end.
"""
import math

import numpy as np

try:
    from . import track_spec as T
except ImportError:
    import track_spec as T


# ------------------------------------------------------------------------------------------------ SE(2)
def compose(a, b):
    """a o b for (x, y, yaw) tuples."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def apply(a, pts):
    """Transform (N, 2) points by a."""
    pts = np.asarray(pts, float)
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.column_stack([a[0] + c * pts[:, 0] - s * pts[:, 1], a[1] + s * pts[:, 0] + c * pts[:, 1]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ------------------------------------------------------------------------------------------- lane centre
def lap_len(lane):
    return T.LAP_LEN[lane]


def station(lane, x, y):
    """Arc length of the closest lane-centre point, 0 .. lap."""
    R, L = T.R_CENTER[lane], T.STRAIGHT / 2
    if abs(x) <= L:
        return (x + L) if y < 0 else (2 * L + math.pi * R + (L - x))
    if x > L:
        phi = math.atan2(y, x - L)
        return 2 * L + R * (phi + math.pi / 2)
    phi = math.atan2(y, x + L)
    if phi < 0:
        phi += 2 * math.pi
    return 4 * L + math.pi * R + R * (phi - math.pi / 2)


def pose_at(lane, s, off=0.0):
    """Lane-centre pose (x, y, yaw) at station s (any real s), shifted `off` to the left of travel."""
    R, L = T.R_CENTER[lane], T.STRAIGHT / 2
    s = s % lap_len(lane)
    if s < 2 * L:
        x, y, yaw = -L + s, -R, 0.0
    elif s < 2 * L + math.pi * R:
        phi = -math.pi / 2 + (s - 2 * L) / R
        x, y, yaw = L + R * math.cos(phi), R * math.sin(phi), phi + math.pi / 2
    elif s < 4 * L + math.pi * R:
        x, y, yaw = L - (s - 2 * L - math.pi * R), R, math.pi
    else:
        phi = math.pi / 2 + (s - 4 * L - math.pi * R) / R
        x, y, yaw = -L + R * math.cos(phi), R * math.sin(phi), phi + math.pi / 2
    return x - off * math.sin(yaw), y + off * math.cos(yaw), wrap(yaw)


class RulebookLoop:
    """The lane centre from the rulebook drawing (default path)."""

    def __init__(self, lane):
        self.lane, self.length, self.name = lane, lap_len(lane), 'rulebook'

    def pose_at(self, s, off=0.0):
        return pose_at(self.lane, s, off)

    def station(self, x, y):
        return station(self.lane, x, y)


class PinLoop:
    """A closed loop through pins (track frame), smoothed (periodic Catmull-Rom) and resampled every `ds`.
    Station = arc length along the loop, starting at `s0` at the first pin."""

    def __init__(self, xy, s0=0.0, ds=0.1, name='pins'):
        P = np.asarray(xy, float)[:, :2]
        if len(P) > 2 and math.hypot(*(P[0] - P[-1])) < 1e-6:
            P = P[:-1]
        if len(P) < 4:
            raise ValueError('need at least 4 pins')
        u = np.linspace(0.0, 1.0, 10, endpoint=False)[:, None]
        p0, p1, p2, p3 = np.roll(P, 1, 0), P, np.roll(P, -1, 0), np.roll(P, -2, 0)
        dense = []
        for a, b, c, d in zip(p0, p1, p2, p3):
            dense.append(0.5 * ((2 * b) + (-a + c) * u + (2 * a - 5 * b + 4 * c - d) * u ** 2 + (-a + 3 * b - 3 * c + d) * u ** 3))
        D = np.vstack(dense + [P[:1]])
        L = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(D, axis=0).T))])
        self.length = float(L[-1])
        n = max(8, int(round(self.length / ds)))
        sq = np.linspace(0.0, self.length, n, endpoint=False)
        self.pts = np.column_stack([np.interp(sq, L, D[:, 0]), np.interp(sq, L, D[:, 1])])
        self.s = sq
        self.s0, self.name = float(s0), name
        nxt = np.roll(self.pts, -1, 0)
        self.seg = nxt - self.pts
        self.seg_len = np.hypot(self.seg[:, 0], self.seg[:, 1])
        self.yaw = np.arctan2(self.seg[:, 1], self.seg[:, 0])

    def pose_at(self, s, off=0.0):
        u = (s - self.s0) % self.length
        i = min(int(np.searchsorted(self.s, u, side='right')) - 1, len(self.s) - 1)
        f = (u - self.s[i]) / max(1e-9, self.seg_len[i])
        x, y = self.pts[i] + f * self.seg[i]
        yaw = float(self.yaw[i])
        return x - off * math.sin(yaw), y + off * math.cos(yaw), yaw

    def station(self, x, y):
        d = np.array([x, y]) - self.pts
        f = np.clip((d * self.seg).sum(1) / np.maximum(1e-12, self.seg_len ** 2), 0.0, 1.0)
        e = d - f[:, None] * self.seg
        i = int(np.argmin((e * e).sum(1)))
        return self.s0 + self.s[i] + f[i] * self.seg_len[i]


def load_pins(path, lane):
    """Pins CSV (lane,station_m,x_m,y_m,yaw_deg; same start-relative track frame) -> PinLoop of that lane."""
    rows = []
    with open(path) as fh:
        head = fh.readline().strip().split(',')
        k = {n: i for i, n in enumerate(head)}
        for line in fh:
            v = line.strip().split(',')
            if len(v) < len(head) or int(float(v[k['lane']])) != lane:
                continue
            rows.append((float(v[k['station_m']]), float(v[k['x_m']]), float(v[k['y_m']])))
    if len(rows) < 10:
        raise ValueError('%s: %d pins for lane %d (need >= 10)' % (path, len(rows), lane))
    rows.sort()
    R = np.array(rows)
    return PinLoop(R[:, 1:3], s0=R[0, 0], name='pins %s (%d)' % (path.split('/')[-1], len(rows)))


def lateral_of(loop, x, y):
    """Signed offset (+ = left of travel) and heading of the closest loop point."""
    s = loop.station(x, y)
    cx, cy, cyaw = loop.pose_at(s)
    return -(x - cx) * math.sin(cyaw) + (y - cy) * math.cos(cyaw), cyaw, s


def smoothstep(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


class LanePlan:
    """The path for one run: lane centre from the start to start + lap + extra, in the track frame.

    The robot rarely stands on the centre: the first `blend_m` start at its own lateral offset and ease onto
    the centre (no sharp correction towards a line right after the start).
    """

    def __init__(self, lane, s_start, lat0, extra=1.2, blend_m=2.0, ds=0.10, loop=None):
        self.lane, self.s_start, self.lat0 = lane, s_start, lat0
        self.loop = loop or RulebookLoop(lane)
        self.s_end = s_start + self.loop.length + extra
        self.blend_m, self.ds = blend_m, ds

    def off(self, s):
        return self.lat0 * (1.0 - smoothstep((s - self.s_start) / self.blend_m)) if self.blend_m > 0 else 0.0

    def points(self, s_from, s_to=None):
        s_to = self.s_end if s_to is None else min(s_to, self.s_end)
        s_from = max(self.s_start - 0.5, s_from)
        n = max(2, int(math.ceil((s_to - s_from) / self.ds)) + 1)
        return np.array([self.loop.pose_at(s, self.off(s))[:2] for s in np.linspace(s_from, s_to, n)])


class Progress:
    """Station of the robot since the start, unwrapped over laps (closest centre point near the last one)."""

    def __init__(self, lane, s_start, loop=None):
        self.lane, self.s = lane, s_start
        self.loop = loop or RulebookLoop(lane)

    def feed(self, x, y, max_jump=1.0):
        lap = self.loop.length
        s0 = self.loop.station(x, y)
        k = round((self.s - s0) / lap)
        s = s0 + k * lap
        if abs(s - self.s) < max_jump:      # at 20 Hz a jump > 1 m is a bad pose, keep the old one
            self.s = s
        return self.s


# --------------------------------------------------------------------------------------------- start lock
class StartLock:
    """Pools wall_align poses taken while standing. Each one gives track <- odom = wall o odom^-1; the start is
    accepted when the last `n` of those agree within tol (robot still, walls consistent), and the pose is
    plausible for the lane: lateral offset and heading within the limits.

    offset = (dx, dy, dyaw): the drawn track centre relative to the wall-rectangle centre, in the start-relative
    frame (the robot's start on the lower half y < 0, travel +x along the start straight). Default 0."""

    def __init__(self, lane, n=5, tol_m=0.03, tol_deg=1.0, max_lat=0.35, max_head_deg=15.0, loop=None,
                 offset=(0.0, 0.0, 0.0)):
        self.loop = loop or RulebookLoop(lane)
        self.off_inv = inverse(tuple(offset))
        self.lane, self.n, self.tol_m, self.tol = lane, n, tol_m, math.radians(tol_deg)
        self.max_lat, self.max_head = max_lat, math.radians(max_head_deg)
        self.buf = []
        self.why = 'no wall pose yet'

    def feed(self, wall, odom):
        """wall: dict from wall_align.align (needs ok, x, y, yaw); odom: (x, y, yaw) of the robot at the same time.
        Returns None until accepted, then dict(T_track_odom, pose, s, lateral, heading_err)."""
        if not wall.get('ok'):
            self.why = 'wall: ' + wall.get('why', '?')
            return None
        tw = (wall['x'], wall['y'], wall['yaw'])
        if self.buf:   # keep the same 180 deg twin as the first sample
            p = self.buf[0][1]
            if math.hypot(tw[0] - p[0], tw[1] - p[1]) > math.hypot(-tw[0] - p[0], -tw[1] - p[1]):
                tw = (-tw[0], -tw[1], wrap(tw[2] + math.pi))
        tt = compose(self.off_inv, tw)        # wall frame -> track frame (drawn track centre)
        self.buf.append((compose(tt, inverse(odom)), tw, odom))
        self.buf = self.buf[-self.n:]
        if len(self.buf) < self.n:
            self.why = 'pooling %d/%d' % (len(self.buf), self.n)
            return None
        A = np.array([b[0] for b in self.buf])
        yc = math.atan2(np.sin(A[:, 2]).mean(), np.cos(A[:, 2]).mean())
        spread = np.hypot(A[:, 0] - A[:, 0].mean(), A[:, 1] - A[:, 1].mean()).max()
        yspread = np.abs([wrap(a - yc) for a in A[:, 2]]).max()
        if spread > self.tol_m or yspread > self.tol:
            self.why = 'not steady: %.1f cm / %.2f deg over %d poses' % (100 * spread, math.degrees(yspread), self.n)
            return None
        Tto = (float(A[:, 0].mean()), float(A[:, 1].mean()), yc)
        pose = compose(Tto, self.buf[-1][2])
        lat, cyaw, s = lateral_of(self.loop, pose[0], pose[1])
        head = wrap(pose[2] - cyaw)
        lat_rb = lateral_of(RulebookLoop(self.lane), pose[0], pose[1])[0]   # lane membership: rulebook lanes
        other = 2 if self.lane == 1 else 1
        s2 = station(other, pose[0], pose[1])
        c2 = pose_at(other, s2)
        lat2 = -(pose[0] - c2[0]) * math.sin(c2[2]) + (pose[1] - c2[1]) * math.cos(c2[2])
        if abs(lat2) < abs(lat_rb):
            self.why = 'robot is in lane %d (%.2f m from its centre), not lane %d' % (other, abs(lat2), self.lane)
            return None
        if abs(lat) > self.max_lat or abs(head) > self.max_head:
            self.why = 'placement: lateral %+.2f m, heading %+.1f deg (limits %.2f m, %.0f deg)' % (
                lat, math.degrees(head), self.max_lat, math.degrees(self.max_head))
            return None
        self.why = ''
        return dict(T_track_odom=Tto, pose=pose, s=s, lateral=lat, heading_err=head)
