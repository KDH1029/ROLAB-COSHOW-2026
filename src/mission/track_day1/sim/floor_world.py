"""Synthetic CO SHOW floor + D435 colour/depth renderer for closed-loop tests (no ROS).

World frame = track centre, x along the long axis, y up in the drawing, z up. Colours sampled from
the rulebook drawing (fig. 1). The real print under hall lights will differ; the vision side only
uses colour edges, and tests vary gain / tint / noise.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import track_spec as T  # noqa: E402

RGB = dict(lane1=(31, 79, 151), lane2=(65, 148, 214), yellow=(254, 224, 42), green=(140, 206, 36),
           board=(81, 81, 81), pgray=(144, 144, 144), white=(250, 252, 255), venue=(150, 147, 138),
           wall=(120, 118, 115), robot=(45, 45, 48))
BOARD_HX, BOARD_HY = 4.5, 3.5


def stadium_r(x, y):
    ax = np.abs(x) - T.STRAIGHT / 2
    return np.where(ax <= 0, np.abs(y), np.hypot(np.maximum(ax, 0), y))


def lane_dir(x, y):
    """Unit travel direction (CCW) at a point."""
    ax = np.abs(x) - T.STRAIGHT / 2
    on_straight = ax <= 0
    # straight: top goes -x, bottom +x ; curve: tangent of the circle about (+-1, 0), CCW
    cx = np.sign(x) * T.STRAIGHT / 2
    tx = np.where(on_straight, np.where(y > 0, -1.0, 1.0), -(y))
    ty = np.where(on_straight, 0.0, (x - cx))
    n = np.hypot(tx, ty) + 1e-12
    return tx / n, ty / n


def _arrows():
    """(cx, cy, dirx, diry) of the floor arrows, roughly where fig. 1 has them."""
    a = []
    for lane in (1, 2):
        r = T.R_CENTER[lane]
        a.append((0.0, r, -1.0, 0.0))                      # top straight
        a.append((0.6, -r, 1.0, 0.0))                      # bottom straight
        a.append((T.STRAIGHT / 2 + r, 0.0, 0.0, 1.0))      # right curve, going up
        a.append((-T.STRAIGHT / 2 - r, 0.0, 0.0, -1.0))    # left curve, going down
    return a


SEAMS = []   # [(axis, positions)] thin dark gaps between foamex panels, e.g. [('x', [...]), ('y', [...])]


def floor_rgb(x, y, start_lines=()):
    """Vectorised floor colour (float 0..255, shape x.shape + (3,))."""
    out = np.empty(x.shape + (3,), np.float32)
    out[...] = RGB['venue']
    # venue floor texture
    tex = (np.sin(x * 37.0) * np.sin(y * 41.0)) * 6.0
    out += tex[..., None]
    on_board = (np.abs(x) <= BOARD_HX) & (np.abs(y) <= BOARD_HY)
    out[on_board] = RGB['board']
    r = stadium_r(x, y)
    out[on_board & (r < T.R_OUTER)] = RGB['lane1']
    out[r < T.R_LANE12] = RGB['lane2']
    out[r < T.R_INFIELD] = RGB['yellow']
    green = (np.abs(x) <= 1.5) & (np.abs(y) <= 0.75)
    out[green] = RGB['green']
    pbox = (np.abs(x + 0.75) <= 0.4) & (np.abs(y) <= 0.4)
    out[pbox] = RGB['pgray']
    lw = 0.025
    white = (np.abs(r - T.R_LANE12) < lw / 2) | (np.abs(r - T.R_INFIELD) < lw / 2)
    for (cx, cy, dx, dy) in _arrows():
        u = (x - cx) * dx + (y - cy) * dy
        v = -(x - cx) * dy + (y - cy) * dx
        shaft = (u > -0.30) & (u < 0.12) & (np.abs(v) < 0.035)
        head = (u >= 0.12) & (u < 0.30) & (np.abs(v) < (0.30 - u) * 0.55)
        white |= shaft | head
    for lane, xs in ((1, -0.6), (2, -0.6)):              # lane numbers, bottom straight
        r0 = T.R_CENTER[lane]
        white |= (np.abs(x - xs) < 0.05) & (np.abs(y + r0) < 0.18)
    for (lane, sx, side) in start_lines:                   # start/finish lines across one lane
        rc = T.R_CENTER[lane]
        white |= (np.abs(x - sx) < 0.025) & (np.abs(y - side * rc) < T.LANE_W / 2)
    out[white] = RGB['white']
    for axis, pos in SEAMS:
        q = x if axis == 'x' else y
        for p0 in pos:
            out[on_board & (np.abs(q - p0) < 0.004)] *= 0.55
    return out


def rot_body_cam(pitch_down):
    c, s = math.cos(pitch_down), math.sin(pitch_down)
    xr = np.array([0.0, -1.0, 0.0])
    zf = np.array([c, 0.0, -s])
    yd = np.cross(zf, xr)
    return np.stack([xr, yd, zf], axis=1)


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(roll), math.sin(roll), math.cos(pitch),
                                                math.sin(pitch), math.cos(yaw), math.sin(yaw)))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz.dot(Ry).dot(Rx)


class Camera(object):
    def __init__(self, w=848, h=480, hfov_deg=69.0, ds=1):
        self.w, self.h = w // ds, h // ds
        f = (self.w / 2.0) / math.tan(math.radians(hfov_deg) / 2)
        self.K = np.array([[f, 0, self.w / 2.0], [0, f, self.h / 2.0], [0, 0, 1]])
        u, v = np.meshgrid(np.arange(self.w) + 0.5, np.arange(self.h) + 0.5)
        self.rays = np.stack([(u - self.K[0, 2]) / f, (v - self.K[1, 2]) / f, np.ones_like(u)], axis=-1)


def box_hits(o, d, boxes):
    """Nearest positive hit distance (in units of d) of rays o + t d against yawed boxes. inf = none."""
    best = np.full(d.shape[:-1], np.inf)
    for (bx, by, byaw, lx, ly, hz) in boxes:
        c, s = math.cos(-byaw), math.sin(-byaw)
        ox = c * (o[0] - bx) - s * (o[1] - by)
        oy = s * (o[0] - bx) + c * (o[1] - by)
        oz = o[2]
        dx = c * d[..., 0] - s * d[..., 1]
        dy = s * d[..., 0] + c * d[..., 1]
        dz = d[..., 2]
        tmin = np.full(best.shape, -np.inf)
        tmax = np.full(best.shape, np.inf)
        for oc, dc, lo, hi in ((ox, dx, -lx / 2, lx / 2), (oy, dy, -ly / 2, ly / 2), (oz, dz, 0.0, hz)):
            with np.errstate(divide='ignore', invalid='ignore'):
                t1 = (lo - oc) / dc
                t2 = (hi - oc) / dc
            par = np.abs(dc) < 1e-12
            inside = (oc >= lo) & (oc <= hi)
            t1 = np.where(par, np.where(inside, -np.inf, np.inf), t1)
            t2 = np.where(par, np.where(inside, np.inf, -np.inf), t2)
            tmin = np.maximum(tmin, np.minimum(t1, t2))
            tmax = np.minimum(tmax, np.maximum(t1, t2))
        hit = (tmax >= tmin) & (tmin > 0)
        best = np.where(hit & (tmin < best), tmin, best)
    return best


def render(cam, cam_pos, R_wc, boxes=(), start_lines=(), gain=1.0, tint=(1.0, 1.0, 1.0), noise=3.0, rng=None,
           depth=False):
    """Colour (BGR uint8) or z-depth (float32 m) image from a camera at cam_pos with optical->world R_wc."""
    d = cam.rays.dot(R_wc.T)
    t_floor = np.where(d[..., 2] < -1e-6, -cam_pos[2] / np.minimum(d[..., 2], -1e-6), np.inf)
    t_box = box_hits(cam_pos, d, boxes) if boxes else np.full(t_floor.shape, np.inf)
    t = np.minimum(t_floor, t_box)
    if depth:
        z = np.where(np.isfinite(t) & (t < 6.0), t, 0.0).astype(np.float32)   # rays have optical z = 1
        if rng is not None:
            z = np.where(z > 0, z + rng.normal(0, 0.002, z.shape) * z * z, 0).astype(np.float32)
        return z
    px = cam_pos[0] + t_floor * d[..., 0]
    py = cam_pos[1] + t_floor * d[..., 1]
    finite = np.isfinite(t_floor)
    px = np.where(finite, px, 0.0)
    py = np.where(finite, py, 0.0)
    img = floor_rgb(px, py, start_lines)
    img[~finite] = RGB['wall']
    img[t_box < t_floor] = RGB['robot']
    # light falloff with distance + vignette + gain/tint
    dist = np.where(finite, t_floor, 8.0)
    img *= (1.0 - 0.04 * np.clip(dist - 1.0, 0, 6))[..., None]
    vy, vx = np.mgrid[0:cam.h, 0:cam.w]
    vig = 1.0 - 0.25 * (((vx - cam.w / 2) / cam.w) ** 2 + ((vy - cam.h / 2) / cam.h) ** 2)
    img *= vig[..., None] * gain * np.array(tint, np.float32)
    if rng is not None and noise > 0:
        img += rng.normal(0, noise, img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    return img[..., ::-1].copy()     # RGB -> BGR


class Rig(object):
    """Robot body -> D435 colour/depth cameras."""

    def __init__(self, pitch_deg=20.0, cam_x=0.466, cam_y=0.0096, cam_z=0.43, ds=1):
        self.color = Camera(848, 480, 69.0, ds)
        self.depth = Camera(848, 480, 87.0, ds)
        self.pitch = math.radians(pitch_deg)
        self.mount = np.array([cam_x, cam_y, cam_z])
        self.R_bc = rot_body_cam(self.pitch)

    def poses(self, x, y, yaw, body_roll=0.0, body_pitch=0.0, dz=0.0):
        R_wb = rpy_matrix(body_roll, body_pitch, yaw)
        pos = np.array([x, y, 0.0]) + R_wb.dot(self.mount) + np.array([0, 0, dz])
        return pos, R_wb.dot(self.R_bc)

    def render_pair(self, x, y, yaw, boxes=(), rng=None, **kw):
        pos, R = self.poses(x, y, yaw, kw.pop('body_roll', 0.0), kw.pop('body_pitch', 0.0), kw.pop('dz', 0.0))
        bgr = render(self.color, pos, R, boxes=boxes, rng=rng, **kw)
        dep = render(self.depth, pos, R, boxes=boxes, rng=rng, depth=True)
        return bgr, dep
