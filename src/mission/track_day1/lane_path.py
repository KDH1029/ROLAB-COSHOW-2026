"""Lane centre line in the odometry frame, corrected by floor-edge points every frame.

No lane shape is assumed: the centre line is a polyline (5 cm spacing) from ~1 m behind the robot
to `ahead` m in front. Each frame, edge points are matched to the nearest lane boundary
(centre line +- known offsets, gate + normal direction), and their lateral residuals are fitted
as a smooth correction delta(s) along the line (hat functions every 0.5 m, second-difference
smoothing, ridge towards no change, Huber). The end is extended by continuing the last curvature,
so a straight that turns into a curve gets bent as soon as the curve comes into view.

This is what Nav2 RPP follows (published as nav_msgs/Path in the odometry frame).
Pure numpy, no ROS.
"""
import math

import numpy as np

import track_spec as T

DEFAULTS = dict(
    step=0.05, behind=1.0, ahead=3.6,
    knot=0.5,
    gate=0.18, gate_lost=0.26,
    normal_tol_deg=25.0,
    huber=0.04, sigma0=0.02, sigma_per_m=0.02,
    n_eff=40.0,
    ridge=1.0 / 0.08 ** 2,      # prior: no change, sigma 8 cm per frame
    smooth=1.0 / 0.03 ** 2,     # second difference between knots, sigma 3 cm
    max_corr=0.15,
    min_inliers=20,
    ext_fit=1.0,                # m of line used to estimate the curvature for extension
    kappa_max=0.9,
)


def wrap_half(a):
    return (a + np.pi / 2) % np.pi - np.pi / 2


class LanePath(object):
    def __init__(self, lane, cfg=None):
        self.cfg = dict(DEFAULTS)
        if cfg:
            self.cfg.update(cfg)
        self.offs = np.array([o for o, _ in T.boundary_offsets(lane)])
        self.pts = None
        self.last = dict(n=0, rms=None, ok=False, per=[0] * len(self.offs))
        self.last_tail = None
        self.miss = 0

    # ------------------------------------------------------------------ geometry helpers
    def init_at(self, x, y, yaw, kappa=0.0, lateral=0.0):
        """Robot pose in odom; lateral = robot's offset from the lane centre (+ left) if known."""
        c = self.cfg
        s = np.arange(-c['behind'], c['ahead'] + 1e-9, c['step'])
        cx = x + lateral * math.sin(yaw)
        cy = y - lateral * math.cos(yaw)
        if abs(kappa) < 1e-6:
            px, py = cx + s * math.cos(yaw), cy + s * math.sin(yaw)
        else:
            px = cx + (np.sin(yaw + kappa * s) - math.sin(yaw)) / kappa
            py = cy - (np.cos(yaw + kappa * s) - math.cos(yaw)) / kappa
        self.pts = np.stack([px, py], axis=1)

    def initialize(self, P, x, y, yaw, kappas=(0.0,), c_max=0.35, psi_max_deg=12.0, near=1.7):
        """Start the line where the edge points say the lane is (robot standing, odom frame).

        Grid over the lane centre's lateral position c (robot frame, + left), heading psi and curvature.
        |c| <= 0.35 m keeps it away from the lane-width alias (a 0.9 m shift also lines boundaries up).
        Returns (c, psi, kappa, inliers) or None.
        """
        if P is None or len(P) < self.cfg['min_inliers']:
            return None
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        px, py = P[:, 0] - x, P[:, 1] - y
        rx, ry = cy * px - sy * py, sy * px + cy * py          # robot frame
        ang = P[:, 2] - yaw
        # near points only: one shape (straight or arc) explains them; far ones may already be past a
        # straight/curve change and make wrong hypotheses tie with the right one
        sel = rx < near
        if sel.sum() < self.cfg['min_inliers']:
            sel = rx < near + 0.6
        rx, ry, ang = rx[sel], ry[sel], ang[sel]
        if len(rx) < self.cfg['min_inliers']:
            return None
        tol = 0.05
        best = None
        cs = np.arange(-c_max, c_max + 1e-9, 0.025)
        for k in kappas:
            for psi in np.radians(np.arange(-psi_max_deg, psi_max_deg + 1e-9, 1.0)):
                cps, sps = math.cos(psi), math.sin(psi)
                # all c at once: points relative to (0, c)
                dx = rx[None, :]
                dy = ry[None, :] - cs[:, None]
                u = cps * dx + sps * dy
                v = -sps * dx + cps * dy
                q = u * u + v * v
                root = np.sqrt(np.maximum(1.0 - 2.0 * k * v + k * k * q, 0.0))
                d = (2.0 * v - k * q) / (1.0 + root)
                if abs(k) < 1e-4:
                    nrm = np.full(d.shape, psi + np.pi / 2)
                else:
                    ccx = -sps / k
                    ccy = cs[:, None] + cps / k
                    nrm = np.arctan2(ry[None, :] - ccy, rx[None, :] - ccx)
                e = np.abs(d[:, :, None] - self.offs[None, None, :]).min(2)
                ok = (e < tol) & (np.abs(wrap_half(ang[None, :] - nrm)) < math.radians(self.cfg['normal_tol_deg']))
                n = ok.sum(1)
                cost = n - 0.5 * (np.where(ok, e, 0).sum(1) / tol)
                i = int(cost.argmax())
                if best is None or cost[i] > best[0]:
                    best = (float(cost[i]), float(cs[i]), float(psi), float(k), int(n[i]))
        if best is None or best[4] < self.cfg['min_inliers']:
            return None
        _, c, psi, k, n = best
        cxo = x - c * math.sin(yaw)
        cyo = y + c * math.cos(yaw)
        self.init_at(cxo, cyo, yaw + psi, kappa=k)
        # init_at puts the line through (cxo, cyo) starting `behind` m before it
        return c, psi, k, n

    def _frames(self):
        p = self.pts
        d = np.diff(p, axis=0)
        seg = np.hypot(d[:, 0], d[:, 1])
        s = np.concatenate([[0.0], np.cumsum(seg)])
        tang = np.arctan2(d[:, 1], d[:, 0])
        tang = np.concatenate([tang, tang[-1:]])
        return s, tang

    def project(self, qx, qy):
        """Station s, signed lateral d (+ left), tangent angle at nearest points (vectorised, local search)."""
        p = self.pts
        a = p[:-1]
        b = p[1:]
        ab = b - a
        L2 = np.maximum((ab ** 2).sum(1), 1e-12)
        s_cum, tang = self._frames()
        qx = np.atleast_1d(qx)
        qy = np.atleast_1d(qy)
        # brute force over segments (<= ~100 segments x <= 700 points)
        apx = qx[:, None] - a[None, :, 0]
        apy = qy[:, None] - a[None, :, 1]
        t = np.clip((apx * ab[None, :, 0] + apy * ab[None, :, 1]) / L2[None, :], 0, 1)
        dx = apx - t * ab[None, :, 0]
        dy = apy - t * ab[None, :, 1]
        dist2 = dx * dx + dy * dy
        i = dist2.argmin(1)
        ti = t[np.arange(len(i)), i]
        cross = ab[i, 0] * apy[np.arange(len(i)), i] - ab[i, 1] * apx[np.arange(len(i)), i]
        d = np.sign(cross) * np.sqrt(dist2[np.arange(len(i)), i])
        s = s_cum[i] + ti * np.sqrt(L2[i])
        return s, d, tang[i]

    def robot_station(self, x, y):
        s, d, th = self.project(np.array([x]), np.array([y]))
        return float(s[0]), float(d[0]), float(th[0])

    # ------------------------------------------------------------------ update
    def _tail_hypotheses(self, P, s_from):
        """Re-shape the line ahead of s_from as an arc (curvature from a small set) if that explains
        clearly more edge points than the current line. Fixes curve entry / exit, where continuing
        the old curvature is off by s^2/2R (1.2 m at 2 m on lane 2) - far outside any gate."""
        c = self.cfg
        s_cum, tang = self._frames()
        if s_cum[-1] - s_from < 0.5:
            return False
        tol = 0.06
        cos_tol = math.radians(c['normal_tol_deg'])
        x, y, ang = P[:, 0], P[:, 1], P[:, 2]

        def score_d(d, nrm):
            e = np.abs(d[:, None] - self.offs[None, :]).min(1)
            return (e < tol) & (np.abs(wrap_half(ang - nrm)) < cos_tol)

        s0, d0, t0 = self.project(x, y)
        base_ok = score_d(d0, t0 + np.pi / 2)
        best = (int(base_ok.sum()), None)
        kgrid = np.concatenate([np.linspace(-0.25, 0.25, 5), np.abs(self.offs[:2]).max() * 0 + self._kappa_set()])
        for st in np.arange(s_from, s_cum[-1] - 0.4, 0.25):
            i = int(np.searchsorted(s_cum, st))
            xs, ys, th = self.pts[i, 0], self.pts[i, 1], tang[i]
            cs, sn = math.cos(th), math.sin(th)
            u = cs * (x - xs) + sn * (y - ys)
            v = -sn * (x - xs) + cs * (y - ys)
            ahead = u > 0
            q = u * u + v * v
            for k in kgrid:
                root = np.sqrt(np.maximum(1.0 - 2.0 * k * v + k * k * q, 0.0))
                da = (2.0 * v - k * q) / (1.0 + root)
                if abs(k) < 1e-4:
                    na = np.full(len(x), th + np.pi / 2)
                else:
                    cx_, cy_ = xs - sn / k, ys + cs / k
                    na = np.arctan2(y - cy_, x - cx_)
                d = np.where(ahead, da, d0)
                nrm = np.where(ahead, na, t0 + np.pi / 2)
                n_ok = int(score_d(d, nrm).sum())
                if n_ok > best[0]:
                    best = (n_ok, (i, k))
        if best[1] is None or best[0] < max(c['min_inliers'], 1.15 * base_ok.sum()):
            return False
        i, k = best[1]
        xs, ys, th = self.pts[i, 0], self.pts[i, 1], tang[i]
        n = len(self.pts) - i - 1
        ss = np.arange(1, n + 1) * c['step']
        if abs(k) < 1e-6:
            ex, ey = xs + ss * math.cos(th), ys + ss * math.sin(th)
        else:
            ex = xs + (np.sin(th + k * ss) - math.sin(th)) / k
            ey = ys - (np.cos(th + k * ss) - math.cos(th)) / k
        self.pts = np.vstack([self.pts[:i + 1], np.stack([ex, ey], axis=1)])
        self.last_tail = (float(s_cum[i]), float(k), best[0], int(base_ok.sum()))
        return True

    def _kappa_set(self):
        return np.array([T.CURVATURE[1], T.CURVATURE[2], 0.5 * (T.CURVATURE[1] + T.CURVATURE[2])])

    def update(self, P, s_robot=None):
        """P: N x 4 [x, y, normal_angle, strength] in the odometry frame. Returns True if corrected."""
        c = self.cfg
        self.last = dict(n=0, rms=None, ok=False, per=[0] * len(self.offs))
        self.last_tail = None
        if self.pts is None or P is None or len(P) == 0:
            self.miss += 1
            return False
        self._tail_hypotheses(P, (s_robot if s_robot is not None else c['behind']) + 0.3)
        s, d, tang = self.project(P[:, 0], P[:, 1])
        diff = d[:, None] - self.offs[None, :]
        k = np.abs(diff).argmin(1)
        e = diff[np.arange(len(k)), k]
        gate = c['gate_lost'] if self.miss >= 5 else c['gate']
        nrm = tang + np.pi / 2
        s_end = self._frames()[0][-1]
        ok = (np.abs(e) < gate) & (np.abs(wrap_half(P[:, 2] - nrm)) < math.radians(c['normal_tol_deg'])) \
            & (s > 0.05) & (s < s_end - 0.05)
        if ok.sum() < c['min_inliers']:
            self.miss += 1
            self.last['n'] = int(ok.sum())
            return False
        s, e, k = s[ok], e[ok], k[ok]
        # range from the robot ~ station minus the robot's station (behind part has no points)
        rng = np.maximum(s - c['behind'], 0.3)
        sig = c['sigma0'] + c['sigma_per_m'] * rng
        knots = np.arange(0.0, s_end + c['knot'], c['knot'])
        nk = len(knots)
        # hat basis
        u = s / c['knot']
        j = np.clip(np.floor(u).astype(int), 0, nk - 2)
        f = u - j
        scale = min(1.0, c['n_eff'] / float(len(s)))
        D2 = np.zeros((max(nk - 2, 0), nk))
        for r in range(nk - 2):
            D2[r, r:r + 3] = (1, -2, 1)
        A0 = c['ridge'] * np.eye(nk) + c['smooth'] * D2.T.dot(D2)
        w_rob = np.ones(len(s))
        delta = np.zeros(nk)
        for _ in range(3):
            w = w_rob / sig ** 2 * scale
            A = A0.copy()
            bvec = np.zeros(nk)
            np.add.at(A, (j, j), w * (1 - f) ** 2)
            np.add.at(A, (j + 1, j + 1), w * f ** 2)
            np.add.at(A, (j, j + 1), w * (1 - f) * f)
            np.add.at(A, (j + 1, j), w * (1 - f) * f)
            np.add.at(bvec, j, w * (1 - f) * e)
            np.add.at(bvec, j + 1, w * f * e)
            delta = np.linalg.solve(A, bvec)
            r = e - (delta[j] * (1 - f) + delta[j + 1] * f)
            a = np.abs(r)
            w_rob = np.where(a < c['huber'], 1.0, c['huber'] / np.maximum(a, 1e-9))
        delta = np.clip(delta, -c['max_corr'], c['max_corr'])
        # apply along the line
        s_cum, tang_all = self._frames()
        dl = np.interp(s_cum, knots, delta)
        nx, ny = -np.sin(tang_all), np.cos(tang_all)
        self.pts = self.pts + np.stack([dl * nx, dl * ny], axis=1)
        self._resample()
        rms = float(np.sqrt(np.mean(np.minimum(r * r, 0.1 ** 2))))
        self.last = dict(n=int(len(s)), rms=rms, ok=rms < 0.06,
                         per=[int((k == i).sum()) for i in range(len(self.offs))])
        self.miss = 0
        return True

    # ------------------------------------------------------------------ keep the window around the robot
    def advance(self, x, y, extend=True):
        """Trim behind the robot and extend ahead by continuing the end curvature.

        extend=False when the lane has not been seen for a while: the robot then runs to the end of
        the line it already confirmed (<= `ahead` m) and stops there instead of guessing on."""
        c = self.cfg
        s_r, _, _ = self.robot_station(x, y)
        s_cum, tang = self._frames()
        keep = s_cum >= s_r - c['behind']
        self.pts = self.pts[keep]
        s_cum, tang = self._frames()
        need = c['behind'] + c['ahead']          # total length wanted
        have = s_cum[-1] - s_cum[0]
        if extend and have < need:
            kappa = self.end_curvature()
            n = int(math.ceil((need - have) / c['step']))
            x0, y0 = self.pts[-1]
            th0 = tang[-1]
            ss = np.arange(1, n + 1) * c['step']
            if abs(kappa) < 1e-6:
                ex, ey = x0 + ss * math.cos(th0), y0 + ss * math.sin(th0)
            else:
                ex = x0 + (np.sin(th0 + kappa * ss) - math.sin(th0)) / kappa
                ey = y0 - (np.cos(th0 + kappa * ss) - math.cos(th0)) / kappa
            self.pts = np.vstack([self.pts, np.stack([ex, ey], axis=1)])
        self._resample()

    def end_curvature(self):
        c = self.cfg
        s_cum, tang = self._frames()
        sel = s_cum >= s_cum[-1] - c['ext_fit']
        if sel.sum() < 4:
            return 0.0
        th = np.unwrap(tang[sel])
        k = np.polyfit(s_cum[sel], th, 1)[0]
        return float(np.clip(k, -c['kappa_max'], c['kappa_max']))

    def _resample(self):
        s_cum, _ = self._frames()
        if s_cum[-1] < 2 * self.cfg['step']:
            return
        ss = np.arange(0.0, s_cum[-1] + 1e-9, self.cfg['step'])
        self.pts = np.stack([np.interp(ss, s_cum, self.pts[:, 0]), np.interp(ss, s_cum, self.pts[:, 1])], axis=1)

    def truncate_ahead(self, x, y, remaining):
        """Cut the line `remaining` m ahead of the robot (lap end)."""
        s_r, _, _ = self.robot_station(x, y)
        s_cum, _ = self._frames()
        keep = s_cum <= s_r + max(remaining, 0.0)
        if keep.sum() >= 2:
            self.pts = self.pts[keep]
