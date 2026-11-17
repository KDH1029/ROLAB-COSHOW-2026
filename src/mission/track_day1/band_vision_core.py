"""Floor band edges from one D435 colour + depth pair (no ROS, Python 3.8 / numpy / OpenCV).

1. Floor plane from depth (RANSAC on a sparse grid) -> camera height, pitch, roll every frame,
   so the mount angle never needs calibrating.
2. Colour image warped to a bird's-eye view (BEV) of that plane, 2 cm cells.
3. Non-floor pixels (other robot, people, cones) masked out using depth.
4. Colour edges (Di Zenzo gradient on Lab, non-maximum suppression) -> points on the floor with
   their normal direction, in the robot frame (x forward, y left).

The lane tracker decides which edges are lane boundaries; nothing here knows the track.
"""
import math

import cv2
import numpy as np


DEFAULTS = dict(
    bev_res=0.02,          # m per BEV cell
    bev_x0=0.45, bev_x1=3.0,   # m ahead of the camera foot
    bev_y=1.8,             # m to each side
    depth_stride=8,        # depth pixels per plane sample
    plane_iters=60, plane_tol=0.02,
    h_min=0.20, h_max=0.80,        # camera height accepted (m)
    up_min=0.45,           # |n . expected_up| for a floor candidate
    nonfloor_h=0.05,       # m above the plane = not floor
    nonfloor_stride=4,
    edge_min=6.0,          # Lab gradient magnitude floor (per cell)
    edge_pct=92.0,         # keep the strongest (100 - pct)% of valid cells, but never below edge_min
    max_points=700,
    # a lane boundary separates two different coloured bands: sample across the edge at +-side_m;
    # both samples must be valid, neither white, and differ by >= side_de (8-bit Lab distance).
    # Arrows, lane numbers, start lines, glare and panel seams have the same band (or white) on a side.
    side_m=0.15, side_de=14.0, white_L=185.0, white_chroma=22.0,
    cam_x=0.466, cam_y=0.0096, cam_yaw=0.0,   # camera foot in the robot frame (m, rad)
    d2c=(0.015, 0.0, 0.0),                    # depth -> colour translation, optical frame (m)
)


def deproject_grid(depth_m, K, stride):
    h, w = depth_m.shape
    vs, us = np.mgrid[stride // 2:h:stride, stride // 2:w:stride]
    z = depth_m[vs, us]
    ok = (z > 0.25) & (z < 4.5)
    u, v, z = us[ok].astype(np.float64), vs[ok].astype(np.float64), z[ok].astype(np.float64)
    x = (u - K[0, 2]) / K[0, 0] * z
    y = (v - K[1, 2]) / K[1, 1] * z
    return np.stack([x, y, z], axis=1)


def fit_floor(pts, cfg, rng, prev=None):
    """RANSAC plane n.X + d = 0 with n pointing up (towards the camera), d = camera height.

    Returns (n, d, inlier_ratio) or None.
    """
    if len(pts) < 50:
        return None
    best = None
    tol = cfg['plane_tol']
    cand = []
    if prev is not None:
        cand.append(prev)
    for _ in range(cfg['plane_iters']):
        i = rng.choice(len(pts), 3, replace=False)
        a, b, c = pts[i]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        d = -float(n.dot(a))
        if d < 0:
            n, d = -n, -d
        cand.append((n, d))
    for n, d in cand:
        # camera optical frame: y down. "up" has a negative y component for any sane pitch.
        if -n[1] < cfg['up_min'] or not (cfg['h_min'] <= d <= cfg['h_max']):
            continue
        r = np.abs(pts.dot(n) + d)
        k = int((r < tol).sum())
        if best is None or k > best[2]:
            best = (n, d, k)
    if best is None:
        return None
    n, d, _ = best
    inl = pts[np.abs(pts.dot(n) + d) < tol]
    if len(inl) < 30:
        return None
    c = inl.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((inl - c).T))
    n = v[:, 0]
    if -n[1] < 0:
        n = -n
    d = -float(n.dot(c))
    if not (cfg['h_min'] <= d <= cfg['h_max']):
        return None
    return n, d, len(inl) / float(len(pts))


class BandVision(object):
    def __init__(self, cfg=None, seed=0):
        self.cfg = dict(DEFAULTS)
        if cfg:
            self.cfg.update(cfg)
        self.rng = np.random.default_rng(seed)
        self.prev_plane = None
        c = self.cfg
        self.nx = int(round((c['bev_x1'] - c['bev_x0']) / c['bev_res']))
        self.ny = int(round(2 * c['bev_y'] / c['bev_res']))
        # BEV cell centres: row r -> x (far at the top), col j -> y (left at col 0)
        r = np.arange(self.nx)
        j = np.arange(self.ny)
        self.cell_x = c['bev_x1'] - (r + 0.5) * c['bev_res']
        self.cell_y = c['bev_y'] - (j + 0.5) * c['bev_res']

    # ------------------------------------------------------------------ geometry
    def floor_frame(self, n_c, h):
        """Floor frame in the colour optical frame: origin under the camera, x = optical axis on the floor."""
        o = -h * n_c
        z = np.array([0.0, 0.0, 1.0])
        xf = z - z.dot(n_c) * n_c
        xf /= np.linalg.norm(xf)
        yf = np.cross(n_c, xf)
        return o, xf, yf

    def homography(self, Kc, o, xf, yf):
        """Colour pixel <- BEV cell (col, row)."""
        c = self.cfg
        corners_rc = np.array([[0, 0], [self.ny - 1, 0], [self.ny - 1, self.nx - 1], [0, self.nx - 1]], np.float64)
        src = []
        for col, row in corners_rc:
            x = c['bev_x1'] - (row + 0.5) * c['bev_res']
            y = c['bev_y'] - (col + 0.5) * c['bev_res']
            P = o + x * xf + y * yf
            if P[2] <= 0.05:
                return None
            src.append([Kc[0, 0] * P[0] / P[2] + Kc[0, 2], Kc[1, 1] * P[1] / P[2] + Kc[1, 2]])
        return cv2.getPerspectiveTransform(corners_rc.astype(np.float32), np.array(src, np.float32))

    # ------------------------------------------------------------------ main
    def process(self, bgr, depth_m, Kc, Kd):
        """bgr: HxWx3 uint8, depth_m: float32 metres (0 = invalid). Returns dict."""
        c = self.cfg
        out = dict(ok=False, pts=np.zeros((0, 4), np.float32), why='')
        pts_d = deproject_grid(depth_m, Kd, c['depth_stride'])
        fit = fit_floor(pts_d, c, self.rng, self.prev_plane)
        if fit is None:
            out['why'] = 'no floor plane'
            self.prev_plane = None
            return out
        n_d, h, ratio = fit
        self.prev_plane = (n_d, h)
        t = np.asarray(c['d2c'], np.float64)
        n_c = n_d                       # depth and colour optical frames are parallel on the D435
        h_c = h - float(n_c.dot(t))     # plane distance seen from the colour camera
        o, xf, yf = self.floor_frame(n_c, h_c)
        pitch = math.degrees(math.atan2(-n_c[2], -n_c[1]))   # + = looking down
        roll = math.degrees(math.atan2(n_c[0], -n_c[1]))
        out.update(h=h_c, pitch=pitch, roll=roll, floor_ratio=ratio)

        H = self.homography(Kc, o, xf, yf)
        if H is None:
            out['why'] = 'BEV outside the image'
            return out
        size = (self.ny, self.nx)
        flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
        bev = cv2.warpPerspective(bgr, H, size, flags=flags, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        hh, ww = bgr.shape[:2]

        # valid = inside the colour image and on the floor
        mask_img = np.full((hh, ww), 255, np.uint8)
        self._paint_nonfloor(mask_img, depth_m, Kd, Kc, n_d, h, t)
        valid = cv2.warpPerspective(mask_img, H, size, flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP,
                                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        valid = cv2.erode(valid, np.ones((5, 5), np.uint8)) > 0
        # edges need a wider margin: blur + Sobel reach ~4 cells into the mask border (robot, image edge)
        valid_edge = cv2.erode(valid.astype(np.uint8), np.ones((11, 11), np.uint8)) > 0

        lab = cv2.cvtColor(bev, cv2.COLOR_BGR2LAB).astype(np.float32)
        lab = cv2.GaussianBlur(lab, (5, 5), 1.0)
        sxx = np.zeros(lab.shape[:2], np.float32)
        syy = np.zeros_like(sxx)
        sxy = np.zeros_like(sxx)
        for ch, wgt in ((0, 1.0), (1, 2.0), (2, 2.0)):     # chroma counts more: blue|blue|yellow edges are hue changes
            gx = cv2.Sobel(lab[:, :, ch], cv2.CV_32F, 1, 0, ksize=3) * wgt
            gy = cv2.Sobel(lab[:, :, ch], cv2.CV_32F, 0, 1, ksize=3) * wgt
            sxx += gx * gx
            syy += gy * gy
            sxy += gx * gy
        # Di Zenzo: largest eigenvalue / direction of the colour structure tensor
        tr = sxx + syy
        det_term = np.sqrt(np.maximum((sxx - syy) ** 2 + 4 * sxy * sxy, 0))
        mag = np.sqrt(np.maximum(0.5 * (tr + det_term), 0)) / 8.0
        ang = 0.5 * np.arctan2(2 * sxy, sxx - syy)          # image-axis angle of the normal
        mag[~valid_edge] = 0
        vals = mag[valid_edge]
        if vals.size < 100:
            out['why'] = 'no valid floor in view'
            return out
        thr = max(c['edge_min'], float(np.percentile(vals, c['edge_pct'])))
        cand = mag > thr
        # non-maximum suppression along the normal (4 directions)
        q = (np.round(ang / (np.pi / 4)).astype(np.int32)) % 4
        off = {0: (1, 0), 1: (1, 1), 2: (0, 1), 3: (-1, 1)}   # (dcol, drow) along the normal; col = image x
        keep = np.zeros_like(cand)
        pad = np.pad(mag, 1)
        R, C = np.nonzero(cand)
        for k in range(4):
            sel = q[R, C] == k
            r, cc = R[sel], C[sel]
            dc, dr = off[k]
            a = pad[r + 1 + dr, cc + 1 + dc]
            b = pad[r + 1 - dr, cc + 1 - dc]
            m = mag[r, cc]
            ok = (m >= a) & (m >= b)
            keep[r[ok], cc[ok]] = True
        R, C = np.nonzero(keep)
        if len(R) and c['side_m'] > 0:
            k = c['side_m'] / c['bev_res']
            gc, gr = np.cos(ang[R, C]), np.sin(ang[R, C])
            hh_, ww_ = valid.shape
            c1 = np.clip(np.round(C + k * gc).astype(int), 0, ww_ - 1)
            r1 = np.clip(np.round(R + k * gr).astype(int), 0, hh_ - 1)
            c2 = np.clip(np.round(C - k * gc).astype(int), 0, ww_ - 1)
            r2 = np.clip(np.round(R - k * gr).astype(int), 0, hh_ - 1)
            a1, a2 = lab[r1, c1], lab[r2, c2]
            de = np.sqrt(((a1 - a2) ** 2).sum(1))

            def white(x):
                return (x[:, 0] > c['white_L']) & (np.hypot(x[:, 1] - 128, x[:, 2] - 128) < c['white_chroma'])
            side_ok = valid[r1, c1] & valid[r2, c2] & (de >= c['side_de']) & ~white(a1) & ~white(a2)
            out['n_side_rejected'] = int((~side_ok).sum())
            R, C = R[side_ok], C[side_ok]
        if len(R) == 0:
            out['why'] = 'no edges'
            return out
        if len(R) > c['max_points']:
            # strongest first, but spread: random subset weighted by magnitude
            w = mag[R, C].astype(np.float64)
            idx = self.rng.choice(len(R), c['max_points'], replace=False, p=w / w.sum())
            R, C = R[idx], C[idx]
        xf_ = self.cell_x[R]
        yf_ = self.cell_y[C]
        # normal angle in the floor frame: image col axis = -y, row axis = -x
        gcol = np.cos(ang[R, C])
        grow = np.sin(ang[R, C])
        nx_f = -grow
        ny_f = -gcol
        th_f = np.arctan2(ny_f, nx_f)
        # floor frame -> robot frame (camera foot at cam_x, cam_y, yawed by cam_yaw)
        cy, sy = math.cos(c['cam_yaw']), math.sin(c['cam_yaw'])
        xr = c['cam_x'] + cy * xf_ - sy * yf_
        yr = c['cam_y'] + sy * xf_ + cy * yf_
        th = th_f + c['cam_yaw']
        th = (th + np.pi / 2) % np.pi - np.pi / 2          # normals are directionless
        out['pts'] = np.stack([xr, yr, th, mag[R, C]], axis=1).astype(np.float32)
        out['ok'] = True
        out['n_valid'] = int(vals.size)
        out['thr'] = thr
        out['bev'] = bev
        out['valid'] = valid
        return out

    def _paint_nonfloor(self, mask_img, depth_m, Kd, Kc, n_d, h, t):
        """Zero the colour pixels that look at something standing on the floor (robot, person, cone)."""
        s = self.cfg['nonfloor_stride']
        P = deproject_grid(depth_m, Kd, s)
        if len(P) == 0:
            return
        height = P.dot(n_d) + h
        sel = height > self.cfg['nonfloor_h']
        if not np.any(sel):
            return
        Q = P[sel] + t
        Q = Q[Q[:, 2] > 0.1]
        u = (Kc[0, 0] * Q[:, 0] / Q[:, 2] + Kc[0, 2]).astype(np.int32)
        v = (Kc[1, 1] * Q[:, 1] / Q[:, 2] + Kc[1, 2]).astype(np.int32)
        hh, ww = mask_img.shape
        ok = (u >= 0) & (u < ww) & (v >= 0) & (v < hh)
        u, v = u[ok], v[ok]
        r = max(2, int(round(s * Kc[0, 0] / Kd[0, 0] * 1.2)))
        small = np.zeros_like(mask_img)
        small[v, u] = 255
        small = cv2.dilate(small, np.ones((2 * r + 1, 2 * r + 1), np.uint8))
        mask_img[small > 0] = 0
