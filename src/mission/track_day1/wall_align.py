"""Start pose from the rectangular arena wall (no map, no ROS; numpy only, Python 3.8).

The robot stands still on its start mark. One or a few XT16 frames (points in the robot base frame,
x forward, z up) are enough: the 0.6 m wall around the track is a rectangle WALL_LX x WALL_LY with the
same centre as the track (track_spec). align() finds the robot pose (x, y, yaw) in the track frame.

  1. floor plane by RANSAC on low points -> height above the floor
  2. wall band: 0.08 .. WALL_H - 0.05 above the floor, 0.5 .. 12 m away (body, other robot's feet,
     people behind the wall above 0.6 m are mostly gone; what remains of the other robot is a small
     blob that the histogram peaks and the inlier fit ignore)
  3. yaw: rotate the band points, sharpness of the x and y histograms (walls = two thin peaks per axis)
  4. walls: per axis the peak pair on either side of the robot whose spacing matches LX or LY
  5. refine: least squares of yaw and the four wall offsets on the points within inlier_m of a wall
  6. pose: the robot is at minus the wall-centre in the rotated frame

A rectangle looks the same turned 180 deg, and so does the track: the two answers give the same lane
centre relative to the robot, so either will do at the start; while moving, `prior` keeps the same one.

Gates: measured wall spacing within dim_tol of LX/LY on the axes where both walls were seen, enough inlier
points on at least three walls, inlier rms small. On failure: ok=False and the reason.
"""
import math

import numpy as np

try:
    from . import track_spec as T
except ImportError:  # run as a script / from the sim folder
    import track_spec as T

DEFAULTS = dict(
    floor_z_range=(-0.70, -0.05),   # base frame, where the floor is looked for (Go2 base ~0.3 m above it)
    floor_tol=0.03,
    band=(0.08, T.WALL_H - 0.05),   # m above the floor
    r_range=(0.5, 12.0),
    yaw_step_deg=1.0,
    bin_m=0.03,
    dim_tol=0.12,                   # m, measured wall spacing vs WALL_LX / WALL_LY (real arena: 9.00 x 7.01)
    inlier_m=0.08,
    min_wall_pts=40,
    max_pts=4000,                   # wall-band points kept (random subset): yaw search cost
    max_rms=0.04,
)


# XT16 in base_link (go2_bridge bridge_params.yaml, same numbers as FAST-LIO extrinsic)
LIDAR_EXT = dict(x=0.25, y=0.0, z=0.1588, roll=0.012330, pitch=0.005579, yaw=1.57079632679)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


R_LIDAR = _rpy(LIDAR_EXT['roll'], LIDAR_EXT['pitch'], LIDAR_EXT['yaw'])
T_LIDAR = np.array([LIDAR_EXT['x'], LIDAR_EXT['y'], LIDAR_EXT['z']])


def lidar_to_base(P):
    """(N, 3) XT16 points (hesai_lidar frame, /lidar_points) -> base_link."""
    return np.asarray(P, float)[:, :3] @ R_LIDAR.T + T_LIDAR


def fit_floor(P, cfg, rng):
    """RANSAC plane on low points. Returns (n, d) with n.z > 0, height = P @ n + d."""
    lo = P[(P[:, 2] > cfg['floor_z_range'][0]) & (P[:, 2] < cfg['floor_z_range'][1])]
    if len(lo) > 3000:
        lo = lo[rng.choice(len(lo), 3000, replace=False)]
    if len(lo) < 50:
        return None
    best, best_n = None, 0
    for _ in range(80):
        a, b, c = lo[rng.choice(len(lo), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        if n[2] < 0:
            n = -n
        if n[2] < 0.9:              # within ~25 deg of level
            continue
        d = -n @ a
        k = int((np.abs(lo @ n + d) < cfg['floor_tol']).sum())
        if k > best_n:
            best, best_n = (n, d), k
    if best is None:
        return None
    n, d = best
    inl = lo[np.abs(lo @ n + d) < cfg['floor_tol']]
    c = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - c)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    return n, -n @ c


def _sharpness(u, bin_m):
    lo, hi = u.min(), u.max()
    h, _ = np.histogram(u, bins=max(8, int((hi - lo) / bin_m) + 1), range=(lo, hi + bin_m))
    return float((h.astype(float) ** 2).sum())


def _peaks(u, bin_m, min_pts):
    """Histogram peaks (centre, count), strongest first."""
    lo, hi = u.min() - bin_m, u.max() + bin_m
    h, e = np.histogram(u, bins=int((hi - lo) / bin_m) + 1, range=(lo, hi))
    hs = h.astype(float)
    hs = hs + 0.5 * (np.roll(hs, 1) + np.roll(hs, -1))
    idx = [i for i in range(1, len(hs) - 1) if hs[i] >= hs[i - 1] and hs[i] > hs[i + 1] and h[i - 1:i + 2].sum() >= min_pts]
    idx.sort(key=lambda i: -hs[i])
    return [(0.5 * (e[i] + e[i + 1]), int(h[i - 1:i + 2].sum())) for i in idx]


def _wall_pair(u, L, cfg):
    """Walls on one axis: (neg, pos) offsets with pos - neg ~ L. A missing wall is put at L from the other."""
    pk = _peaks(u, cfg['bin_m'], cfg['min_wall_pts'] // 2)
    neg = [p for p in pk if p[0] < -0.2]
    pos = [p for p in pk if p[0] > 0.2]
    best = None
    for a, na in neg[:6]:
        for b, nb in pos[:6]:
            err = abs((b - a) - L)
            if err < cfg['dim_tol']:
                score = na + nb - 200.0 * err
                if best is None or score > best[0]:
                    best = (score, a, b, 2)
    if best is not None:
        return best[1], best[2], best[3]
    # one wall only (the other hidden): the strongest peak that leaves the robot inside
    for c, _ in pk[:4]:
        if c < 0 and L + c > 0.2:
            return c, c + L, 1
        if c > 0 and c - L < -0.2:
            return c - L, c, 1
    return None


def align(points_list, cfg=None, seed=0, prior=None):
    """points_list: list of (N, 3) arrays in the robot base frame (robot standing still).

    Returns dict(ok, x, y, yaw, why, walls_seen, dims, rms, n_in, floor_h). (x, y, yaw) = robot in the
    track frame (centre origin, x along the long axis), heading counter-clockwise.
    prior = (x, y, yaw) of the last pose: of the two 180 deg twins the one nearer to it is returned
    (without a prior: the twin with y <= 0, i.e. on the lower half of the drawing).
    """
    c = dict(DEFAULTS)
    if cfg:
        c.update(cfg)
    rng = np.random.default_rng(seed)
    P = np.concatenate([np.asarray(p, float)[:, :3] for p in points_list])
    P = P[np.isfinite(P).all(1)]
    fl = fit_floor(P, c, rng)
    if fl is None:
        return dict(ok=False, why='no floor plane')
    n, d = fl
    h = P @ n + d
    r = np.hypot(P[:, 0], P[:, 1])
    m = (h > c['band'][0]) & (h < c['band'][1]) & (r > c['r_range'][0]) & (r < c['r_range'][1])
    Q = P[m, :2]
    if len(Q) > c['max_pts']:
        Q = Q[rng.choice(len(Q), c['max_pts'], replace=False)]
    if len(Q) < 4 * c['min_wall_pts']:
        return dict(ok=False, why='few wall-band points (%d)' % len(Q))

    # 3. yaw of the wall directions (mod 90 deg)
    def rot(th):
        cs, sn = math.cos(th), math.sin(th)
        return Q @ np.array([[cs, -sn], [sn, cs]])   # rows: points rotated by -th
    best_th, best_s = 0.0, -1.0
    for th in np.deg2rad(np.arange(0.0, 90.0, c['yaw_step_deg'])):
        R = rot(th)
        s = _sharpness(R[:, 0], c['bin_m']) + _sharpness(R[:, 1], c['bin_m'])
        if s > best_s:
            best_th, best_s = th, s
    for step in (0.2, 0.05):
        for th in best_th + np.deg2rad(np.arange(-1.0, 1.0001, step)):
            R = rot(th)
            s = _sharpness(R[:, 0], c['bin_m']) + _sharpness(R[:, 1], c['bin_m'])
            if s > best_s:
                best_th, best_s = th, s

    # 4. which rotated axis is the long one: try both, keep the consistent one
    R = rot(best_th)
    cands = []
    for swap in (False, True):
        Lu, Lv = (T.WALL_LX, T.WALL_LY) if not swap else (T.WALL_LY, T.WALL_LX)
        pu = _wall_pair(R[:, 0], Lu, c)
        pv = _wall_pair(R[:, 1], Lv, c)
        if pu is None or pv is None:
            continue
        cands.append((pu[2] + pv[2], swap, pu, pv))
    if not cands:
        return dict(ok=False, why='no wall pair matches %.2f x %.2f m' % (T.WALL_LX, T.WALL_LY))
    cands.sort(key=lambda k: -k[0])
    _, swap, pu, pv = cands[0]

    # 5. refine yaw + four offsets on inliers (Gauss-Newton, small angle)
    th = best_th
    off = np.array([pu[0], pu[1], pv[0], pv[1]], float)
    for _ in range(5):
        R = rot(th)
        rows, res, cnt = [], [], [0, 0, 0, 0]
        for k, (ax, o) in enumerate(zip((0, 0, 1, 1), off)):
            sel = np.abs(R[:, ax] - o) < c['inlier_m']
            cnt[k] = int(sel.sum())
            if not sel.any():
                continue
            # d(u)/d(th) = v, d(v)/d(th) = -u  (rotating the points by -th)
            dth = R[sel, 1] if ax == 0 else -R[sel, 0]
            J = np.zeros((sel.sum(), 5))
            J[:, 0] = dth
            J[:, 1 + k] = -1.0
            rows.append(J)
            res.append(R[sel, ax] - o)
        J = np.vstack(rows)
        e = np.concatenate(res)
        dx = np.linalg.lstsq(J, -e, rcond=None)[0]
        th += dx[0]
        off += dx[1:]
        if abs(dx[0]) < 1e-5 and np.abs(dx[1:]).max() < 1e-4:
            break
    rms = float(np.sqrt(np.mean(e ** 2)))
    seen = [k >= c['min_wall_pts'] for k in cnt]
    du, dv = off[1] - off[0], off[3] - off[2]
    Lu, Lv = (T.WALL_LX, T.WALL_LY) if not swap else (T.WALL_LY, T.WALL_LX)
    out = dict(walls_seen=int(sum(seen)), n_in=cnt, rms=rms, dims=(float(du), float(dv)),
               floor_h=float(d))
    if sum(seen) < 3:
        out.update(ok=False, why='walls seen %d < 3 (inliers %s)' % (sum(seen), cnt))
        return out
    if (seen[0] and seen[1] and abs(du - Lu) > c['dim_tol']) or (seen[2] and seen[3] and abs(dv - Lv) > c['dim_tol']):
        out.update(ok=False, why='wall spacing %.2f x %.2f vs %.2f x %.2f' % (du, dv, Lu, Lv))
        return out
    if rms > c['max_rms']:
        out.update(ok=False, why='inlier rms %.3f m' % rms)
        return out

    # 6. robot pose. In the rotated frame (points turned by -th) the wall centre is at (cu, cv); a wall seen
    # on one side only is placed by the nominal size, which the pair finder already did.
    cu, cv = 0.5 * (off[0] + off[1]), 0.5 * (off[2] + off[3])
    # rotated frame -> track frame: if swap, rotated u is the track's y axis
    if not swap:
        x, y, yaw = -cu, -cv, -th
    else:
        x, y, yaw = cv, -cu, -th + math.pi / 2     # track x = -v, track y = u
    # The 180 deg twin (-x, -y, yaw + pi) fits the walls and the track equally well. Pick by the prior,
    # else the lower half. Both twins head the same way along the track (counter-clockwise or not).
    if prior is not None:
        if math.hypot(x - prior[0], y - prior[1]) > math.hypot(-x - prior[0], -y - prior[1]):
            x, y, yaw = -x, -y, yaw + math.pi
    elif y > 0:
        x, y, yaw = -x, -y, yaw + math.pi
    lane = 1 if abs(T.lane_frame(1, x, y)[0]) < abs(T.lane_frame(2, x, y)[0]) else 2
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    off_lat, tang = T.lane_frame(lane, x, y)
    out.update(ok=True, why='', x=x, y=y, yaw=yaw, lane=lane, lateral=off_lat,
               heading_err=math.atan2(math.sin(yaw - tang), math.cos(yaw - tang)))
    return out
