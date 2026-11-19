"""Cone avoidance on a taught route — the logic only (numpy/scipy, no ROS), used by cone_runner_node.py.

The taught route stays the plan as long as nothing changed: inside a cone zone (a box in the track frame, from
<route>.cones.json) the obstacle cells the LiDAR scan shows are compared with the map of the course (where the
cones stood when the route was taught). Cells the map does not have = moved / new cones. The taught route is
kept while no new cell is nearer than `clear` to it and no cell at all nearer than `keep_min`. Otherwise the route
ahead is shifted sideways:

  candidates   Q[i, k] = P[i] + off_k * N[i]       P = taught route every ds, N = normal of the route smoothed
                                                    over ~1 m (its general direction, not the weave), off_k = k * dk
  hard         clearance >= clear to every cell (except the first 0.4 m: the robot is where it is),
               inside the zone box at zone stations, at most `slope` m of sideways shift per m forward
  cost         off^2 (closest to the taught route; much dearer outside the zone, so it rejoins there)
               + (shift per m)^2 (smooth) + nearness inside `soft` + (off - last plan)^2 (no flip-flop)

solved exactly by dynamic programming over the stations. No path at `clear` -> retry with `min_clear`; still none
-> 'blocked' (the caller stops, and after a while may force the least-bad path).

Distances are to scan cells, i.e. the cone's surface at the LiDAR ring heights (0.2-0.6 m), a few cm inside the
cone's base. The 11-19 cone field is tight: the taught route passes 0.16 m from the mapped bases (~0.2 m from
what the scan sees) at its narrowest, so the defaults are clear 0.20 / keep_min 0.15, not a comfortable 0.3.
"""
import json
import math
import os

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree


def in_boxes(xy, boxes, margin=0.0):
    """(N,) bool: point inside any box [x0, y0, x1, y1] grown by margin."""
    xy = np.atleast_2d(np.asarray(xy, float))
    m = np.zeros(len(xy), bool)
    for b in boxes:
        m |= ((xy[:, 0] > b[0] - margin) & (xy[:, 0] < b[2] + margin)
              & (xy[:, 1] > b[1] - margin) & (xy[:, 1] < b[3] + margin))
    return m


def zones_file(path_file, fallback_dir=None):
    """<route>.cones.json next to the route, else in fallback_dir (the repo's routes/). None if neither."""
    name = os.path.splitext(os.path.basename(path_file))[0] + '.cones.json'
    for d in (os.path.dirname(path_file), fallback_dir):
        if d and os.path.exists(os.path.join(d, name)):
            return os.path.join(d, name)
    return None


def load_zones(path_file, fallback_dir=None):
    f = zones_file(path_file, fallback_dir) if path_file else None
    if f is None:
        return [], None
    zones = [z for z in json.load(open(f)) if 'box' in z]
    for z in zones:
        b = z['box']
        z['box'] = [min(b[0], b[2]), min(b[1], b[3]), max(b[0], b[2]), max(b[1], b[3])]
    return zones, f


class ObstacleMemory:
    """Obstacle cells (track frame) from the last `memory_s` seconds of scans. A cell counts when at least
    `min_hits` different scans hit it (one stray return is not a cone). Cells are kept while they leave the view
    (the scan does not see closer than ~0.4 m, i.e. a cone right beside the robot)."""

    def __init__(self, memory_s=6.0, res=0.05, min_hits=2):
        self.memory_s, self.res, self.min_hits = memory_s, res, min_hits
        self.chunks = []        # (time, unique cell indices of one scan)

    def add(self, t, xy):
        xy = np.asarray(xy, float).reshape(-1, 2)
        if len(xy):
            self.chunks.append((t, np.unique(np.floor(xy / self.res).astype(np.int64), axis=0)))
        self.chunks = [c for c in self.chunks if t - c[0] <= self.memory_s]

    def clear(self):
        self.chunks = []

    def cells(self, t=None):
        ch = [c for c in self.chunks if t is None or t - c[0] <= self.memory_s]
        if not ch:
            return np.zeros((0, 2))
        u, n = np.unique(np.vstack([c[1] for c in ch]), axis=0, return_counts=True)
        return (u[n >= self.min_hits] + 0.5) * self.res


def axis_normals(P, ds, sigma_m=0.5):
    """Left unit normals of the route smoothed with a Gaussian of sigma_m (m along the route)."""
    sg = max(0.5, sigma_m / ds)
    A = np.column_stack([gaussian_filter1d(P[:, 0], sg, mode='nearest'), gaussian_filter1d(P[:, 1], sg, mode='nearest')])
    T = np.gradient(A, axis=0)
    T /= np.maximum(1e-9, np.hypot(T[:, 0], T[:, 1]))[:, None]
    return np.column_stack([-T[:, 1], T[:, 0]])


def dp(node, k0, step, step_cost):
    """Min-cost path through node[i, k] (inf = forbidden), start fixed at k0, |k_i - k_(i-1)| <= step,
    step_cost[d] for a change of d (index d + step). Returns (k per station, total cost) or (None, inf)."""
    S, K = node.shape
    C = np.full(K, np.inf)
    C[k0] = node[0, k0]
    back = np.zeros((S, K), np.int32)
    idx = np.arange(K)
    for i in range(1, S):
        best = np.full(K, np.inf)
        arg = np.zeros(K, np.int32)
        for d in range(-step, step + 1):        # predecessor j = k - d
            sh = np.full(K, np.inf)
            if d >= 0:
                sh[d:] = C[:K - d]
            else:
                sh[:K + d] = C[-d:]
            cand = sh + step_cost[d + step]
            b = cand < best
            best[b] = cand[b]
            arg[b] = (idx - d)[b]
        C = best + node[i]
        back[i] = arg
        if not np.isfinite(C).any():
            return None, math.inf
    k = np.empty(S, np.int32)
    k[-1] = int(np.argmin(C))
    for i in range(S - 1, 0, -1):
        k[i - 1] = back[i, k[i]]
    return k, float(C[k[-1]])


class ConePlanner:
    def __init__(self, clear=0.20, soft=0.40, min_clear=0.17, keep_min=0.15, dmax=0.9, dk=0.04, ds=0.10,
                 slope=0.8, free_m=0.4, w_dev=1.0, w_out=40.0, w_near=4.0, w_step=0.3, w_hold=2.0, sigma_m=0.5,
                 rejoin_tol=0.12):
        self.clear, self.soft, self.min_clear = clear, max(soft, clear + 0.05), min(min_clear, clear)
        self.keep_min = keep_min
        self.dk, self.ds, self.sigma_m, self.rejoin_tol = dk, ds, sigma_m, rejoin_tol
        self.kmax = int(round(dmax / dk))
        self.off = (np.arange(2 * self.kmax + 1) - self.kmax) * dk
        self.step = max(1, int(math.floor(slope * ds / dk + 1e-9)))
        self.step_cost = np.array([w_step * (d * dk / ds) ** 2 for d in range(-self.step, self.step + 1)])
        self.n_free = int(math.ceil(free_m / ds))
        self.w = dict(dev=w_dev, out=w_out, near=w_near, hold=w_hold)

    def clearance(self, tree, Q):
        if tree is None:
            return np.full(len(Q), np.inf)
        return tree.query(Q, distance_upper_bound=self.soft + 0.5)[0]

    def plan(self, P, N, inside, robot_xy, obs, boxes, new=None, stations=None, prev=None, force=False):
        """P (S,2) taught points ahead (P[0] = the robot's station), N (S,2) normals (axis_normals), inside (S,)
        station in a cone zone, obs (M,2) obstacle cells, new (M,) bool cell not in the map (None = all new),
        prev = (stations, offsets) of the last plan.
        -> dict(mode 'taught'|'avoid'|'blocked'|'forced', pts (S,2), off (S,), min_clear, used)"""
        S = len(P)
        tree = cKDTree(obs) if len(obs) else None
        enforce = in_boxes(P, boxes, margin=0.3)          # clearance is checked in (and just around) the zones
        enforce[:self.n_free] = False
        e = float(np.dot(np.asarray(robot_xy[:2], float) - P[0], N[0]))      # robot's offset at its station
        d0 = self.clearance(tree, P)
        mc = float(d0[enforce].min()) if enforce.any() else math.inf
        if new is None:
            mn = mc
        else:
            nn = np.asarray(obs)[np.asarray(new, bool)] if len(obs) else np.zeros((0, 2))
            dn = self.clearance(cKDTree(nn) if len(nn) else None, P)
            mn = float(dn[enforce].min()) if enforce.any() else math.inf
        if not force and abs(e) < self.rejoin_tol and mn >= self.clear and mc >= self.keep_min:
            return dict(mode='taught', pts=P, off=np.zeros(S), min_clear=mc, used=self.clear)
        K = len(self.off)
        Q = P[:, None, :] + self.off[None, :, None] * N[:, None, :]           # (S, K, 2)
        dist = self.clearance(tree, Q.reshape(-1, 2)).reshape(S, K)
        o2 = self.off[None, :] ** 2
        base = np.where(inside[:, None], self.w['dev'], self.w['out']) * o2
        near = np.where(dist < self.soft, ((self.soft - dist) / (self.soft - self.min_clear)) ** 2, 0.0)
        base = base + self.w['near'] * near
        if prev is not None and stations is not None and len(prev[0]) > 1:
            po = np.interp(stations, prev[0], prev[1], left=np.nan, right=np.nan)
            has = np.isfinite(po)
            base[has] += self.w['hold'] * (self.off[None, :] - po[has, None]) ** 2
        outside_box = ~in_boxes(Q.reshape(-1, 2), boxes).reshape(S, K)
        outside_box[:self.n_free] = False                # the robot may stand a little outside the box
        base[inside[:, None] & outside_box] = np.inf
        k0 = int(np.clip(round(e / self.dk) + self.kmax, 0, K - 1))
        base[0, :] = np.inf
        base[0, k0] = 0.0
        levels = [None] if force else sorted({self.clear, self.min_clear}, reverse=True)
        for hard in levels:
            node = base.copy()
            if hard is not None:
                node[enforce[:, None] & (dist < hard)] = np.inf
            k, cost = dp(node, k0, self.step, self.step_cost)
            if k is None:
                continue
            off = self.off[k]
            pts = P + off[:, None] * N
            # light smoothing (0.5 m), kept only if it does not get closer than allowed
            sm = np.convolve(np.pad(off, 2, mode='edge'), np.ones(5) / 5.0, mode='valid')
            sm[0] = off[0]
            ps = P + sm[:, None] * N
            dsm = self.clearance(tree, ps)
            lim = hard if hard is not None else 0.0
            if (not enforce.any()) or dsm[enforce].min() >= lim - 0.01:
                off, pts = sm, ps
            dd = self.clearance(tree, pts)
            return dict(mode='forced' if force else 'avoid', pts=pts, off=off,
                        min_clear=float(dd[enforce].min()) if enforce.any() else math.inf,
                        used=hard if hard is not None else 0.0)
        return dict(mode='blocked', pts=P, off=np.zeros(S), min_clear=mc, used=self.min_clear)
