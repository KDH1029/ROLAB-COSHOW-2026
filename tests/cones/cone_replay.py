import sys, os, math, time, json, numpy as np
ROOT = os.path.expanduser('~/ws/ROLAB-COSHOW-2026')
sys.path.insert(0, ROOT + '/src/mission/track_day1'); sys.path.insert(0, ROOT + '/scripts')
import cone_avoid as CA, track_path as TP, cones_view as V
from scipy.spatial import cKDTree
from scipy.ndimage import label

Z = np.load('scan105326.npz')
L = np.genfromtxt(ROOT + '/maps/coshow_1119/routes/rec105326/raw.csv', delimiter=',', names=True)
lio = np.genfromtxt(ROOT + '/logs/route_105326/lio.csv', delimiter=',', names=True)
t0 = lio['t'][0]
Lt = L['t_s'] + t0; Lyaw = np.unwrap(np.radians(L['yaw_deg']))
box = json.load(open(ROOT + '/routes/rec105326_lap1.cones.json'))[0]['box']
M = V.map_cells(ROOT + '/maps/coshow_1119')
Mz = M[CA.in_boxes(M, [box], 0.8)]
mtree = cKDTree(Mz)

# mapped cones = clusters of map cells inside the box (not walls)
Mi = M[CA.in_boxes(M, [box], 0.0)]
res = 0.05
ij = np.floor((Mi - [box[0], box[1]]) / res).astype(int)
G = np.zeros(ij.max(0) + 3, bool); G[ij[:, 0], ij[:, 1]] = True
lab, n = label(G, structure=np.ones((3, 3)))
cones = []
for k in range(1, n + 1):
    c = Mi[lab[ij[:, 0], ij[:, 1]] == k]
    if 5 <= len(c) <= 80:
        cones.append(c.mean(0))
cones = np.array(cones)
print('mapped cones in the zone: %d  %s' % (len(cones), np.round(cones, 2).tolist()))

# scans -> track frame
use_stamp = Z['stamp']
a = Z['amin'] + np.arange(Z['R'].shape[1]) * Z['inc']
mem = CA.ObstacleMemory(memory_s=6.0)
allpts, seen_first = [], {}
robot_track = []
first_t, last_t = Lt[0], Lt[-1]
lp = TP.load_pins(ROOT + '/logs/day1/routes/rec105326_lap1.csv', 2)
planner = CA.ConePlanner()
prog_s = None
modes = {}
events = []
tplan = []
rows = []
inj = None
if len(sys.argv) > 1 and sys.argv[1] == 'inject':
    inj = np.array([float(v) for v in sys.argv[2].split(',')])
def plan_at(tp, s_r, cells, new):
    ss = np.arange(s_r - 1.5, s_r + 6.0 + 1.5 + 1e-9, 0.1)
    P = np.array([lp.pose_at(s)[:2] for s in ss])
    N = CA.axis_normals(P, 0.1, planner.sigma_m)
    w = (ss >= s_r - 1e-9) & (ss <= s_r + 6.0 + 1e-9)
    ss, P, N = ss[w], P[w], N[w]
    inside = CA.in_boxes(P, [box])
    if not inside.any() and not CA.in_boxes(np.array(tp[:2]), [box], 0.3)[0]:
        return None, ss
    return planner.plan(P, N, inside, tp[:2], cells, [box], new=new, stations=ss, prev=PREV[0]), ss

last_plan_t = -1
PREV = [None]
PLANS = []
for k in range(len(use_stamp)):
    t = use_stamp[k]
    if t < first_t or t > last_t:
        continue
    x = np.interp(t, Lt, L['x_m']); y = np.interp(t, Lt, L['y_m']); yaw = np.interp(t, Lt, Lyaw)
    r = Z['R'][k].astype(float)
    ok = np.isfinite(r) & (r > max(Z['rmin'], 0.05)) & (r < 3.5)
    p = np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
    P = TP.apply((x, y, yaw), p)
    rng = r[ok]
    if inj is not None:     # a synthetic cone: scan returns on a circle r 0.10 around inj where the beam would hit it
        d = inj - [x, y]; dist = math.hypot(*d)
        if 0.45 < dist < 3.5:
            ang = math.atan2(d[1], d[0]) - yaw
            half = math.asin(min(1.0, 0.10 / dist))
            aa = np.arange(ang - half, ang + half, Z['inc'])
            rr = dist - 0.10 * np.cos(np.arcsin(np.clip(dist * np.sin(aa - ang) / 0.10, -1, 1)))
            pp = TP.apply((x, y, yaw), np.column_stack([rr * np.cos(aa), rr * np.sin(aa)]))
            P = np.vstack([P, pp]); rng = np.concatenate([rng, rr])
    keep = CA.in_boxes(P, [box], 0.5)
    mem.add(t, P[keep])
    allpts.append(np.column_stack([P[keep], rng[keep], np.full(keep.sum(), t)]))
    robot_track.append((t, x, y, yaw))
    # planner every 0.1 s like the runner
    if t - last_plan_t >= 0.1:
        last_plan_t = t
        s0 = lp.station(x, y)
        if prog_s is not None:
            kk = round((prog_s - s0) / lp.length); s0 += kk * lp.length
            if abs(s0 - prog_s) > 1.0: s0 = prog_s
        prog_s = s0
        cells = mem.cells(t)
        new = np.ones(len(cells), bool) if not len(cells) else mtree.query(cells, distance_upper_bound=0.13)[0] > 0.12
        c0 = time.perf_counter()
        res, ss = plan_at((x, y, yaw), s0, cells, new)
        tplan.append(time.perf_counter() - c0)
        if res is None:
            continue
        e = float(np.dot(np.array([x, y]) - np.array(lp.pose_at(s0)[:2]), CA.axis_normals(np.array([lp.pose_at(s0 + d)[:2] for d in np.arange(-1, 1.01, 0.1)]), 0.1)[10]))
        PREV[0] = (ss, res['off']) if res['mode'] in ('avoid', 'forced') else None
        PLANS.append((t - first_t, x, y, yaw, res['mode'], res['pts'], cells, new, res['min_clear'], float(np.abs(res['off']).max())))
        modes[res['mode']] = modes.get(res['mode'], 0) + 1
        rows.append(dict(t=t - first_t, x=x, y=y, mode=res['mode'], e=e, mc=res['min_clear'], new=int(new.sum()),
                         cells=len(cells), shift=float(np.abs(res['off']).max())))
        if res['mode'] != 'taught':
            events.append(rows[-1])
        if inj is not None and res['mode'] != 'taught' and rows[-1]['shift'] > 0:
            pass
A = np.vstack(allpts)
Rt = np.array(robot_track)
print('scans used %d over %.0f s; zone cells %d' % (len(Rt), Rt[-1, 0] - Rt[0, 0], len(A)))
# 1. scan cells vs map
d_map = mtree.query(A[:, :2])[0]
print('scan returns in zone+0.5 -> distance to the nearest map cell: p50 %.3f p90 %.3f p95 %.3f p99 %.3f max %.3f; > 0.08 m: %.2f %%'
      % tuple(list(np.percentile(d_map, [50, 90, 95, 99])) + [d_map.max(), 100 * (d_map > 0.08).mean()]))
cells_all = np.unique(np.floor(A[:, :2] / 0.05).astype(int), axis=0)
# per-cell hit count (scans) over whole drive and distance to map
ijA = np.floor(A[:, :2] / 0.05).astype(int)
uc, inv = np.unique(ijA, axis=0, return_inverse=True)
nsc = np.array([len(np.unique(A[inv.ravel() == i, 3])) for i in range(len(uc))]) if len(uc) < 4000 else None
cc = (uc + 0.5) * 0.05
dc = mtree.query(cc)[0]
if nsc is not None:
    solid = nsc >= 2
    print('cells hit by >=2 scans: %d; of them farther than 0.08 m from the map: %d (%.1f %%), farther than 0.12: %d'
          % (solid.sum(), (dc[solid] > 0.08).sum(), 100 * (dc[solid] > 0.08).mean(), (dc[solid] > 0.12).sum()))
    far = cc[solid & (dc > 0.08)]
    if len(far):
        print('  where (x, y, dist, scans):', [(round(p[0], 2), round(p[1], 2), round(q, 3), int(h)) for p, q, h in zip(far, dc[solid & (dc > 0.08)], nsc[solid & (dc > 0.08)])][:25])
# 2. detection range per mapped cone
print('cones: first seen at robot distance (m) / max range any return / scans hitting:')
for c in cones:
    near = np.hypot(A[:, 0] - c[0], A[:, 1] - c[1]) < 0.2
    if not near.any():
        print('  cone (%.2f, %.2f): never seen' % tuple(c)); continue
    tt = A[near, 3]
    tf = tt.min()
    rx = np.interp(tf, Rt[:, 0], Rt[:, 1]); ry = np.interp(tf, Rt[:, 0], Rt[:, 2])
    print('  cone (%.2f, %.2f): first at %.2f m (t %.1f s), returns up to %.2f m, %d scans'
          % (c[0], c[1], math.hypot(c[0] - rx, c[1] - ry), tf - first_t, A[near, 2].max(), len(np.unique(tt))))
# 3. taught route clearance to scan cells (all solid cells over the drive)
S = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length, 0.02)])
inb = CA.in_boxes(S, [box], 0.3)
solid_cells = cc[solid] if nsc is not None else cc
d_route = cKDTree(solid_cells).query(S[inb])[0]
d_route_map = cKDTree(cones_cells := Mi).query(S[inb])[0]
j = np.argmin(d_route)
print('taught lap1 in the zone: clearance to scan cells min %.3f at (%.2f, %.2f) | p5 %.3f | to map cells min %.3f'
      % (d_route.min(), S[inb][j, 0], S[inb][j, 1], np.percentile(d_route, 5), d_route_map.min()))
# robot's actual drive
inr = CA.in_boxes(Rt[:, 1:3], [box], 0.0)
d_drive = cKDTree(solid_cells).query(Rt[inr, 1:3])[0]
print('robot as driven in the zone: clearance to scan cells min %.3f p5 %.3f' % (d_drive.min(), np.percentile(d_drive, 5)))
# 4. planner along the replay
print('planner along the drive (every 0.1 s near the zone): %s; compute median %.1f ms, max %.1f ms'
      % (modes, 1000 * np.median(tplan), 1000 * np.max(tplan)))
for ev in []:
    print('  %.1f s at (%.2f, %.2f): %s, robot off taught %.3f m, min clear %.3f, new cells %d/%d, shift %.2f'
          % (ev['t'], ev['x'], ev['y'], ev['mode'], ev['e'], ev['mc'], ev['new'], ev['cells'], ev['shift']))
if len(events) > 12: print('  ... %d more' % (len(events) - 12))
import pickle; pickle.dump(PLANS, open('plans_%s.pkl' % ('inject' if inj is not None else 'base'), 'wb'))
# event clusters
cl = []
for ev in events:
    if cl and ev['t'] - cl[-1][1] < 0.6: cl[-1][1] = ev['t']; cl[-1][2].append(ev)
    else: cl.append([ev['t'], ev['t'], [ev]])
for a0, a1, evs in cl:
    print('  non-taught %.1f-%.1f s (%d): %s, x %.2f..%.2f, max shift %.2f, min clear %.3f, new %d'
          % (a0, a1, len(evs), sorted({q['mode'] for q in evs}), min(q['x'] for q in evs), max(q['x'] for q in evs),
             max(q['shift'] for q in evs), min(q['mc'] for q in evs), max(q['new'] for q in evs)))
np.savez('replay_out_%s.npz' % ('inject' if inj is not None else 'base'), A=A, Rt=Rt, rows=json.dumps(rows), cones=cones)
