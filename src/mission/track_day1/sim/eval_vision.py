#!/usr/bin/env python3
"""Vision-only check over random poses and nuisance conditions (truth = rulebook stadium).

Per frame: plane error, edge points near a true boundary (precision / error), points that are wrong
but inside the tracker gate (the dangerous ones), own-lane boundary coverage, time.
"""
import math, os, sys, time
import numpy as np, cv2
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..')); sys.path.insert(0, HERE)
import band_vision_core as BV, floor_world as W, track_spec as T
from sim_day1 import pose_at_s

RB = np.array([T.R_INFIELD, T.R_LANE12, T.R_OUTER])
BAND = {1: (T.R_LANE12, T.R_OUTER), 2: (T.R_INFIELD, T.R_LANE12)}

def highlights(bgr, rng, n=3):
    h, w = bgr.shape[:2]
    out = bgr.astype(np.float32)
    for _ in range(n):
        m = np.zeros((h, w), np.float32)
        cv2.ellipse(m, (int(rng.uniform(0, w)), int(rng.uniform(h * 0.3, h))), (int(rng.uniform(20, 80)), int(rng.uniform(8, 30))),
                    rng.uniform(0, 180), 0, 360, 1.0, -1)
        m = cv2.GaussianBlur(m, (0, 0), 8)
        out = out * (1 - 0.8 * m[..., None]) + 255 * 0.8 * m[..., None]
    return np.clip(out, 0, 255).astype(np.uint8)

COND = {
    'base':       dict(),
    'flat_cam':   dict(pitch=0.0),
    'pitch30':    dict(pitch=30.0),
    'dim_tint':   dict(gain=0.55, tint=(1.0, 0.88, 0.75)),
    'bright':     dict(gain=1.35),
    'glare':      dict(glare=True),
    'blur':       dict(blur=7),
    'robot':      dict(other=True),
    'seams':      dict(seams=True),
}

def run(cond, n=120, seed=0, pitch=20.0, gain=1.0, tint=(1, 1, 1), glare=False, blur=0, other=False, seams=False):
    rng = np.random.default_rng(seed)
    W.SEAMS[:] = [('x', list(np.arange(-4.5, 4.6, 1.22))), ('y', list(np.arange(-3.5, 3.6, 2.44)))] if seams else []
    rig = W.Rig(pitch_deg=pitch)
    vis = BV.BandVision(seed=seed)
    st = dict(fail=0, hp=[], pp=[], err=[], conf=[], own_ok=0, ms=[], frames=0, nall=0)
    for i in range(n):
        lane = 1 + (i % 2)
        R = T.R_CENTER[lane]
        x, y, yaw = pose_at_s(rng.uniform(0, T.LAP_LEN[lane]), R, rng.uniform(-0.2, 0.2))
        yaw += math.radians(rng.uniform(-10, 10))
        bp, br = math.radians(rng.uniform(-2, 2)), math.radians(rng.uniform(-2, 2))
        boxes = []
        if other:
            lo = 2 if lane == 1 else 1
            bx, by, byaw = pose_at_s(rng.uniform(0, T.LAP_LEN[lo]), T.R_CENTER[lo])
            # put it 1-2.5 m ahead in the other lane
            s_me = rng.uniform(1.0, 2.5)
            bx, by, byaw = x + s_me * math.cos(yaw) + (0.9 if lane == 2 else -0.9) * -math.sin(yaw) * (1 if lane == 1 else -1), \
                y + s_me * math.sin(yaw) + (0.9 if lane == 2 else -0.9) * math.cos(yaw) * (1 if lane == 1 else -1), yaw
            boxes = [(bx, by, byaw, 0.7, 0.31, 0.42)]
        bgr, dep = rig.render_pair(x, y, yaw, boxes=boxes, rng=rng, gain=gain, tint=tint, body_pitch=bp, body_roll=br)
        if glare:
            bgr = highlights(bgr, rng)
        if blur:
            k = np.zeros((blur, blur), np.float32); k[:, blur // 2] = 1.0 / blur
            bgr = cv2.filter2D(bgr, -1, k)
        t0 = time.time()
        out = vis.process(bgr, dep, rig.color.K, rig.depth.K)
        st['ms'].append((time.time() - t0) * 1e3)
        st['frames'] += 1
        if not out['ok']:
            st['fail'] += 1
            continue
        st['hp'].append(out['h'] - 0.43)
        p = out['pts']
        wx = x + math.cos(yaw) * p[:, 0] - math.sin(yaw) * p[:, 1]
        wy = y + math.sin(yaw) * p[:, 0] + math.cos(yaw) * p[:, 1]
        r = W.stadium_r(wx, wy)
        d = np.abs(r[:, None] - RB[None, :])
        k = d.argmin(1); db = d.min(1)
        good = db < 0.04
        st['nall'] += len(p)
        st['err'] += list(db[good])
        st['pp'].append(good.mean())
        st['conf'].append(((db >= 0.06) & (db < 0.28)).mean())   # false features inside the tracker gate
        lo_, hi_ = BAND[lane]
        own = ((np.abs(r - lo_) < 0.04).sum() >= 15) or ((np.abs(r - hi_) < 0.04).sum() >= 15)
        st['own_ok'] += int(own)
    e = np.array(st['err']) * 100
    return dict(cond=cond, ok=1 - st['fail'] / st['frames'], prec=np.mean(st['pp']) if st['pp'] else 0,
                conf=np.mean(st['conf']) if st['conf'] else 0, conf95=np.percentile(st['conf'], 95) if st['conf'] else 0,
                e50=np.median(e) if len(e) else 0, e95=np.percentile(e, 95) if len(e) else 0,
                own=st['own_ok'] / st['frames'], h=np.percentile(np.abs(st['hp']), 95) * 100 if st['hp'] else 0,
                ms=np.median(st['ms']), pts=st['nall'] / max(1, st['frames'] - st['fail']))

if __name__ == '__main__':
    names = sys.argv[1].split(',') if len(sys.argv) > 1 else list(COND)
    print('%-9s %6s %6s %7s %8s %6s %6s %7s %6s %5s %5s' % ('cond', 'plane', 'prec', 'false', 'false p95', 'e50cm', 'e95cm', 'ownlane', 'h95cm', 'ms', 'pts'))
    for c in names:
        r = run(c, **COND[c])
        print('%-9s %5.0f%% %5.0f%% %6.1f%% %7.1f%% %6.1f %6.1f %6.0f%% %6.1f %5.1f %5.0f' % (
            r['cond'], 100 * r['ok'], 100 * r['prec'], 100 * r['conf'], 100 * r['conf95'], r['e50'], r['e95'], 100 * r['own'], r['h'], r['ms'], r['pts']), flush=True)
