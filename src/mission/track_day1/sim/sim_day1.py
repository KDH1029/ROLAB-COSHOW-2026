#!/usr/bin/env python3
"""Closed-loop day-1 sim: rendered D435 frames -> band_vision_core -> lane_tracker -> day1_logic -> Go2 model.

Same code as the robot, only the camera / odometry / body are simulated. Truth is the stadium
geometry of the rulebook; the runner never sees it.

  python3 sim/sim_day1.py                  # all scenarios, table
  python3 sim/sim_day1.py --only lane2 --save out_dir   # one scenario, frames for checking
"""
import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
import band_vision_core as BV  # noqa: E402
import day1_logic as DL  # noqa: E402
import floor_world as W  # noqa: E402
import lane_path as LPm  # noqa: E402
import track_spec as T  # noqa: E402

FEET = [(0.19, 0.14), (0.19, -0.14), (-0.19, 0.14), (-0.19, -0.14)]
FOOT_R = 0.02
DEBUG = os.environ.get('SIM_DEBUG') == '1'
BAND = {1: (T.R_LANE12, T.R_OUTER), 2: (T.R_INFIELD, T.R_LANE12)}


def track_s(x, y, R):
    """Arc length along a lane centre (radius R), 0 at the bottom straight x = -1, CCW."""
    L = T.STRAIGHT / 2
    if abs(x) <= L:
        return (x + L) if y < 0 else (2 * L + math.pi * R + (L - x))
    if x > L:
        phi = math.atan2(y, x - L)          # -pi/2 .. pi/2
        return 2 * L + R * (phi + math.pi / 2)
    phi = math.atan2(y, x + L)              # pi/2 .. pi or -pi .. -pi/2
    if phi < 0:
        phi += 2 * math.pi
    return 4 * L + math.pi * R + R * (phi - math.pi / 2)


def pose_at_s(s, R, off=0.0):
    """Centre-line pose (x, y, yaw) at arc length s (+ off to the left)."""
    L = T.STRAIGHT / 2
    lap = 4 * L + 2 * math.pi * R
    s = s % lap
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
    return x - off * math.sin(yaw), y + off * math.cos(yaw), yaw


SCEN = {
    'lane2':          dict(lane=2),
    'lane1':          dict(lane=1),
    'lane2_place':    dict(lane=2, off0=0.15, yaw0=5.0),
    'lane1_place':    dict(lane=1, off0=-0.15, yaw0=-5.0, odom_true=0.80, drift=0.03),
    'lane2_flatcam':  dict(lane=2, pitch=0.0),
    'lane1_cam30':    dict(lane=1, pitch=30.0),
    'lane2_drops':    dict(lane=2, drop=0.25, latency=0.20),
    'lane1_robot':    dict(lane=1, other=True),
    'lane2_fast':     dict(lane=2, v_max=0.65),
    'lane1_dim':      dict(lane=1, gain=0.6, tint=(1.0, 0.9, 0.8)),
    'lane2_glare':    dict(lane=2, glare=True),
    'lane1_glare':    dict(lane=1, glare=True),
    'lane2_curvestart': dict(lane=2, s0=3.4),
    'lane2_place_out':  dict(lane=2, off0=-0.15, yaw0=-8.0),
    'lane1_place_in':   dict(lane=1, off0=0.15, yaw0=8.0),
    'lane2_curve_place': dict(lane=2, s0=3.4, off0=0.12, yaw0=6.0),
}


def run(name, lane, pitch=20.0, off0=0.0, yaw0=0.0, s0=0.4, odom_true=1 / 1.189, drift=0.012, drop=0.0,
        latency=0.12, other=False, v_max=0.5, gain=1.0, tint=(1.0, 1.0, 1.0), seed=0, ds=2, save=None,
        t_green=1.5, t_max=75.0, glare=False):
    rng = np.random.default_rng(seed)
    R = T.R_CENTER[lane]
    lap = T.LAP_LEN[lane]
    rig = W.Rig(pitch_deg=pitch, ds=ds)
    vision = BV.BandVision(seed=seed)
    path = LPm.LanePath(lane)
    ctrl = DL.RppLike(v=v_max)
    gate = DL.LightGate()
    lapend = DL.LapEnd(lane)
    sc = 1.189                                         # runner-side odometry scale (in the TF on the robot)
    s_line = s0
    x, y, yaw = pose_at_s(s_line - 0.30, R, off0)
    yaw += math.radians(yaw0)
    on_straight = (s_line % lap) < T.STRAIGHT
    start_lines = [(lane, pose_at_s(s_line, R)[0], -1 if pose_at_s(s_line, R)[1] < 0 else 1)] if on_straight else []
    vx_a = wz_a = 0.0
    ox = oy = 0.0
    oyaw = 0.3                                         # odometry frame has its own origin / heading
    path.init_at(0.0, 0.0, oyaw, kappa=0.0 if on_straight else T.CURVATURE[lane])
    pend = []
    dt = 0.02
    t = 0.0
    next_cam = 0.0
    prog = 0.0
    s_prev = track_s(x, y, R)
    need = lap + 0.30 + 0.21                           # centre travel until the rear feet cross the line
    t_go = t_fin = None
    min_margin = 1e9
    max_off = 0.0
    viol = 0
    in_viol = False
    n_frames = n_ok = 0
    other_s = s_line + 1.0
    lane_o = 2 if lane == 1 else 1
    vis_ms = []
    st = 'WAIT'
    finished = False
    n_init = 0
    init_pts = []
    last_ok = None
    while t < t_max:
        if t >= next_cam - 1e-9:
            next_cam += 0.1
            boxes = []
            if other:
                bx, by, byaw = pose_at_s(other_s, T.R_CENTER[lane_o])
                boxes = [(bx, by, byaw, 0.70, 0.31, 0.42)]
            ph = 2 * math.pi * 2.5 * t
            bgr, dep = rig.render_pair(x, y, yaw, boxes=boxes, rng=rng, start_lines=start_lines, gain=gain,
                                       tint=tint, body_pitch=math.radians(1.5) * math.sin(ph),
                                       body_roll=math.radians(1.5) * math.cos(ph), dz=0.01 * math.sin(2 * ph))
            if glare:
                import eval_vision
                bgr = eval_vision.highlights(bgr, rng)
            if rng.random() >= drop:
                t0 = time.time()
                out = vision.process(bgr, dep, rig.color.K, rig.depth.K)
                vis_ms.append((time.time() - t0) * 1e3)
                n_frames += 1
                if out['ok']:
                    pend.append((t + latency, (ox * sc, oy * sc, oyaw), out['pts'].copy()))
                if save and n_frames % 20 == 1:
                    import cv2
                    cv2.imwrite(os.path.join(save, '%s_%05.1f.jpg' % (name, t)), bgr)
        # late measurements: robot frame at capture -> odom frame (TF at capture time)
        while pend and pend[0][0] <= t + 1e-9:
            _, (cx_, cy_, cyaw), pts = pend.pop(0)
            p = pts.astype(np.float64).copy()
            cr, sr = math.cos(cyaw), math.sin(cyaw)
            px, py = p[:, 0].copy(), p[:, 1].copy()
            p[:, 0] = cx_ + cr * px - sr * py
            p[:, 1] = cy_ + sr * px + cr * py
            p[:, 2] = p[:, 2] + cyaw
            if t_go is None and n_init < 3:
                init_pts.append(p)
                n_init += 1
                if n_init == 3:
                    res = path.initialize(np.vstack(init_pts), ox * sc, oy * sc, oyaw,
                                          kappas=(0.0, T.CURVATURE[lane]))
                    if res is None:
                        n_init, init_pts = 0, []
                    else:
                        last_ok = t
                    if DEBUG:
                        print('init', res)
            elif path.update(p, path.robot_station(ox * sc, oy * sc)[0]):
                n_ok += 1
                last_ok = t
            if DEBUG:
                r_ = W.stadium_r(np.array([x]), np.array([y]))[0]
                _, dd, _ = path.robot_station(ox * sc, oy * sc)
                print('t=%5.2f true off=%+.3f | path off=%+.3f n=%3d per=%s rms=%s st=%s tail=%s' % (
                    t, R - r_, dd, path.last['n'], path.last['per'],
                    '%.3f' % path.last['rms'] if path.last['rms'] is not None else '-', st, path.last_tail))
        path.advance(ox * sc, oy * sc, extend=(last_ok is not None and t - last_ok < 1.0))
        gate.feed('GREEN' if t >= t_green else 'RED', t)
        if gate.go(t) and t_go is None:
            t_go = t
            lapend.feed(ox * sc, oy * sc)
        vx = wz = 0.0
        if t_go is not None:
            rem = lapend.feed(ox * sc, oy * sc)
            if rem < LPm.DEFAULTS['ahead']:
                path.truncate_ahead(ox * sc, oy * sc, rem)
            # path in the robot frame, from the closest point on
            s_r, _, _ = path.robot_station(ox * sc, oy * sc)
            s_cum, _ = path._frames()
            sel = s_cum >= s_r
            P = path.pts[sel]
            c0, s0_ = math.cos(-oyaw), math.sin(-oyaw)
            dx_, dy_ = P[:, 0] - ox * sc, P[:, 1] - oy * sc
            Pr = np.stack([c0 * dx_ - s0_ * dy_, s0_ * dx_ + c0 * dy_], axis=1)
            remaining_len = float(s_cum[-1] - s_r)
            vx, wz = ctrl.step(Pr, remaining_len)
            st = 'RUN'
            if rem <= 0.25 and ctrl.v < 1e-3:
                finished = True
        if finished:
            st = 'FINISH'
            break
        a = dt / 0.25
        vx_a += (vx * 0.97 - vx_a) * a
        moving = abs(vx_a) > 0.05
        wz_a += (wz + (drift if moving else 0.0) - wz_a) * (dt / 0.15)
        nvx = vx_a * (1 + rng.normal(0, 0.03))
        nvy = rng.normal(0, 0.01) * moving
        nwz = wz_a + rng.normal(0, 0.02) * moving
        dxb, dyb, dthb = nvx * dt, nvy * dt, nwz * dt
        x += math.cos(yaw) * dxb - math.sin(yaw) * dyb
        y += math.sin(yaw) * dxb + math.cos(yaw) * dyb
        yaw += dthb
        odx, ody = dxb * odom_true * (1 + rng.normal(0, 0.02)), dyb * odom_true
        ox += math.cos(oyaw) * odx - math.sin(oyaw) * ody
        oy += math.sin(oyaw) * odx + math.cos(oyaw) * ody
        oyaw += dthb + 0.002 * dt
        if other:
            other_s += 0.45 * dt * (T.R_CENTER[lane_o] / R)
        s_now = track_s(x, y, R)
        ds_ = s_now - s_prev
        if ds_ < -lap / 2:
            ds_ += lap
        elif ds_ > lap / 2:
            ds_ -= lap
        s_prev = s_now
        if t_go is not None:
            prog += ds_
        r = W.stadium_r(np.array([x]), np.array([y]))[0]
        max_off = max(max_off, abs(r - R))
        lo, hi = BAND[lane]
        m = 1e9
        for fx, fy in FEET:
            px_ = x + math.cos(yaw) * fx - math.sin(yaw) * fy
            py_ = y + math.sin(yaw) * fx + math.cos(yaw) * fy
            rf = W.stadium_r(np.array([px_]), np.array([py_]))[0]
            m = min(m, rf - lo, hi - rf)
        m -= FOOT_R
        min_margin = min(min_margin, m)
        if m < 0 and not in_viol:
            viol += 1
        in_viol = m < 0
        if t_go is not None and t_fin is None and prog >= need:
            t_fin = t
        t += dt
    return dict(name=name, lap_s=(t_fin - t_go) if (t_fin and t_go) else None, margin_cm=min_margin * 100,
                max_off_cm=max_off * 100, viol=viol, end=st, overshoot_m=prog - need, frames=n_frames,
                ok_frac=n_ok / max(1, n_frames), vis_ms=float(np.median(vis_ms)) if vis_ms else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default='')
    ap.add_argument('--save', default='')
    ap.add_argument('--ds', type=int, default=2)
    a = ap.parse_args()
    names = [n for n in SCEN if (not a.only or n in a.only.split(','))]
    print('%-17s %7s %9s %9s %4s %7s %9s %6s %6s' % ('scenario', 'lap s', 'margin cm', 'max off', 'viol',
                                                      'end', 'overshoot', 'ok%', 'vis ms'))
    for n in names:
        if a.save:
            os.makedirs(a.save, exist_ok=True)
        r = run(n, save=a.save or None, ds=a.ds, **SCEN[n])
        print('%-17s %7s %9.1f %9.1f %4d %7s %9.2f %6.0f %6.1f' % (
            n, '%.1f' % r['lap_s'] if r['lap_s'] else '-', r['margin_cm'], r['max_off_cm'], r['viol'], r['end'],
            r['overshoot_m'], 100 * r['ok_frac'], r['vis_ms']), flush=True)


if __name__ == '__main__':
    main()
