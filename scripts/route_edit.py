#!/usr/bin/env python3
"""Route editor — pins on the arena map (track frame), the file the runner follows (day1.sh run ... route=<name>).

    python3 scripts/route_edit.py <route.csv> [map=maps/coshow_1119] [raw=<drive raw.csv>]
    (day1.sh edit <name>  opens logs/day1/routes/<name>.csv)

The route is a closed loop through the pins in order; pin 1 (green) is where the robot starts, travel runs 1 -> 2 -> ...
The thin line is what the runner actually follows (pins smoothed). Pin colour = room to the nearest obstacle of the
2D map: red < 0.25 m, orange < 0.35 m, green otherwise (robot half width 0.155 m).

Mouse   left click on empty space  add a pin into the nearest stretch of the route
        left drag a pin            move it
        right click a pin          delete it
Keys    u  undo      s  save (old file kept as .bak_HHMMSS)      q  quit (asks nothing: save first)
        n / b  select next / previous pin      d  delete the selected pin
        r  re-space the pins evenly (0.5 m)     v  show / hide the recorded drive      h  this help
Korean keyboard layout works too (ㄴ = s, ㅕ = u, ...).
"""
import os
import shutil
import sys
import time

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
import track_path as TP   # noqa: E402
import track_spec as T    # noqa: E402

HANGUL = {'ㄴ': 's', 'ㅕ': 'u', 'ㅂ': 'q', 'ㅜ': 'n', 'ㅠ': 'b', 'ㅇ': 'd', 'ㄱ': 'r', 'ㅍ': 'v', 'ㅗ': 'h'}


def load_route(path):
    head = open(path).readline().strip().split(',')
    D = np.genfromtxt(path, delimiter=',', skip_header=1)
    D = np.atleast_2d(D)
    k = {n: i for i, n in enumerate(head)}
    lane = int(D[0, k['lane']]) if 'lane' in k else 1
    return lane, D[:, [k['x_m'], k['y_m']]].astype(float)


def save_route(path, lane, P):
    if os.path.exists(path):
        shutil.copy2(path, path[:-4] + time.strftime('.bak_%H%M%S.csv'))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    nxt = np.roll(P, -1, 0)
    yaw = np.degrees(np.arctan2(nxt[:, 1] - P[:, 1], nxt[:, 0] - P[:, 0]))
    with open(path, 'w') as f:
        f.write('lane,station_m,x_m,y_m,yaw_deg\n')
        for a, (x, y), b in zip(s, P, yaw):
            f.write('%d,%.3f,%.4f,%.4f,%.1f\n' % (lane, a, x, y, b))


def load_map(d):
    z = np.load(os.path.join(d, 'map_clean.npz'))
    y = {}
    for line in open(os.path.join(d, 'map.yaml')):
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1)
            y[k.strip()] = v.split('#')[0].strip()
    raw = open(os.path.join(d, y['image']), 'rb').read().split(b'\n', 3)
    w, h = map(int, raw[1].split())
    occ = np.frombuffer(raw[3], np.uint8).reshape(h, w)[::-1] == 0
    org = [float(v) for v in y['origin'].strip('[]').split(',')]
    res = float(y['resolution'])
    jj, ii = np.nonzero(occ)
    cells = np.column_stack([org[0] + (ii + 0.5) * res, org[1] + (jj + 0.5) * res])
    return z['xyz'], z['height'], cells


def clearance(P, cells):
    if not len(cells):
        return np.full(len(P), 9.0)
    out = np.empty(len(P))
    for i, p in enumerate(P):
        out[i] = np.sqrt(((cells - p) ** 2).sum(1).min()) - 0.025
    return out


def main():
    path = sys.argv[1]
    kv = dict(a.split('=', 1) for a in sys.argv[2:])
    mapd = kv.get('map', os.path.join(ROOT, 'maps/coshow_1119'))
    lane, P = load_route(path)
    M, hz, cells = load_map(mapd)
    raw = None
    rawp = kv.get('raw') or (path[:-4] + '_raw.csv')
    if os.path.exists(rawp):    # recorded drive: columns x_m, y_m (teach / route_from_bag raw files)
        head = open(rawp).readline().split(',')
        ix = next((i for i, h in enumerate(head) if h.strip().startswith('x_m')), 0)
        Rw = np.genfromtxt(rawp, delimiter=',', skip_header=1)
        raw = Rw[:, ix:ix + 2]

    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import plot_plan
    plot_plan._font()
    for k in list(plt.rcParams):
        if k.startswith('keymap.') and k not in ('keymap.zoom', 'keymap.pan', 'keymap.home', 'keymap.back', 'keymap.forward'):
            plt.rcParams[k] = []
    st = dict(P=P.copy(), sel=0, drag=None, hist=[], show_raw=True, saved=True)
    fig, ax = plt.subplots(figsize=(14, 11))
    sel = (hz > 0.05) & (hz < 0.6)
    ax.scatter(M[sel, 0], M[sel, 1], c=hz[sel], cmap='Greys', vmin=-0.3, vmax=0.7, s=0.5, zorder=1)
    for r in (T.R_OUTER, T.R_LANE12, T.R_INFIELD):
        p = plot_plan._stadium(r)
        ax.plot(p[:, 0], p[:, 1], color='#b8c4d6', lw=0.8, zorder=1)
    if raw is not None:
        raw_line, = ax.plot(raw[:, 0], raw[:, 1], color='#f5a623', lw=0.8, alpha=0.7, zorder=2, label='녹화 주행')
    else:
        raw_line = None
    zf = path[:-4] + '.zones.json'
    if os.path.exists(zf):     # zones of this route (route_corridor.py): other driving inside
        import json
        from matplotlib.patches import Rectangle
        for z in json.load(open(zf)):
            b = z['box']
            ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], fill=False, ec='#d4ac0d', ls='--', lw=1.4, zorder=2))
            ax.text(b[0], b[3] + 0.05, '%s: v %.2f, 앞 %.2f m, 모서리 제자리 회전' % (z['name'], z.get('v', 0), z.get('lookahead', 0)),
                    color='#9a7d0a', fontsize=9, zorder=7)
    smooth, = ax.plot([], [], color='#1f6fb2', lw=1.6, zorder=3)
    poly, = ax.plot([], [], color='#1f6fb2', lw=0.6, ls=':', zorder=3)
    dots = ax.scatter([], [], s=60, zorder=5, edgecolors='k', linewidths=0.6)
    selm, = ax.plot([], [], 'o', ms=16, mfc='none', mec='m', mew=2, zorder=6)
    startm, = ax.plot([], [], 'o', ms=18, mfc='none', mec='#27ae60', mew=3, zorder=6)
    arrow = [None]
    labels = []
    ax.set_aspect('equal')
    ax.set_xlim(-T.WALL_LX / 2 - 0.2, T.WALL_LX / 2 + 0.2)
    ax.set_ylim(-T.WALL_LY / 2 - 0.2, T.WALL_LY / 2 + 0.2)
    ax.grid(alpha=0.2)

    def draw():
        P = st['P']
        c = clearance(P, cells)
        col = np.where(c < 0.25, '#e74c3c', np.where(c < 0.35, '#f39c12', '#2ecc71'))
        dots.set_offsets(P)
        dots.set_facecolors(col)
        poly.set_data(np.r_[P[:, 0], P[:1, 0]], np.r_[P[:, 1], P[:1, 1]])
        try:
            lp = TP.PinLoop(P, s0=0.0)
            Q = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length, 0.05)])
            smooth.set_data(np.r_[Q[:, 0], Q[:1, 0]], np.r_[Q[:, 1], Q[:1, 1]])
            cs = clearance(Q[::2], cells)
            L = lp.length
            worst = cs.min()
        except Exception:
            L, worst = float('nan'), float('nan')
        selm.set_data([P[st['sel'], 0]], [P[st['sel'], 1]])
        startm.set_data([P[0, 0]], [P[0, 1]])
        if arrow[0] is not None:
            arrow[0].remove()
        d = P[1] - P[0]
        d = 0.35 * d / max(1e-6, np.hypot(*d))
        arrow[0] = ax.annotate('', xy=P[0] + d, xytext=P[0], arrowprops=dict(arrowstyle='-|>', color='#27ae60', lw=2.5), zorder=7)
        for t in labels:
            t.remove()
        labels.clear()
        for i in range(0, len(P), 5):
            labels.append(ax.text(P[i, 0], P[i, 1], ' %d' % (i + 1), fontsize=8, zorder=7))
        if raw_line is not None:
            raw_line.set_visible(st['show_raw'])
        ax.set_title('%s%s — 핀 %d개, 한 바퀴 %.2f m, 장애물까지 최소 %.2f m (고른 핀 %d: %.2f m)\n'
                     '왼클릭 추가 · 드래그 이동 · 오른클릭 지우기 · u 되돌리기 · s 저장 · q 끝 · h 도움말'
                     % (os.path.basename(path), '' if st['saved'] else ' *', len(P), L, worst, st['sel'] + 1, c[st['sel']]),
                     fontsize=11)
        fig.canvas.draw_idle()

    def push():
        st['hist'] = (st['hist'] + [st['P'].copy()])[-100:]
        st['saved'] = False

    def hit(ev, r=0.12):
        if ev.xdata is None:
            return None
        d = np.hypot(st['P'][:, 0] - ev.xdata, st['P'][:, 1] - ev.ydata)
        i = int(np.argmin(d))
        return i if d[i] < r else None

    def insert(x, y):
        P = st['P']
        A, B = P, np.roll(P, -1, 0)
        AB = B - A
        t = np.clip(((np.array([x, y]) - A) * AB).sum(1) / np.maximum(1e-9, (AB ** 2).sum(1)), 0, 1)
        d = np.hypot(*(A + t[:, None] * AB - [x, y]).T)
        i = int(np.argmin(d))
        push()
        st['P'] = np.insert(P, i + 1, [x, y], axis=0)
        st['sel'] = i + 1

    def on_press(ev):
        if ev.inaxes != ax or fig.canvas.toolbar.mode:
            return
        i = hit(ev)
        if ev.button == 3 and i is not None:
            if len(st['P']) > 4:
                push()
                st['P'] = np.delete(st['P'], i, axis=0)
                st['sel'] = min(i, len(st['P']) - 1)
        elif ev.button == 1 and i is not None:
            push()
            st['drag'], st['sel'] = i, i
        elif ev.button == 1:
            insert(ev.xdata, ev.ydata)
        draw()

    def on_move(ev):
        if st['drag'] is not None and ev.xdata is not None:
            st['P'][st['drag']] = [ev.xdata, ev.ydata]
            draw()

    def on_release(ev):
        st['drag'] = None

    def on_key(ev):
        k = HANGUL.get(ev.key, ev.key)
        P = st['P']
        if k == 'u' and st['hist']:
            st['P'] = st['hist'].pop()
            st['sel'] = min(st['sel'], len(st['P']) - 1)
        elif k == 's':
            save_route(path, lane, P)
            st['saved'] = True
            print('saved %s (%d pins)' % (path, len(P)))
        elif k == 'q':
            if not st['saved']:
                print('★ quit without saving the last changes (s saves)')
            plt.close(fig)
            return
        elif k == 'n':
            st['sel'] = (st['sel'] + 1) % len(P)
        elif k == 'b':
            st['sel'] = (st['sel'] - 1) % len(P)
        elif k == 'd' and len(P) > 4:
            push()
            st['P'] = np.delete(P, st['sel'], axis=0)
            st['sel'] = min(st['sel'], len(st['P']) - 1)
        elif k == 'r':
            push()
            lp = TP.PinLoop(P, s0=0.0)
            st['P'] = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length - 0.25, 0.5)])
            st['sel'] = 0
        elif k == 'v':
            st['show_raw'] = not st['show_raw']
        elif k == 'h':
            print(__doc__)
        else:
            return
        draw()

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_move)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('key_press_event', on_key)
    draw()
    print(__doc__)
    if os.environ.get('ROUTE_EDIT_SELFTEST'):
        fig.savefig(os.environ['ROUTE_EDIT_SELFTEST'], dpi=80)
        return
    try:   # in front of the other windows (it used to open behind the app), large
        w = fig.canvas.manager.window
        w.attributes('-zoomed', True)
        w.lift()
        w.attributes('-topmost', True)
        w.after(2000, lambda: w.attributes('-topmost', False))
        w.focus_force()
    except Exception:
        pass
    print('editor window open — close it (or q) to finish', flush=True)
    plt.show()


if __name__ == '__main__':
    main()
