#!/usr/bin/env python3
"""Picture of a day-1 run: track, wall, the path of the chosen course, start and end marked ON the lane centre line.

The end is one lap past the start plus `extra` (the rear feet must cross the line). A robot pose that is off the
line is drawn as a hollow mark joined to the start point it projects to.
    python3 tools/plot_plan.py [s_in=0.5] [s_out=0.5] [extra=1.2] [out=logs/day1/plan.png]
The runner calls save_plan() once the start is found (logs/day1/plan_HHMMSS.png).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import track_path as TP   # noqa: E402
import track_spec as T    # noqa: E402

COL = {2: '#1f6fb2', 1: '#c0392b'}


def _stadium(r, n=400):
    L = T.STRAIGHT / 2
    a = np.linspace(-math.pi / 2, math.pi / 2, n // 2)
    right = np.column_stack([L + r * np.cos(a), r * np.sin(a)])
    left = np.column_stack([-L - r * np.cos(a), -r * np.sin(a)])
    return np.vstack([right, left, right[:1]])


def _font():
    import matplotlib
    from matplotlib import font_manager
    for f in font_manager.fontManager.ttflist:
        if f.name in ('NanumSquare', 'NanumGothic', 'Noto Sans CJK KR', 'Noto Serif CJK KR'):
            matplotlib.rcParams['font.family'] = f.name
            break
    matplotlib.rcParams['axes.unicode_minus'] = False


def draw_track(ax):
    ax.add_patch(__import__('matplotlib.patches', fromlist=['Rectangle']).Rectangle(
        (-T.WALL_LX / 2, -T.WALL_LY / 2), T.WALL_LX, T.WALL_LY, fill=False, lw=5, ec='#444'))
    o, i = _stadium(T.R_OUTER), _stadium(T.R_INFIELD)
    ax.fill(o[:, 0], o[:, 1], color='#e1e6ee', zorder=0)
    ax.fill(i[:, 0], i[:, 1], color='#f6e6a8', zorder=1)
    for r, c in ((T.R_OUTER, '#333'), (T.R_LANE12, '#333'), (T.R_INFIELD, '#b08a00')):
        p = _stadium(r)
        ax.plot(p[:, 0], p[:, 1], color=c, lw=1.2, zorder=2)
    ax.text(0, 0, '인필드', ha='center', va='center', color='#9a7a00', fontsize=11)
    ax.set_aspect('equal')
    ax.set_xlim(-T.WALL_LX / 2 - 0.3, T.WALL_LX / 2 + 0.3)
    ax.set_ylim(-T.WALL_LY / 2 - 0.3, T.WALL_LY / 2 + 0.3)
    ax.grid(alpha=0.25)


def draw_run(ax, lane, s_start, extra=1.2, loop=None, robot=None, lat0=0.0, blend_m=2.0):
    """Path of one run on ax. robot = (x, y, yaw) in the track frame, if known."""
    loop = loop or TP.RulebookLoop(lane)
    plan = TP.LanePlan(lane, s_start, lat0, extra=extra, blend_m=blend_m, loop=loop)
    col = COL[lane]
    for ln in (1, 2):              # both lane centres, faint
        lp = TP.RulebookLoop(ln)
        q = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length + 0.05, 0.05)])
        ax.plot(q[:, 0], q[:, 1], ls=(0, (4, 4)), color=COL[ln], lw=0.8, alpha=0.35, zorder=3)
    s_lap = s_start + loop.length
    P = plan.points(s_start, s_lap)
    ax.plot(P[:, 0], P[:, 1], color=col, lw=2.6, zorder=4, label='한 바퀴 %.2f m' % loop.length)
    for s in np.arange(s_start + 1.5, s_lap, 3.0):          # travel direction
        x, y, yaw = loop.pose_at(s, plan.off(s))
        ax.annotate('', xy=(x + 0.12 * math.cos(yaw), y + 0.12 * math.sin(yaw)), xytext=(x, y),
                    arrowprops=dict(arrowstyle='-|>', color=col, lw=1.5), zorder=5)
    E = plan.points(s_lap, plan.s_end)
    ax.plot(E[:, 0], E[:, 1], color='#e67e22', lw=6, alpha=0.85, zorder=4, solid_capstyle='round',
            label='출발선 지나 %.1f m 더' % extra)
    sx, sy, syaw = loop.pose_at(s_start)
    ex, ey, eyaw = loop.pose_at(plan.s_end)
    # start line across the lane at the start point
    nx, ny = -math.sin(syaw), math.cos(syaw)
    h = T.LANE_W / 2
    ax.plot([sx - h * nx, sx + h * nx], [sy - h * ny, sy + h * ny], color='k', lw=2.2, zorder=6)
    ax.plot(sx, sy, 'o', ms=13, mfc='#27ae60', mec='white', mew=2, zorder=7, label='출발 (레인 중앙선 위)')
    ax.plot(ex, ey, 's', ms=12, mfc='#c0392b', mec='white', mew=2, zorder=7, label='종료 = 멈추는 곳')
    side = -1 if sy < 0 else 1
    ax.annotate('출발\nstation %.2f m' % (s_start % loop.length), (sx, sy), xytext=(sx - 0.9, sy + side * 0.75),
                ha='center', fontsize=10, color='#1e8449', arrowprops=dict(arrowstyle='-', color='#1e8449'), zorder=8)
    ax.annotate('종료\n출발 + %.2f + %.1f m' % (loop.length, extra), (ex, ey), xytext=(ex + 0.9, ey + side * 0.75),
                ha='center', fontsize=10, color='#a93226', arrowprops=dict(arrowstyle='-', color='#a93226'), zorder=8)
    if robot is not None:
        ax.plot(robot[0], robot[1], 'o', ms=9, mfc='none', mec='k', mew=1.5, zorder=7, label='실제 로봇 위치')
        ax.plot([robot[0], sx], [robot[1], sy], color='k', lw=0.8, ls=':', zorder=6)
        ax.annotate('', xy=(robot[0] + 0.3 * math.cos(robot[2]), robot[1] + 0.3 * math.sin(robot[2])),
                    xytext=robot[:2], arrowprops=dict(arrowstyle='-|>', color='k', lw=1.2), zorder=7)
    ax.legend(loc='upper right', fontsize=9, framealpha=0.9)
    return plan


def save_plan(path, lane, s_start, extra=1.2, loop=None, robot=None, lat0=0.0, blend_m=2.0, title=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _font()
    fig, ax = plt.subplots(figsize=(10, 8))
    draw_track(ax)
    draw_run(ax, lane, s_start, extra, loop, robot, lat0, blend_m)
    ax.set_title(title or ('%s코스 (%d레인) 반시계 한 바퀴' % ('인' if lane == 2 else '아웃', lane)), fontsize=14)
    ax.set_xlabel('x (m), 트랙 중심 원점 · 출발 직선이 아래')
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def main():
    kv = dict(a.split('=') for a in sys.argv[1:])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _font()
    extra = float(kv.get('extra', 1.2))
    fig, axs = plt.subplots(1, 2, figsize=(18, 7.6))
    for ax, lane, key, name in ((axs[0], 2, 's_in', '인코스 (2레인)'), (axs[1], 1, 's_out', '아웃코스 (1레인)')):
        draw_track(ax)
        draw_run(ax, lane, float(kv.get(key, 0.5)), extra)
        ax.set_title('%s · 반시계 한 바퀴 %.2f m + %.1f m' % (name, T.LAP_LEN[lane], extra), fontsize=14)
        ax.set_xlabel('x (m), 트랙 중심 원점 · 출발 직선이 아래')
    fig.suptitle('1일차: 출발·종료는 레인 중앙선 위 (출발 station 0.5 m = 오늘 녹화의 출발 위치)', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = kv.get('out', os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../../logs/day1/plan.png'))
    out = os.path.normpath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=100)
    print(out)


if __name__ == '__main__':
    main()
