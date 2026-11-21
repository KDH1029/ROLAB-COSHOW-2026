import sys, os, pickle, numpy as np, json
ROOT=os.path.expanduser('~/ws/ROLAB-COSHOW-2026')
sys.path.insert(0, ROOT+'/src/mission/track_day1'); sys.path.insert(0, ROOT+'/scripts'); sys.path.insert(0, ROOT+'/src/mission/track_day1/tools')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import plot_plan, cones_view as V, track_path as TP
plot_plan._font()
lp = TP.load_pins(ROOT+'/logs/day1/routes/rec105326_lap1.csv', 2)
S = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length, 0.05)])
M = V.map_cells(ROOT+'/maps/coshow_1119')
fig, axs = plt.subplots(1, 2, figsize=(17, 7))
for ax, c in zip(axs, sys.argv[1:]):
    cone = np.array([float(v) for v in c.split(',')])
    P = pickle.load(open('plans_inject_%s.pkl' % c, 'rb'))
    first = [q for q in P if q[0] < 15]
    # side of the planned path at the cone, per plan (flip-flop check)
    sides = []
    for q in first:
        pts = q[5]
        d = np.hypot(*(pts - cone).T); j = int(np.argmin(d))
        if d[j] < 1.0 and q[4] != 'taught':
            t_ = np.array(lp.pose_at(lp.station(*cone))[:2]); n = lp.pose_at(lp.station(*cone))[2]
            sides.append(np.sign(-(pts[j,0]-cone[0])*np.sin(n) + (pts[j,1]-cone[1])*np.cos(n)))
    flips = int(np.sum(np.abs(np.diff(sides)) > 0)) if sides else 0
    ax.scatter(M[:,0], M[:,1], s=1, c='0.65')
    ax.plot(S[:,0], S[:,1], '-', c='tab:blue', lw=1.2, label='가르친 경로 lap1')
    for q in first[::10]:
        ax.plot(q[5][:,0], q[5][:,1], '-', c='tab:purple' if q[4]!='taught' else 'tab:cyan', lw=0.8, alpha=0.6)
    # the moment ~1.5 m before the cone
    k = min(range(len(first)), key=lambda i: abs(np.hypot(first[i][1]-cone[0], first[i][2]-cone[1]) - 1.5))
    q = first[k]
    cells, new = q[6], q[7]
    ax.scatter(cells[~new,0], cells[~new,1], s=10, c='tab:red', marker='s', label='스캔 칸 (지도에 있음)')
    ax.scatter(cells[new,0], cells[new,1], s=24, c='magenta', marker='s', label='지도에 없던 칸 (가짜 콘)')
    ax.plot(q[5][:,0], q[5][:,1], '-', c='k', lw=2.5, label='%.1f s 에 보낸 경로 (%s)' % (q[0], q[4]))
    ax.plot(q[1], q[2], 'ko', ms=7)
    ax.add_patch(plt.Circle(cone, 0.15, fill=False, ec='magenta', lw=2))
    ax.set_xlim(-0.8, 3.9); ax.set_ylim(-3.3, -0.6); ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_title('가짜 콘 (%.2f, %.2f) · 그때 최소 여유 %.3f m, 최대 비킴 %.2f m · 경로 갈래 바뀜 %d번 (보라 = 1초마다 경로)'
                 % (cone[0], cone[1], q[8], q[9], flips), fontsize=9)
    ax.legend(fontsize=7, loc='upper left')
fig.savefig(sys.argv[-1], dpi=105, bbox_inches='tight'); print(sys.argv[-1])
