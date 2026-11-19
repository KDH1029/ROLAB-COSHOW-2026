#!/usr/bin/env python3
"""On the internal PC (Python 3.8, TensorRT): the hoop FP16 engine on labelled pictures — does the engine still find
the cards the ONNX finds on the NUC (rh 98-100 %, yh 100 %)?  python3 hoop_engine_check.py <dir> <red id> <yellow id>
<dir>/img/*.jpg + <dir>/lab/*.txt (YOLO labels, 0 rh 1 yh). Run from ~/junsu_vision (tl_yolo_common, trt_engine)."""
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.getcwd())
import tl_yolo_common as C  # noqa: E402
from trt_engine import TrtEngine  # noqa: E402

d, red, yel = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
eng = TrtEngine(os.path.join(os.getcwd(), 'hoop_384x640_fp16.engine'))
H, W = eng.input_shape[2], eng.input_shape[3]
MAP = {0: red, 1: yel}
res = {red: [], yel: []}
for ip in sorted(glob.glob(d + '/img/*.jpg')):
    lp = d + '/lab/' + os.path.basename(ip)[:-4] + '.txt'
    if not os.path.exists(lp):
        continue
    bgr = cv2.imread(ip)
    h, w = bgr.shape[:2]
    blob, r, lft, top = C.letterbox(bgr, (H, W))
    dets = C.decode(eng.infer(blob), r, lft, top, (h, w), conf_thr=0.05)
    for line in open(lp):
        p = line.split()
        if not p or int(p[0]) not in MAP:
            continue
        c = MAP[int(p[0])]
        cx, cy, bw, bh = [float(v) for v in p[1:5]]
        g = [(cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h]
        best = 0.0
        for dc, s, x1, y1, x2, y2 in dets:
            if dc != c:
                continue
            ix = max(0, min(g[2], x2) - max(g[0], x1))
            iy = max(0, min(g[3], y2) - max(g[1], y1))
            iou = ix * iy / ((g[2] - g[0]) * (g[3] - g[1]) + (x2 - x1) * (y2 - y1) - ix * iy + 1e-9)
            if iou > 0.4:
                best = max(best, s)
        res[c].append(best)
for c, name in ((red, 'rh red'), (yel, 'yh yellow')):
    a = np.array(res[c])
    if len(a):
        print('%-9s %d cards: found at conf>=0.35 %.0f %% (NUC FP32: rh 98-100 %%, yh 100 %%), conf median %.2f'
              % (name, len(a), 100 * (a >= 0.35).mean(), np.median(a)))
