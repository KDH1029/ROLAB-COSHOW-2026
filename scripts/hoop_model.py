#!/usr/bin/env python3
"""Take any YOLOv8 model with the hula-hoop classes and make it the hoop model of the internal-PC node.

    ~/venv/go2/bin/python3 scripts/hoop_model.py <model.pt | model.onnx> [red=<id> yellow=<id>]     (day1.sh hoop use ...)

- .pt   -> ONNX 384x640, opset 17 (the same input as the traffic-light model: 848x480 letterboxed)
- .onnx -> used as is (it must take 1x3x384x640)
- class ids of the red / yellow hoop cards from the model's names: 'rh' / 'yh' (insu_label, tl6_1119), or names with
  red+hoop / yellow+hoop; red= / yellow= override
- writes src/vision/start_light/hoop_384x640.onnx + hoop_classes.txt ("<red id> <yellow id>"), then a quick look at a
  few labelled pictures (logs/hoop_train/data/images/val) if they are there
Next: day1.sh hoop deploy  (push + TensorRT engine on the internal PC with the robot lying down + start the node)
"""
import ast
import glob
import os
import shutil
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
DST = os.path.join(ROOT, 'src/vision/start_light')


def names_of(path):
    if path.endswith('.pt'):
        from ultralytics import YOLO
        return dict(YOLO(path).names)
    import onnx
    meta = {p.key: p.value for p in onnx.load(path).metadata_props}
    return {int(k): v for k, v in ast.literal_eval(meta['names']).items()} if 'names' in meta else {}


def pick(names, colour, short):
    for k, v in names.items():
        if v.lower() == short:
            return k
    for k, v in names.items():
        if colour in v.lower() and 'hoop' in v.lower():
            return k
    return None


def main():
    src = sys.argv[1]
    kv = dict(a.split('=') for a in sys.argv[2:])
    names = names_of(src)
    red = int(kv['red']) if 'red' in kv else pick(names, 'red', 'rh')
    yel = int(kv['yellow']) if 'yellow' in kv else pick(names, 'yellow', 'yh')
    print('model %s, classes %s' % (os.path.basename(src), names or '(no names in the file)'))
    if red is None or yel is None:
        sys.exit('★ no hoop classes found — give them: red=<id> yellow=<id>')
    if src.endswith('.pt'):
        from ultralytics import YOLO
        onnx = YOLO(src).export(format='onnx', imgsz=(384, 640), opset=17, simplify=True, dynamic=False)
    else:
        onnx = src
    import onnx as ox
    shp = [d.dim_value for d in ox.load(onnx).graph.input[0].type.tensor_type.shape.dim]
    if shp != [1, 3, 384, 640]:
        sys.exit('★ the ONNX input is %s, the node needs [1, 3, 384, 640] — give the .pt, it is exported to that' % shp)
    shutil.copy(onnx, os.path.join(DST, 'hoop_384x640.onnx'))
    open(os.path.join(DST, 'hoop_classes.txt'), 'w').write('%d %d\n' % (red, yel))
    print('-> %s/hoop_384x640.onnx, red hoop = class %d (%s), yellow = class %d (%s)'
          % (DST, red, names.get(red, '?'), yel, names.get(yel, '?')))
    # quick look on labelled pictures, with the same pre/post-processing as the node
    pics = sorted(glob.glob(os.path.join(ROOT, 'logs/hoop_train/data/images/val/*.jpg')))[::4]
    if pics:
        import cv2
        sys.path.insert(0, DST)
        import tl_yolo_common as C
        net = cv2.dnn.readNetFromONNX(os.path.join(DST, 'hoop_384x640.onnx'))
        seen = {red: 0, yel: 0}
        for p in pics:
            bgr = cv2.imread(p)
            blob, r, l, t = C.letterbox(bgr, (384, 640))
            net.setInput(blob)
            for c, s, *_ in C.decode(net.forward(), r, l, t, bgr.shape[:2], conf_thr=0.35):
                if c in seen:
                    seen[c] += 1
        print('quick look, %d pictures: red hoop boxes %d, yellow %d (0 for both = wrong classes or a broken export)'
              % (len(pics), seen[red], seen[yel]))
    print('next: day1.sh hoop deploy   (robot lying down: the engine is built on the internal PC)')


if __name__ == '__main__':
    main()
