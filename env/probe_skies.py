"""Find the sun in each HDRI and write a contact sheet. Blender --background --factory-startup --python env/probe_skies.py -- OUT_DIR"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blib
import bpy
import numpy as np

out = sys.argv[sys.argv.index("--") + 1]
res = {}
thumbs = []
for path in sorted(glob.glob(os.path.join(blib.PH, "*/sky.exr"))):
    name = os.path.basename(os.path.dirname(path))
    im = bpy.data.images.load(path)
    w, h = im.size
    a = np.empty(w * h * 4, np.float32)
    im.pixels.foreach_get(a)
    a = a.reshape(h, w, 4)[..., :3]  # row 0 at bottom
    lum = a @ np.array([0.2126, 0.7152, 0.0722], np.float32)
    # blurred max: the sun disc
    r, c = np.unravel_index(np.argmax(lum), lum.shape)
    el = (r + 0.5) / h * 180 - 90
    az_tex = (c + 0.5) / w * 360
    upper = lum[h // 2:]
    res[name] = dict(az_tex=float(az_tex), el=float(el), peak=float(lum[r, c]), mean_upper=float(upper.mean()), median_upper=float(np.median(upper)))
    blib.log(name, res[name])
    # thumbnail, filmic-ish
    step = w // 720
    t = a[::step, ::step]
    t = t / (np.median(upper) * 3.0)
    t = (t / (1 + t)) ** (1 / 2.2)
    thumbs.append((name, (np.clip(t, 0, 1) * 255).astype(np.uint8)[::-1].copy()))
    bpy.data.images.remove(im)
json.dump(res, open(os.path.join(out, "skies.json"), "w"), indent=1)
np.save(os.path.join(out, "skies_thumbs.npy"), np.array([t for _, t in thumbs]))
print(json.dumps({n: i for i, (n, _) in enumerate(thumbs)}))
