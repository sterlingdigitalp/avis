"""Render review stills. Blender --background --factory-startup --python env/still.py -- OUT_DIR VIEW[,VIEW...] [key=value...]

keys: res=1920x1080 samples=64 trees=1 haze=4e-5 sky=name sun=12 world=0.7 mdt=8.7 look=name exposure=0
A view is a named entry in VIEWS below: camera position (x, y, height above ground) and target (x, y, height above ground).
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blib
import bpy
import numpy as np
import scene

VIEWS = {
    # name: (x, y, agl), (tx, ty, agl or abs z when >500), lens
    "river": ((1250, 760, 22), (-800, -300, 120), 38),
    "rundle": ((1080, 820, 45), (-2600, -1500, 800), 32),
    "sisters": ((780, 60, 38), (6000, -9800, 800), 45),
    "upvalley": ((1200, 200, 35), (-3500, 5000, 300), 28),
    "treetop": ((760, 140, 26), (900, -60, 20), 28),
    "wide": ((1900, 1650, 180), (0, -200, 40), 30),
    "bank": ((1112, 652, 1.7), (-1400, -1900, 520), 22),
    "bend": ((1180, 820, 14), (950, 560, 0), 30),
}


def parse():
    a = sys.argv[sys.argv.index("--") + 1:]
    out, views = a[0], a[1].split(",")
    kw = dict(res="1920x1080", samples="64", trees="1", haze="0", sky="evening_road_01_puresky", sun="22", world="0.40", mdt="8.70", look="AgX - Medium High Contrast", exposure="0", suffix="", save="")
    for p in a[2:]:
        k, v = p.split("=", 1)
        kw[k] = v
    return out, views, kw


def hero_camera(forest, species, ground, near=(820.0, 250.0), dist=7.5):
    """Camera 7.5 m from the upper crown of a real tree of this species near the corridor, sun over the shoulder."""
    import trees as T
    sp = np.array([T.VARIANTS[int(v)][1] for v in range(len(T.VARIANTS))])
    want = np.nonzero(sp[forest.var] == species)[0]
    d = np.hypot(forest.x[want] - near[0], forest.y[want] - near[1])
    ok = want[(d < 600) & (forest.h[want] > 14)]
    ok = ok[np.argsort(np.hypot(forest.x[ok] - near[0], forest.y[ok] - near[1]))]
    sun, _, _ = scene.sun_vector()
    s2 = np.array([sun[0], sun[1]]) / np.hypot(sun[0], sun[1])
    # An edge tree: nothing tall between it and the sun for 25 m, so the camera sees its lit side.
    loc = np.hypot(forest.x - near[0], forest.y - near[1]) < 800
    lx, ly, lh = forest.x[loc], forest.y[loc], forest.h[loc]
    i = ok[0]
    for j in ok[:4000]:
        dx, dy = lx - forest.x[j], ly - forest.y[j]
        along = dx * s2[0] + dy * s2[1]
        across = np.abs(-dx * s2[1] + dy * s2[0])
        if not np.any((along > 1.5) & (along < 25) & (across < 7) & (lh > 0.45 * forest.h[j])):
            i = j
            break
    side = np.array([-s2[1], s2[0]])
    h = float(forest.h[i])
    tgt = np.array([forest.x[i], forest.y[i], forest.z[i] + 0.62 * h])
    off = (0.75 * s2 + 0.66 * side)
    cam = tgt + np.array([off[0] * dist, off[1] * dist, 1.2])
    blib.log("hero", species, "tree", int(i), "height %.1f" % h)
    return blib.camera("hero_" + species, tuple(cam), tuple(tgt + np.array([0, 0, 0.6])), lens=32)


def main():
    out, views, kw = parse()
    t_all = time.time()
    blib.clear_scene()
    W, H = [int(v) for v in kw["res"].split("x")]
    s = blib.setup_cycles(samples=int(kw["samples"]), res=(W, H), adaptive=0.015)
    if kw["look"]:
        blib.set_look(kw["look"])
    s.view_settings.exposure = float(kw["exposure"])
    scene.setup_world(kw["sky"], float(kw["world"]), float(kw["sun"]), mdt=float(kw["mdt"]))
    scene.build_terrain()
    ground = scene.Ground()
    if float(kw["haze"]) > 0:
        scene.add_haze(float(kw["haze"]))
    forest = scene.Forest() if kw["trees"] != "0" else None
    res = {}
    for name in views:
        if name.startswith("hero_"):
            cam = hero_camera(forest, name[5:], ground)
        else:
            (cx, cy, ca), (tx, ty, ta), lens = VIEWS[name]
            cz = float(ground(np.array([cx]), np.array([cy]))[0]) + ca
            tz = ta if ta > 500 else float(ground(np.array([tx]), np.array([ty]))[0]) + ta
            cam = blib.camera(name, (cx, cy, cz), (tx, ty, tz), lens=lens)
        if forest:
            forest.scatter([scene.cam_tuple(cam)])
        t0 = time.time()
        dt = blib.render_to(os.path.join(out, f"{name}{kw['suffix']}.png"))
        res[name] = dt
        blib.log("VIEW", name, "render %.1fs rss %.1f GB" % (dt, blib.rss_gb()))
    blib.log("TOTAL %.1fs" % (time.time() - t_all))


main()
