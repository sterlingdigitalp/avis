"""Camera setups relative to a flight path, and the renderer for them.

    Blender --background --factory-startup --python env/shots.py -- \
        scenario=layout|<name in out/flight> shots=all|track_side,... mode=still|frames \
        [bird=path.glb] [res=1920x1080] [samples=96] [stride=1] [out=DIR] [frames=A:B] [span=A:B]

Every camera is a pure function of the flight log, the shot's span and the terrain (env/rigs.py). `span=A:B` is the
whole shot as frame indices into the log resampled at fps; without it each shot uses its default fraction of the log.
`scenario=layout` flies a constant-speed stand-in along data/env/corridor.json for framing only; real film frames use
the physics log. No camera position or angle is keyframed by hand.
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
import numpy as np

import blib
import import_flight as IF
import rigs as R
import scene
from rigs import Surface, smooth  # noqa: F401  (film/blender_shot.py uses S.Surface)

ROOT = blib.ROOT


def window(fl, t0, t1):
    """The span a shot uses when the caller passes none. Module-level on purpose: film/blender_shot.py (before it
    passes span=) replaces it with its hold window."""
    return R.default_window(fl, t0, t1)


def _shot(name):
    def shot(fl, s, span=None):
        if span is None:
            span = window(fl, *R.DEFAULT_WINDOW[name])
        a, b = int(span[0]), int(span[1])
        return R.SHOTS[name](fl, s, (a, max(b, a + 2)))
    shot.__name__ = name
    shot.__doc__ = R.SHOTS[name].__doc__
    return shot


# name -> fn(flight, surface, span=None) -> dict(frames, cam, target, lens); see env/rigs.py
SHOTS = {name: _shot(name) for name in R.SHOTS}


def parse():
    a = sys.argv[sys.argv.index("--") + 1:]
    kw = dict(scenario="layout", shots="all", mode="still", bird="", res="1920x1080", samples="96", stride="1", out="", frames="", span="",
              fps="30", world="0.55", sun="16", haze="0", exposure="0", look="AgX - Medium High Contrast", suffix="")
    for p in a:
        k, v = p.split("=", 1)
        kw[k] = v
    return kw


def set_camera(ob, cam, tgt, lens):
    from mathutils import Vector
    ob.location = tuple(cam)
    ob.rotation_mode = "QUATERNION"
    ob.rotation_quaternion = (Vector(tgt) - Vector(cam)).to_track_quat("-Z", "Y")
    ob.data.lens = lens


def main():
    kw = parse()
    fps = float(kw["fps"])
    blib.clear_scene()
    W, H = [int(v) for v in kw["res"].split("x")]
    s = blib.setup_cycles(samples=int(kw["samples"]), res=(W, H), adaptive=0.015)
    blib.set_look(kw["look"])
    s.view_settings.exposure = float(kw["exposure"])
    scene.setup_world(world_gain=float(kw["world"]), sun_gain=float(kw["sun"]))
    scene.build_terrain()
    if float(kw["haze"]) > 0:
        scene.add_haze(float(kw["haze"]))
    if kw["scenario"] == "layout":
        flight = IF.corridor_flight(fps=fps)
    else:
        flight = IF.load_flight(os.path.join(ROOT, f"out/flight/{kw['scenario']}.json"), fps=fps)
    rig = IF.import_glb(kw["bird"]) if kw["bird"] else IF.make_proxy()
    s.render.use_persistent_data = True
    surface = Surface()
    forest = scene.Forest()
    names = list(SHOTS) if kw["shots"] == "all" else kw["shots"].split(",")
    out = kw["out"] or os.path.join(blib.OUT, "shots", kw["scenario"])
    os.makedirs(out, exist_ok=True)
    timing = {}
    for name in names:
        sh = SHOTS[name](flight, surface, span=tuple(int(v) for v in kw["span"].split(":")) if kw["span"] else None)
        a, b = sh["frames"]
        cam_ob = blib.camera(name, sh["cam"][a], sh["target"][a], lens=sh["lens"])
        # Bake the LOD and frustum set over the shot, once.
        tuples = []
        half = 0.5 * 2 * math.atan(cam_ob.data.sensor_width / (2 * sh["lens"]))
        for i in np.linspace(a, b - 1, 12).astype(int):
            fwd = sh["target"][i] - sh["cam"][i]
            tuples.append((sh["cam"][i], fwd / np.linalg.norm(fwd), half))
        forest.scatter(tuples)
        if kw["mode"] == "still":
            idx = [a + int((b - a) * 0.55)]
        elif kw["frames"]:
            f0, f1 = [int(v) for v in kw["frames"].split(":")]
            idx = range(a + f0, min(a + f1, b), int(kw["stride"]))
        else:
            idx = range(a, b, int(kw["stride"]))
        for i in idx:
            set_camera(cam_ob, sh["cam"][i], sh["target"][i], sh["lens"])
            IF.apply_frame(rig, flight, i)
            bpy.context.view_layer.update()
            path = os.path.join(out, f"{name}{kw['suffix']}_{i:05d}.png")
            dt = blib.render_to(path)
            timing.setdefault(name, []).append(dt)
        blib.log("SHOT", name, "frames", a, b, "mean %.1fs" % np.mean(timing[name]), "rss %.1f GB" % blib.rss_gb())
    json.dump(timing, open(os.path.join(out, f"timing{kw['suffix']}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
