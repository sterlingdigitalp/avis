"""Where the bird is in a flight shot's frame: pixel position and distance of the bird's body origin on every plate
frame, from the same rig Blender films the shot with (env/rigs.py, solved over the shot's whole span) and Blender's
camera (36 mm sensor, horizontal fit, no roll). film/render.mjs runs it for a segment with a "marker", so the film
can ring a bird too small to see.

    out/env/.venv/bin/python -I film/bird_px.py scenario=... shot=... fps=30 slow=1 i0=600 n=150 [lens=] [res=1920x1080]
        [out=FILE]

Writes JSON to FILE (or prints it): {"px": [[x, y, depth_m], ...], "lens": mm, "span_px": [...]}, span_px being
wingtip to wingtip in pixels, wings level.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "env"))
import numpy as np  # noqa: E402

import import_flight as IF  # noqa: E402
import rigs as R  # noqa: E402

SENSOR_W = 36.0


def pinhole(cam, tgt, lens, pts, w=1920.0, h=1080.0):
    """Pixels (origin top left) and depth of world points `pts` (N, 3) seen from cam toward tgt: Blender's camera
    tracked -Z to the target with Y up (no roll), sensor fit horizontal."""
    fwd = (tgt - cam) / np.linalg.norm(tgt - cam)
    side = np.array([fwd[1], -fwd[0], 0.0]) / math.hypot(fwd[0], fwd[1])
    up = np.cross(side, fwd)
    rel = pts - cam
    depth = rel @ fwd
    f_px = lens / SENSOR_W * w
    return w / 2 + f_px * (rel @ side) / depth, h / 2 - f_px * (rel @ up) / depth, depth


def wingtips(fl, frames, design):
    """World positions of both wingtips (level wing, body frame z = +-semi-span), for the bird's size in pixels."""
    semi = R.wingspan(design) / 2
    out = []
    for side in (1, -1):
        local = np.array([0.0, -side * semi, 0.0])  # body z (right) -> Blender -y
        q = fl["quat"][frames]  # (w, x, y, z)
        w, u = q[:, :1], q[:, 1:]
        t = 2 * np.cross(u, local)
        out.append(local + w * t + np.cross(u, t) + fl["pos"][frames])
    return out


def main(argv):
    kw = dict(lens="", res="1920x1080", slow="1", out="")
    for a in argv:
        k, v = a.split("=", 1)
        kw[k] = v
    fps = float(kw["fps"]) * float(kw["slow"])
    i0, n = int(kw["i0"]), int(kw["n"])
    W, H = [float(v) for v in kw["res"].split("x")]
    fl = IF.load_flight(os.path.join(R.ROOT, "out/flight/%s.json" % kw["scenario"]), fps=fps)
    span = (i0, max(i0 + n, i0 + 2))
    sh = R.SHOTS[kw["shot"]](fl, R.Surface(), span)
    lens = float(kw["lens"]) if kw["lens"] else float(sh["lens"])
    frames = np.arange(i0, i0 + n)
    px, spans = [], []
    tips = wingtips(fl, frames, R.load_design(fl["design"]))
    for k, i in enumerate(frames):
        x, y, d = pinhole(sh["cam"][i], sh["target"][i], lens, fl["pos"][i][None], W, H)
        tx, _, _ = pinhole(sh["cam"][i], sh["target"][i], lens, np.stack([tips[0][k], tips[1][k]]), W, H)
        px.append([round(float(x[0]), 1), round(float(y[0]), 1), round(float(d[0]), 1)])
        spans.append(round(float(abs(tx[0] - tx[1])), 1))
    res = {"px": px, "lens": lens, "span_px": spans}
    if kw["out"]:
        with open(kw["out"], "w") as fh:
            json.dump(res, fh)
    else:
        json.dump(res, sys.stdout)


if __name__ == "__main__":
    main(sys.argv[1:])
