"""Camera setups relative to a flight path, and the renderer for them.

    Blender --background --factory-startup --python env/shots.py -- \
        scenario=layout|<name in out/flight> shots=all|track_side,... mode=still|frames \
        [bird=path.glb] [res=1920x1080] [samples=96] [stride=1] [out=DIR] [frames=A:B]

Every camera is a pure function of the flight log. `scenario=layout` flies a constant-speed stand-in along
data/env/corridor.json for framing only; real film frames use the physics log. No camera position or angle is keyframed by hand.
"""
import json
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
import numpy as np

import blib
import import_flight as IF
import scene

ROOT = blib.ROOT


def smooth(a, win):
    """Moving average with edge hold, so cameras glide over the wingbeat and do not shake with the body."""
    if win <= 1:
        return a
    k = np.ones(win) / win
    pad = win // 2
    if a.ndim == 1:
        return np.convolve(np.pad(a, pad, mode="edge"), k, "valid")[: len(a)]
    return np.stack([smooth(a[:, i], win) for i in range(a.shape[1])], 1)


class Surface:
    """Ground + lidar canopy in Blender axes, read from data/env/terrain_height.bin with env/terrain.js's format."""

    def __init__(self):
        raw = open(os.path.join(ROOT, "data/env/terrain_height.bin"), "rb").read()
        _, _, off, scale, nx, nz, x0, z0, dx, fx, fz, fx0, fz0, fdx = struct.unpack("<4sIffIIfffIIfff", raw[:56])
        self.off, self.scale = off, scale
        n = nx * nz
        self.near = (np.frombuffer(raw, "<u2", n, 56).reshape(nz, nx), x0, z0, dx)
        self.can = np.frombuffer(raw, np.uint8, n, 56 + 2 * n).reshape(nz, nx)
        self.far = (np.frombuffer(raw, "<u2", fx * fz, 56 + 3 * n).reshape(fz, fx), fx0, fz0, fdx)

    def _grid(self, grid, x, z, data=None):
        h, x0, z0, dx = grid
        d = h if data is None else data
        c = np.clip((x - x0) / dx, 0, d.shape[1] - 1.001)
        r = np.clip((z - z0) / dx, 0, d.shape[0] - 1.001)
        c0, r0 = c.astype(int), r.astype(int)
        fc, fr = c - c0, r - r0
        return d[r0, c0] * (1 - fc) * (1 - fr) + d[r0, c0 + 1] * fc * (1 - fr) + d[r0 + 1, c0] * (1 - fc) * fr + d[r0 + 1, c0 + 1] * fc * fr

    def top(self, bx, by):
        """Canopy top height at Blender (x, y)."""
        z = -np.asarray(by, float)
        x = np.asarray(bx, float)
        g = self._grid(self.near, x, z) * self.scale + self.off
        c = self._grid(self.near, x, z, self.can) * 0.25
        return g + c


def frames_of(flight):
    v = smooth(flight["vel"], 45)
    f = v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-3)
    right = np.stack([f[:, 1], -f[:, 0], np.zeros(len(f))], 1)
    right /= np.maximum(np.linalg.norm(right, axis=1, keepdims=True), 1e-3)
    up = np.cross(right, f)
    return f, right, up


def sun_side(flight, right):
    sun, _, _ = scene.sun_vector()
    return np.sign(right[len(right) // 2] @ sun) or 1.0


def lift_clear(cam, surface, margin=3.0):
    t = surface.top(cam[:, 0], cam[:, 1]) + margin
    out = cam.copy()
    out[:, 2] = np.maximum(out[:, 2], smooth(t, 31))
    return out


def window(flight, t0, t1):
    t = flight["t"]
    span = t[-1]
    a = np.searchsorted(t, t0 * span if t0 <= 1 else t0)
    b = np.searchsorted(t, t1 * span if t1 <= 1 else t1)
    return a, max(b, a + 2)


# --- shot definitions: each returns dict(name, frames (first, last), cam (N,3), target (N,3), lens) ---

def track_side(fl, s):
    a, b = window(fl, 0.05, 0.40)
    p = smooth(fl["pos"], 31)
    f, r, u = frames_of(fl)
    side = sun_side(fl, r)
    # On the shadow side looking back into the lit bird and the sun-lit wall is flat; shoot with the sun over the shoulder.
    cam = p + r * side * 15.0 + u * 2.5 - f * 3.0
    return dict(frames=(a, b), cam=lift_clear(cam, s), target=p + f * 2.5, lens=62.0)


def treetop_pass(fl, s):
    a, b = window(fl, 0.10, 0.30)
    m = (a + b) // 2
    p = smooth(fl["pos"], 15)
    f, r, u = frames_of(fl)
    c = p[m] + r[m] * 9.0 - f[m] * 2.0 - u[m] * 3.0
    c[2] = max(c[2], float(s.top(np.array([c[0]]), np.array([c[1]]))[0]) + 1.5)
    n = b - a + 1
    return dict(frames=(a, b), cam=np.tile(c, (len(p), 1)), target=p, lens=26.0)


def wide_valley(fl, s):
    a, b = window(fl, 0.0, 1.0)
    p = smooth(fl["pos"], 61)
    # On the far bank, high: the whole wall of Rundle behind a speck of a bird.
    c = np.array([1900.0, 1650.0, 0.0])
    c[2] = float(s.top(np.array([c[0]]), np.array([c[1]]))[0]) + 90.0
    aim = np.array([300.0, -700.0, p[:, 2].mean() + 240.0])
    tgt = 0.82 * aim + 0.18 * p
    return dict(frames=(a, b), cam=np.tile(c, (len(p), 1)), target=tgt, lens=42.0)


def under_wing(fl, s):
    a, b = window(fl, 0.35, 0.55)
    p = smooth(fl["pos"], 9)
    f, r, u = frames_of(fl)
    side = sun_side(fl, r)
    cam = p - f * 1.05 + r * side * 0.75 - u * 0.62
    tgt = p + r * side * 0.75 + f * 1.2 + u * 0.18
    return dict(frames=(a, b), cam=cam, target=tgt, lens=20.0)


def lead_front(fl, s):
    a, b = window(fl, 0.25, 0.50)
    p = smooth(fl["pos"], 21)
    f, r, u = frames_of(fl)
    cam = p + f * 10.0 + u * 0.8 + r * 2.2
    return dict(frames=(a, b), cam=lift_clear(cam, s), target=p, lens=45.0)


def ridge_rise(fl, s):
    a, b = window(fl, 0.60, 1.0)
    p = smooth(fl["pos"], 25)
    f, r, u = frames_of(fl)
    end = p[b - 1]
    # Fixed on the east shoulder of the ridge; the bird climbs along the crest with the wall of Rundle filling the frame.
    c = end - f[b - 1] * 210.0 - r[b - 1] * 80.0
    c[2] = float(s.top(np.array([c[0]]), np.array([c[1]]))[0]) + 6.0
    return dict(frames=(a, b), cam=np.tile(c, (len(p), 1)), target=p + np.array([0, 0, 2.0]), lens=85.0)


def top_down(fl, s):
    a, b = window(fl, 0.05, 0.30)
    p = smooth(fl["pos"], 25)
    f, r, u = frames_of(fl)
    cam = p + u * 11.0 - f * 7.0 + r * 1.5
    return dict(frames=(a, b), cam=cam, target=p + f * 3.0, lens=30.0)


def river_low(fl, s):
    a, b = window(fl, 0.0, 0.28)
    p = smooth(fl["pos"], 25)
    c = np.array([1110.0, 700.0, 0.0])
    g = scene.Ground()
    c[2] = float(g(np.array([c[0]]), np.array([c[1]]))[0]) + 2.2
    # keep the horizon in frame: aim between the bird and the far valley rather than straight at the bird overhead
    far = np.array([-1400.0, -1900.0, c[2] + 260.0])
    return dict(frames=(a, b), cam=np.tile(c, (len(p), 1)), target=0.45 * p + 0.55 * far, lens=20.0)


SHOTS = dict(track_side=track_side, treetop_pass=treetop_pass, wide_valley=wide_valley, under_wing=under_wing,
             lead_front=lead_front, ridge_rise=ridge_rise, top_down=top_down, river_low=river_low)


def parse():
    a = sys.argv[sys.argv.index("--") + 1:]
    kw = dict(scenario="layout", shots="all", mode="still", bird="", res="1920x1080", samples="96", stride="1", out="", frames="",
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
        sh = SHOTS[name](flight, surface)
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
