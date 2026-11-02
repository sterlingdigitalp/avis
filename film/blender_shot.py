"""Render one flight shot of the film in Blender: the Bow Valley from env/, the camera from env/shots.py (a pure
function of the flight log), the bird from film/export_glb.mjs posed by a bake of the same log.

    Blender --background --factory-startup --python film/blender_shot.py -- \
        scenario=larus-gl-2-corridor shot=track_side glb=out/film/bird/larus-gl-2.glb bake=out/film/bake/<id> \
        frames=I0:I1 out=out/film/shots/<id> [fps=30] [slow=1] [res=1920x1080] [samples=64] [sun=22] [world=0.40]
        [haze=1.6e-5] [look=AgX - Medium High Contrast] [exposure=0] [lens=] [blur=0]

frames are indices into the log as import_flight.load_flight resamples it at fps*slow; each one becomes
out/f_<index>.png. Frames already on disk are skipped, so a killed job resumes where it stopped.
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "env"))
sys.path.insert(0, HERE)

import bpy
import numpy as np

import blib
import import_flight as IF
import scene
import shots as S
import blender_bird as BB


def parse():
    a = sys.argv[sys.argv.index("--") + 1:]
    kw = dict(scenario="larus-gl-2-corridor", shot="track_side", glb="", bake="", frames="0:1", out="", fps="30", slow="1",
              res="1920x1080", samples="64", sun="22", world="0.40", haze="1.6e-5", look="AgX - Medium High Contrast",
              exposure="0", lens="", blur="0", denoise="1")
    for p in a:
        k, v = p.split("=", 1)
        kw[k] = v
    return kw


def main():
    kw = parse()
    t_start = time.time()
    fps = float(kw["fps"]) * float(kw["slow"])
    i0, i1 = [int(v) for v in kw["frames"].split(":")]
    out = kw["out"]
    os.makedirs(out, exist_ok=True)
    todo = [i for i in range(i0, i1) if not os.path.exists(os.path.join(out, "f_%05d.png" % i))]
    if not todo:
        blib.log("nothing to render in", out)
        return
    blib.clear_scene()
    W, H = [int(v) for v in kw["res"].split("x")]
    s = blib.setup_cycles(samples=int(kw["samples"]), res=(W, H), adaptive=0.015, denoise=kw["denoise"] == "1")
    blib.set_look(kw["look"])
    s.view_settings.exposure = float(kw["exposure"])
    scene.setup_world(world_gain=float(kw["world"]), sun_gain=float(kw["sun"]))
    scene.build_terrain()
    if float(kw["haze"]) > 0:
        scene.add_haze(float(kw["haze"]))
    flight = IF.load_flight(os.path.join(ROOT, "out/flight/%s.json" % kw["scenario"]), fps=fps)
    rig = BB.load_bird(kw["glb"])
    bake = BB.load_bake(kw["bake"])
    s.render.use_persistent_data = True
    surface = S.Surface()
    forest = scene.Forest()
    # the shot's camera over our frame window: env's functions pick their window with S.window, so hand them ours
    S.window = lambda fl, t0, t1: (i0, max(i1, i0 + 2))
    sh = S.SHOTS[kw["shot"]](flight, surface)
    lens = float(kw["lens"]) if kw["lens"] else sh["lens"]
    cam_ob = blib.camera(kw["shot"], sh["cam"][i0], sh["target"][i0], lens=lens)
    if float(kw["blur"]) > 0:
        s.render.use_motion_blur = True
        s.render.motion_blur_shutter = float(kw["blur"])
    tuples = []
    half = 0.5 * 2 * math.atan(cam_ob.data.sensor_width / (2 * lens))
    for i in np.linspace(i0, i1 - 1, 8).astype(int):
        fwd = sh["target"][i] - sh["cam"][i]
        tuples.append((sh["cam"][i], fwd / np.linalg.norm(fwd), half))
    forest.scatter(tuples)
    body = rig["body"]
    times = {}
    for i in todo:
        k = bake["index"].get(i)
        if k is None:
            raise RuntimeError("frame %d not in the bake" % i)
        S.set_camera(cam_ob, sh["cam"][i], sh["target"][i], lens)
        body.location = tuple(flight["pos"][i])
        body.rotation_quaternion = tuple(flight["quat"][i])
        BB.pose_frame(rig, bake, k)
        bpy.context.view_layer.update()
        path = os.path.join(out, "f_%05d.png" % i)
        times[i] = blib.render_to(path + ".tmp.png")
        os.replace(path + ".tmp.png", path)
    prev = {}
    tf = os.path.join(out, "timing.json")
    if os.path.exists(tf):
        prev = json.load(open(tf))
    prev.update({str(k): v for k, v in times.items()})
    json.dump(prev, open(tf, "w"))
    blib.log("SHOT", kw["shot"], "frames", len(todo), "mean %.1fs" % np.mean(list(times.values())), "total %.0fs" % (time.time() - t_start),
             "rss %.1f GB" % blib.rss_gb())


if __name__ == "__main__":
    main()
