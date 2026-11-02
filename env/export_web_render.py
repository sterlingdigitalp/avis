"""Cycles-made parts of the web backdrop. Blender --background --factory-startup --python env/export_web_render.py -- atlas|pano [key=value]

atlas: impostors.png + impostors.json, one row of real generated trees, orthographic, transparent film, scene sun.
pano:  sky_pano_raw.png, equirectangular from the corridor mid-point (compressed to JPEG by the caller).
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blib
import bpy
import numpy as np
import scene
import trees as T

WEB = os.path.join(blib.OUT, "web")
os.makedirs(WEB, exist_ok=True)
args = sys.argv[sys.argv.index("--") + 1:]
mode = args[0]
kw = dict(a.split("=", 1) for a in args[1:])
blib.clear_scene()

if mode == "atlas":
    s = blib.setup_cycles(samples=int(kw.get("samples", 128)), res=(2048, 853), adaptive=0.01)
    s.render.film_transparent = True
    s.render.image_settings.color_mode = "RGBA"
    blib.set_look("AgX - Medium High Contrast")
    scene.setup_world(world_gain=0.55, sun_gain=16.0)
    picks = [("eng_b", "engelmann"), ("fir_b", "subalpine"), ("pin_b", "lodgepole"), ("dou_b", "douglas"), ("lar_a", "larch"), ("snag_a", "snag"), ("asp_a", "aspen"), ("fir_d", "subalpine_young")]
    lookup = {v[0]: v for v in T.VARIANTS}
    cell = 0.3
    n = len(picks)
    cells = {}
    for i, (nm, label) in enumerate(picks):
        v = lookup[nm]
        me = blib.build_tree_mesh(nm, v[1], v[2], v[3], v[4], 0)
        ob = bpy.data.objects.new(nm, me)
        bpy.context.scene.collection.objects.link(ob)
        sc = 1.0 / v[2] * 0.985
        ob.scale = (sc, sc, sc)
        ob.location = ((i + 0.5) * cell, 0, 0.005)
        cells[label] = {"u0": i / n, "u1": (i + 1) / n, "v0": 0.0, "v1": 1.0, "base_height_m": v[2]}
    cam = bpy.data.cameras.new("ortho")
    cam.type = "ORTHO"
    cam.ortho_scale = n * cell
    ob = bpy.data.objects.new("ortho", cam)
    bpy.context.scene.collection.objects.link(ob)
    ob.location = (n * cell / 2, -5, 0.5)
    ob.rotation_euler = (math.pi / 2, 0, 0)
    s.camera = ob
    blib.render_to(os.path.join(WEB, "impostors.png"))
    json.dump({"file": "impostors.png", "aspect_cell_w_over_h": cell, "cells": cells}, open(os.path.join(WEB, "impostors.json"), "w"), indent=1)
else:
    s = blib.setup_cycles(samples=int(kw.get("samples", 64)), res=(4096, 2048), adaptive=0.02)
    blib.set_look("AgX - Medium High Contrast")
    scene.setup_world(world_gain=0.55, sun_gain=16.0)
    scene.build_terrain()
    corr = json.load(open(os.path.join(blib.ROOT, "data/env/corridor.json")))
    wp = np.array(corr["waypoints"])
    mid = wp[len(wp) // 2]
    loc = (float(mid[0]), float(-mid[2]), float(mid[1]) + 6.0)
    cam = bpy.data.cameras.new("pano")
    cam.type = "PANO"
    cam.panorama_type = "EQUIRECTANGULAR"
    cam.clip_start = 0.1
    cam.clip_end = 200000.0  # the default far clip of 1 km would cut away every mountain
    ob = bpy.data.objects.new("pano", cam)
    bpy.context.scene.collection.objects.link(ob)
    ob.location = loc
    ob.rotation_euler = (math.pi / 2, 0, -math.pi / 2)  # image centre looks along world +x (east)
    s.camera = ob
    forest = scene.Forest()
    forest.scatter([(np.array(loc), np.array([1.0, 0, 0]), math.pi)], max_hero=0)
    blib.render_to(os.path.join(WEB, "sky_pano_raw.png"))
blib.log("peak rss GB", blib.rss_gb())
