"""Render a species line-up and close-ups of the generated trees.

    Blender --background --factory-startup --python env/tree_preview.py -- OUT_DIR [lineup|close|lods]
    Blender ... -- OUT_DIR hero VARIANT[,VARIANT] [dist=7.5,3] [res=1920x1080] [samples=64] [tag=t]

hero: the variant in a small mixed stand under the film's real sun and sky (env/scene.py), camera at each
distance from its upper crown with the sun over the shoulder, as the bird would pass it.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blib
import bpy
import numpy as np
import trees

args = sys.argv[sys.argv.index("--") + 1:]
out, mode = args[0], (args[1] if len(args) > 1 else "lineup")
kw = dict(dist="7.5,3", res="1920x1080", samples="64", tag="")
for a in args[3:]:
    k, v = a.split("=", 1)
    kw[k] = v
blib.clear_scene()
s = blib.setup_cycles(samples=96 if mode != "hero" else int(kw["samples"]),
                      res=(1600, 900) if mode != "hero" else tuple(int(v) for v in kw["res"].split("x")), adaptive=0.03)

if mode == "hero":
    import scene
    blib.set_look("AgX - Medium High Contrast")
    SUN = scene.setup_world(world_gain=0.40, sun_gain=22.0)[0]
else:
    SUN = np.array([-0.85, -0.45, 0.28])
    SUN /= np.linalg.norm(SUN)
    sky = os.path.join(blib.PH, "qwantani_late_afternoon_puresky/sky.exr")
    if os.path.exists(sky):
        blib.world_hdri(sky, strength=0.8)
    blib.sun_lamp(SUN, strength=5.0, color=(1.0, 0.72, 0.5))

# ground
m, nt = blib.new_mat("ground")
p = blib.N(nt, "ShaderNodeBsdfPrincipled", Roughness=0.9)
p.inputs["Base Color"].default_value = (0.05, 0.045, 0.03, 1)
blib.L(nt, p.outputs[0], nt.nodes.new("ShaderNodeOutputMaterial").inputs["Surface"])
g = blib.make_mesh("ground", np.array([[-400, -400, 0], [400, -400, 0], [400, 400, 0], [-400, 400, 0]]), np.array([[0, 1, 2], [0, 2, 3]]), [m])
bpy.context.scene.collection.objects.link(bpy.data.objects.new("ground", g))


def put(name, species, H, cr, seed, lod, x, y):
    me = blib.build_tree_mesh(name, species, H, cr, seed, lod)
    ob = bpy.data.objects.new(f"{name}_{lod}", me)
    ob.location = (x, y, 0)
    bpy.context.scene.collection.objects.link(ob)
    return ob


lookup = {v[0]: v for v in trees.VARIANTS}

if mode == "lineup":
    names = ["fir_a", "fir_c", "eng_a", "eng_c", "pin_a", "pin_c", "dou_a", "lar_a", "snag_a", "fir_e", "eng_e"]
    x = -38
    for nm in names:
        v = lookup[nm]
        put(nm, v[1], v[2], v[3], v[4], 1, x, 0)
        x += 8
    blib.camera("cam", (0, -62, 12), (0, 0, 14.5), lens=30)
    blib.render_to(f"{out}/trees_lineup.png")
elif mode == "close":
    for nm, tgt in (("eng_b", 12), ("fir_b", 11), ("pin_b", 17), ("dou_b", 12)):
        for ob in [o for o in bpy.data.objects if o.type == "MESH" and o.name != "ground"]:
            bpy.data.objects.remove(ob)
        v = lookup[nm]
        put(nm, v[1], v[2], v[3], v[4], 0, 0, 0)
        blib.camera("cam", (5.5, -6.5, tgt - 0.8), (0.5, 0, tgt + 0.2), lens=40)
        blib.render_to(f"{out}/tree_close_{nm}.png")
elif mode == "lods":
    v = lookup["eng_b"]
    for i in range(4):
        put(v[0], v[1], v[2], v[3], v[4], i, -21 + 14 * i, 0)
    v = lookup["pin_b"]
    for i in range(4):
        put(v[0], v[1], v[2], v[3], v[4], i, -21 + 14 * i, 22)
    blib.camera("cam", (0, -55, 12), (0, 0, 13), lens=35)
    blib.render_to(f"{out}/trees_lods.png")
    blib.camera("cam2", (-22, -9, 12), (-21, 0, 13.5), lens=40)
    blib.render_to(f"{out}/trees_lod0_close.png")
elif mode == "hero":
    import time
    rng = np.random.default_rng(5)
    s2 = SUN[:2] / np.linalg.norm(SUN[:2])
    side = np.array([-s2[1], s2[0]])
    names = args[2].split(",")
    for nm in names:
        for ob in [o for o in bpy.data.objects if o.type == "MESH" and o.name != "ground"]:
            bpy.data.objects.remove(ob)
        v = lookup[nm]
        t0 = time.time()
        put(nm, v[1], v[2], v[3], v[4], 0, 0, 0)
        blib.log("hero mesh", nm, "%.1fs" % (time.time() - t0), "tris %.1fM" % (sum(len(o.data.polygons) for o in bpy.data.objects if o.type == "MESH") / 1e6))
        # a stand behind it, away from the camera: same species mostly, as in the real forest
        same = [x for x in trees.VARIANTS if x[1] == v[1] and x[0] != nm]
        mix = same + [lookup[k] for k in ("eng_c", "fir_b", "pin_b", "pin_c")]
        for ring, n in ((6.5, 4), (11.0, 6), (16.0, 8)):
            for j in range(n):
                a = math.atan2(-(0.75 * s2[1] + 0.66 * side[1]), -(0.75 * s2[0] + 0.66 * side[0])) + rng.uniform(-1.5, 1.5)
                w = mix[rng.integers(0, len(mix))]
                ob = put(w[0], w[1], w[2], w[3], w[4], 1, ring * math.cos(a), ring * math.sin(a))
                ob.rotation_euler[2] = rng.uniform(0, 6.28)
                ob.scale = (v[2] / w[2],) * 3 if w[1] == v[1] else (1, 1, 1)
        H = v[2]
        for dist in [float(d) for d in kw["dist"].split(",")]:
            tgt = np.array([0.0, 0.0, 0.62 * H])
            off = (0.75 * s2 + 0.66 * side)
            off = off / np.linalg.norm(off)
            cam = tgt + np.array([off[0] * dist, off[1] * dist, 1.2])
            blib.camera("cam", tuple(cam), tuple(tgt + np.array([0, 0, 0.6])), lens=32)
            blib.render_to(f"{out}/hero_{nm}_{dist:g}m{kw['tag']}.png")
blib.log("peak rss GB", blib.rss_gb())
