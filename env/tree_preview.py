"""Render a species line-up and close-ups of the generated trees. Usage: Blender --background --factory-startup --python env/tree_preview.py -- OUT_DIR [lineup|close|lods]"""
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
blib.clear_scene()
s = blib.setup_cycles(samples=96, res=(1600, 900), adaptive=0.03)

SUN = np.array([-0.85, -0.45, 0.28])
SUN /= np.linalg.norm(SUN)
sky = os.path.join(blib.PH, "qwantani_late_afternoon_puresky/sky.exr")
if os.path.exists(sky):
    blib.world_hdri(sky, strength=0.8)
blib.sun_lamp(SUN, strength=5.0, color=(1.0, 0.72, 0.5))

# ground
m, nt = blib.new_mat("ground")
p = blib.N(nt, "ShaderNodeBsdfPrincipled", Roughness=0.9)
p.inputs["Base Color"].default_value = (0.08, 0.075, 0.05, 1)
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
blib.log("peak rss GB", blib.rss_gb())
