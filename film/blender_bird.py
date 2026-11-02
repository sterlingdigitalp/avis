"""The bird in Blender: import the GLB from film/export_glb.mjs, give its named materials Cycles node trees, and pose
it from a pose bake (film/bake_pose.mjs). Nothing is keyframed by hand: every frame's matrices and vertices come
from bird.update() in node, which calls src/physics/kinematics.js.

    rig = load_bird(glb_path)          # env/import_flight.import_glb plus materials and the node map
    bake = load_bake(stem)             # header + float32 frames
    pose_frame(rig, bake, k)           # k-th baked frame

Axes: the GLB and the bake are glTF Y-up; Blender's importer maps (x, y, z) -> (x, -z, y), so a local matrix M becomes
C M C^T with C that permutation, and a vertex v becomes C v.
"""
import json
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "env"))

C = Matrix(((1, 0, 0, 0), (0, 0, -1, 0), (0, 1, 0, 0), (0, 0, 0, 1)))
CT = C.transposed()


# ---- materials -------------------------------------------------------------------------------------------------

def _nodes(name):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    return m, nt, out


def _bsdf(nt, **kw):
    p = nt.nodes.new("ShaderNodeBsdfPrincipled")
    for k, v in kw.items():
        if k in p.inputs:
            p.inputs[k].default_value = v
    return p


def _uv(nt):
    t = nt.nodes.new("ShaderNodeTexCoord")
    return t.outputs["UV"]


def _mapping(nt, vec, scale):
    m = nt.nodes.new("ShaderNodeMapping")
    m.inputs["Scale"].default_value = scale
    nt.links.new(vec, m.inputs["Vector"])
    return m.outputs["Vector"]


def carbon_weave(name, scale=(260.0, 260.0, 260.0), base=0.018, coat=1.0):
    """2x2 twill: a checker of two perpendicular wave bands; tow sheen through roughness and a faint colour lift."""
    m, nt, out = _nodes(name)
    obj = nt.nodes.new("ShaderNodeTexCoord").outputs["Object"]
    vec = _mapping(nt, obj, scale)
    chk = nt.nodes.new("ShaderNodeTexChecker")
    chk.inputs["Scale"].default_value = 1.0
    nt.links.new(vec, chk.inputs["Vector"])
    w1 = nt.nodes.new("ShaderNodeTexWave")
    w1.wave_type = "BANDS"
    w1.bands_direction = "X"
    w1.inputs["Scale"].default_value = 1.0
    nt.links.new(vec, w1.inputs["Vector"])
    w2 = nt.nodes.new("ShaderNodeTexWave")
    w2.wave_type = "BANDS"
    w2.bands_direction = "Y"
    w2.inputs["Scale"].default_value = 1.0
    nt.links.new(vec, w2.inputs["Vector"])
    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "FLOAT"
    nt.links.new(chk.outputs["Fac"], mix.inputs[0])
    nt.links.new(w1.outputs["Fac"], mix.inputs[2])
    nt.links.new(w2.outputs["Fac"], mix.inputs[3])
    rough = nt.nodes.new("ShaderNodeMapRange")
    rough.inputs["To Min"].default_value = 0.22
    rough.inputs["To Max"].default_value = 0.42
    nt.links.new(mix.outputs[0], rough.inputs["Value"])
    col = nt.nodes.new("ShaderNodeMapRange")
    col.inputs["To Min"].default_value = base
    col.inputs["To Max"].default_value = base * 2.6
    nt.links.new(mix.outputs[0], col.inputs["Value"])
    p = _bsdf(nt, **{"Metallic": 0.0, "Coat Weight": coat, "Coat Roughness": 0.04, "Specular IOR Level": 0.5})
    nt.links.new(col.outputs[0], p.inputs["Base Color"])
    nt.links.new(rough.outputs[0], p.inputs["Roughness"])
    nt.links.new(p.outputs[0], out.inputs["Surface"])
    return m


def solid(name, color, rough, metal=0.0, coat=0.0, coat_rough=0.1, sss=0.0, aniso=0.0, sheen=0.0):
    m, nt, out = _nodes(name)
    p = _bsdf(nt, **{"Base Color": (*color, 1.0), "Roughness": rough, "Metallic": metal, "Coat Weight": coat,
                     "Coat Roughness": coat_rough, "Subsurface Weight": sss, "Anisotropic": aniso, "Sheen Weight": sheen})
    if sss > 0:
        p.inputs["Subsurface Radius"].default_value = (0.002, 0.002, 0.002)
    nt.links.new(p.outputs[0], out.inputs["Surface"])
    return m


def brushed(name, color, rough, metal=1.0):
    """Machined metal: fine noise stretched along one axis drives roughness."""
    m, nt, out = _nodes(name)
    obj = nt.nodes.new("ShaderNodeTexCoord").outputs["Object"]
    vec = _mapping(nt, obj, (6000.0, 40.0, 40.0))
    n = nt.nodes.new("ShaderNodeTexNoise")
    n.inputs["Scale"].default_value = 1.0
    n.inputs["Detail"].default_value = 6.0
    nt.links.new(vec, n.inputs["Vector"])
    r = nt.nodes.new("ShaderNodeMapRange")
    r.inputs["To Min"].default_value = rough * 0.75
    r.inputs["To Max"].default_value = rough * 1.3
    nt.links.new(n.outputs["Fac"], r.inputs["Value"])
    p = _bsdf(nt, **{"Base Color": (*color, 1.0), "Metallic": metal})
    nt.links.new(r.outputs[0], p.inputs["Roughness"])
    nt.links.new(p.outputs[0], out.inputs["Surface"])
    return m


def balsa(name):
    m, nt, out = _nodes(name)
    obj = nt.nodes.new("ShaderNodeTexCoord").outputs["Object"]
    vec = _mapping(nt, obj, (40.0, 900.0, 40.0))
    w = nt.nodes.new("ShaderNodeTexWave")
    w.wave_type = "BANDS"
    w.inputs["Scale"].default_value = 1.0
    w.inputs["Distortion"].default_value = 6.0
    w.inputs["Detail"].default_value = 3.0
    nt.links.new(vec, w.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    # grain either side of the page's balsa (#c9a77a, linear 0.58 0.39 0.19), so the wood reads as wood in full sun
    ramp.color_ramp.elements[0].color = (0.50, 0.33, 0.16, 1)
    ramp.color_ramp.elements[1].color = (0.66, 0.46, 0.23, 1)
    nt.links.new(w.outputs["Fac"], ramp.inputs["Fac"])
    p = _bsdf(nt, **{"Roughness": 0.72, "Subsurface Weight": 0.15, "Sheen Weight": 0.2})
    p.inputs["Subsurface Radius"].default_value = (0.004, 0.0025, 0.0012)
    nt.links.new(ramp.outputs["Color"], p.inputs["Base Color"])
    nt.links.new(p.outputs[0], out.inputs["Surface"])
    return m


PET_IOR = 1.64  # biaxially oriented polyester film, the base of Oracover films
# The covering's colour: None draws the clear film. The catalogue entry (film-oracover-air-light) records no colour;
# when it does, set the pigmented film's linear RGB here (or AVIS_FILM_RGB="r,g,b" in the environment).
COVERING_RGB = tuple(float(v) for v in os.environ["AVIS_FILM_RGB"].split(",")) if os.environ.get("AVIS_FILM_RGB") else None


def film(name, haze=0.03, rough=0.07, ior=PET_IOR):
    """Clear covering film. The BOM's covering is Oracover Air Light, 12 um polyester (catalogue
    film-oracover-air-light); the catalogue records no colour, so it is drawn as the clear film, an assumption to
    revisit when the catalogue states one. Optically a thin slab: it reflects 2r/(1+r) of one interface's r
    (Schlick, from the incidence angle on either side: a film has no total internal reflection), transmits the
    rest untinted, and scatters `haze` of what it transmits (adhesive, stretch marks). Heat-shrunk film is smooth
    but never optically flat: a little roughness on the reflection."""
    if COVERING_RGB is not None:
        return pigmented_film(name, COVERING_RGB, rough)
    m, nt, out = _nodes(name)
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    dot = nt.nodes.new("ShaderNodeVectorMath")
    dot.operation = "DOT_PRODUCT"
    nt.links.new(geo.outputs["Normal"], dot.inputs[0])
    nt.links.new(geo.outputs["Incoming"], dot.inputs[1])

    def op(kind, a, b=None):
        n = nt.nodes.new("ShaderNodeMath")
        n.operation = kind
        for k, v in enumerate((a, b)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                n.inputs[k].default_value = v
            else:
                nt.links.new(v, n.inputs[k])
        return n.outputs[0]

    r0 = ((ior - 1) / (ior + 1)) ** 2
    cos = op("ABSOLUTE", dot.outputs["Value"])
    r = op("ADD", op("MULTIPLY", op("POWER", op("SUBTRACT", 1.0, cos), 5.0), 1 - r0), r0)
    R = op("DIVIDE", op("MULTIPLY", r, 2.0), op("ADD", r, 1.0))
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    tr.inputs["Color"].default_value = (1, 1, 1, 1)
    tl = nt.nodes.new("ShaderNodeBsdfTranslucent")
    tl.inputs["Color"].default_value = (1, 1, 1, 1)
    gl = nt.nodes.new("ShaderNodeBsdfGlossy")
    gl.inputs["Roughness"].default_value = rough
    gl.inputs["Color"].default_value = (1, 1, 1, 1)
    m1 = nt.nodes.new("ShaderNodeMixShader")
    m1.inputs["Fac"].default_value = haze
    nt.links.new(tr.outputs[0], m1.inputs[1])
    nt.links.new(tl.outputs[0], m1.inputs[2])
    m2 = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(R, m2.inputs["Fac"])
    nt.links.new(m1.outputs[0], m2.inputs[1])
    nt.links.new(gl.outputs[0], m2.inputs[2])
    nt.links.new(m2.outputs[0], out.inputs["Surface"])
    try:
        m.surface_render_method = "BLENDED"
    except Exception:
        pass
    return m


def pigmented_film(name, rgb, rough):
    """A coloured covering film: the pigment scatters, so it reads as a thin coloured skin that passes a little
    light (a sixth, forward-scattered) rather than a window; the same glossy polyester face as the clear film."""
    m, nt, out = _nodes(name)
    p = _bsdf(nt, **{"Base Color": (*rgb, 1.0), "Roughness": rough, "IOR": PET_IOR, "Specular IOR Level": 0.5})
    tl = nt.nodes.new("ShaderNodeBsdfTranslucent")
    tl.inputs["Color"].default_value = (*rgb, 1.0)
    mix = nt.nodes.new("ShaderNodeMixShader")
    mix.inputs["Fac"].default_value = 1.0 / 6.0
    nt.links.new(p.outputs[0], mix.inputs[1])
    nt.links.new(tl.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs["Surface"])
    return m


def build_materials():
    return {
        "carbon_woven": carbon_weave("avis_carbon_woven", (260.0, 260.0, 260.0)),
        "carbon_pultruded": solid("avis_carbon_pultruded", (0.012, 0.013, 0.015), 0.24, coat=0.7, coat_rough=0.08),
        "carbon_plate": carbon_weave("avis_carbon_plate", (900.0, 900.0, 900.0), coat=0.6),
        "balsa": balsa("avis_balsa"),
        "film_wing": film("avis_film_wing"),
        "film_shell": film("avis_film_shell"),
        "steel_ground": brushed("avis_steel_ground", (0.62, 0.63, 0.64), 0.22),
        "steel_shaft": solid("avis_steel_shaft", (0.72, 0.73, 0.74), 0.15, metal=1.0),
        "acetal": solid("avis_acetal", (0.86, 0.84, 0.79), 0.38, sss=0.25),
        "alu_anodised": brushed("avis_alu_anodised", (0.06, 0.065, 0.075), 0.32),
        "alu_machined": brushed("avis_alu_machined", (0.78, 0.79, 0.8), 0.22),
        "servo_case": brushed("avis_servo_case", (0.05, 0.055, 0.06), 0.36),
        "battery_wrap": solid("avis_battery_wrap", (0.02, 0.022, 0.025), 0.3, coat=0.8, coat_rough=0.2),
        "esc_wrap": solid("avis_esc_wrap", (0.025, 0.027, 0.03), 0.45, coat=0.5, coat_rough=0.3),
        "pcb": solid("avis_pcb", (0.01, 0.04, 0.025), 0.45, coat=0.7, coat_rough=0.15),
        "gps_case": solid("avis_gps_case", (0.03, 0.032, 0.035), 0.55),
        "wire": solid("avis_wire", (0.012, 0.013, 0.015), 0.5, sheen=0.4),
        "copper": solid("avis_copper", (0.8, 0.45, 0.28), 0.25, metal=1.0),
        "titanium": brushed("avis_titanium", (0.55, 0.53, 0.5), 0.3),
        "magnet": solid("avis_magnet", (0.35, 0.36, 0.38), 0.35, metal=1.0),
        "accent": solid("avis_accent", (1.0, 0.55, 0.15), 0.4),
    }


# ---- import and pose --------------------------------------------------------------------------------------------

def load_bird(glb):
    import import_flight as IF
    meta = json.load(open(glb + ".json"))
    rig = IF.import_glb(glb)
    mats = build_materials()
    # only the imported bird's objects, by their GLB names (Blender may suffix .001 on a clash with the scene)
    import re
    objs = {}
    for o in [rig["body"]] + list(rig["body"].children_recursive):
        objs.setdefault(re.sub(r"\.\d{3}$", "", o.name), o)
    for o in list(objs.values()):
        if o.type != "MESH":
            continue
        for slot in o.material_slots:
            if slot.material is None:
                continue
            key = slot.material.name.split(".")[0]
            if key in mats:
                slot.material = mats[key]
    # deforming meshes: plain smooth normals (custom normals from the GLB would not follow the vertices)
    deform = []
    for d in meta["deform"]:
        o = objs.get(d["name"])
        if o is None:
            raise RuntimeError("deform mesh %s missing from the GLB import" % d["name"])
        me = o.data
        if len(me.vertices) != d["count"]:
            raise RuntimeError("%s has %d vertices, bake expects %d" % (d["name"], len(me.vertices), d["count"]))
        if hasattr(me, "has_custom_normals") and me.has_custom_normals:
            try:
                me.free_normals_if_custom()
            except Exception:
                pass
        for p in me.polygons:
            p.use_smooth = True
        deform.append(o)
    posed = []
    for n in meta["posed"]:
        o = objs.get(n)
        if o is None:
            raise RuntimeError("posed node %s missing" % n)
        o.rotation_mode = "QUATERNION"
        posed.append(o)
    rig["meta"] = meta
    rig["posed"] = posed
    rig["deform"] = deform
    rig["objects"] = objs
    return rig


def load_bake(stem):
    h = json.load(open(stem + ".json"))
    data = np.fromfile(stem + ".bin", dtype="<f4").reshape(len(h["frames"]), h["floats_per_frame"])
    h["data"] = data
    h["index"] = {f: k for k, f in enumerate(h["frames"])}
    return h


def check_same_design(rig, bake):
    """A bake drawn on a GLB from another design version mangles the bird, so refuse it loudly."""
    glb_sha, bake_sha = rig["meta"].get("design_sha256"), bake.get("design_sha256")
    if glb_sha != bake_sha:
        raise SystemExit("design mismatch: GLB is v%s (%s), bake is v%s (%s); re-run film/export_glb.mjs and the bake"
                         % (rig["meta"].get("design_version"), str(glb_sha)[:12],
                            bake.get("design_version"), str(bake_sha)[:12]))


def pose_frame(rig, bake, k):
    """Set every posed node's local matrix and every deforming mesh's vertices from baked frame k."""
    row = bake["data"][k]
    o = 0
    for ob in rig["posed"]:
        m = row[o:o + 16].reshape(4, 4).T  # column-major -> rows
        o += 16
        M = Matrix(m.tolist())
        ob.matrix_basis = C @ M @ CT
    for ob, d in zip(rig["deform"], bake["deform"]):
        n = d["count"] * 3
        v = row[o:o + n].reshape(-1, 3)
        o += n
        vb = np.empty_like(v)
        vb[:, 0], vb[:, 1], vb[:, 2] = v[:, 0], -v[:, 2], v[:, 1]
        ob.data.vertices.foreach_set("co", vb.ravel())
        ob.data.update()
