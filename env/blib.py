"""Shared Blender helpers for the Avis environment. Imported from scripts run with --background."""
import json
import math
import os
import sys
import time

import bpy
import numpy as np

ENV = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(ENV)
if ENV not in sys.path:
    sys.path.insert(0, ENV)
OUT = os.path.join(ROOT, "out/env")
PH = os.path.join(OUT, "raw/polyhaven")


def log(*a):
    print("[avis]", *a, flush=True)


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def setup_cycles(samples=64, res=(1920, 1080), denoise=True, adaptive=0.02, bounces=(5, 3, 3, 6), film_exposure=1.0):
    """Metal GPU Cycles. Only touches this session; preferences are never saved in --background."""
    cpu = os.environ.get("AVIS_CPU") == "1"  # CPU previews do not need the machine-wide GPU lock
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "NONE" if cpu else "METAL"
    prefs.get_devices()
    used = []
    for d in prefs.devices:
        d.use = (d.type == "METAL") and not cpu
        if d.use:
            used.append(d.name)
    log("devices", used or ["CPU"])
    s = bpy.context.scene
    s.render.engine = "CYCLES"
    c = s.cycles
    c.device = "CPU" if cpu else "GPU"
    c.samples = samples
    c.use_adaptive_sampling = adaptive > 0
    if adaptive > 0:
        c.adaptive_threshold = adaptive
        c.adaptive_min_samples = max(8, samples // 8)
    c.use_denoising = denoise
    c.denoiser = "OPENIMAGEDENOISE"
    c.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    c.denoising_prefilter = "ACCURATE"
    c.max_bounces, c.diffuse_bounces, c.glossy_bounces, c.transmission_bounces = bounces
    c.volume_bounces = 1
    c.transparent_max_bounces = 8
    c.caustics_reflective = False
    c.caustics_refractive = False
    c.sample_clamp_indirect = 8.0
    c.sample_clamp_direct = 12.0
    c.light_sampling_threshold = 0.01
    c.use_light_tree = False
    s.render.resolution_x, s.render.resolution_y = res
    s.render.resolution_percentage = 100
    s.render.film_transparent = False
    s.view_settings.view_transform = "AgX"
    s.view_settings.look = "AgX - Medium High Contrast" if False else "None"
    s.view_settings.exposure = 0.0
    s.view_settings.gamma = 1.0
    s.render.use_persistent_data = False
    s.render.image_settings.file_format = "PNG"
    s.render.image_settings.color_depth = "8"
    s.render.image_settings.compression = 40
    return s


def set_look(name):
    s = bpy.context.scene
    try:
        s.view_settings.look = name
    except TypeError as e:
        log("look rejected", name, e)


def make_mesh(name, verts, faces, mats=(), mat_index=None, attrs=None, flat_mask=None):
    me = bpy.data.meshes.new(name)
    verts = np.asarray(verts, np.float32)
    faces = np.asarray(faces, np.int32)
    me.vertices.add(len(verts))
    me.vertices.foreach_set("co", verts.ravel())
    me.loops.add(faces.size)
    me.polygons.add(len(faces))
    me.loops.foreach_set("vertex_index", faces.ravel())
    me.polygons.foreach_set("loop_start", np.arange(len(faces), dtype=np.int32) * 3)
    me.update(calc_edges=True)
    for m in mats:
        me.materials.append(m)
    if mat_index is not None:
        a = me.attributes.get("material_index") or me.attributes.new("material_index", "INT", "FACE")
        a.data.foreach_set("value", np.asarray(mat_index, np.int32))
    if attrs:
        for k, (dtype, domain, vals) in attrs.items():
            a = me.attributes.new(k, dtype, domain)
            key = {"FLOAT": "value", "FLOAT2": "vector", "FLOAT_VECTOR": "vector", "FLOAT_COLOR": "color", "BOOLEAN": "value", "INT": "value"}[dtype]
            a.data.foreach_set(key, np.asarray(vals).ravel())
    if flat_mask is not None:
        a = me.attributes.get("sharp_face") or me.attributes.new("sharp_face", "BOOLEAN", "FACE")
        a.data.foreach_set("value", np.asarray(flat_mask, bool))
    me.update()
    return me


def new_mat(name):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    return m, nt


def N(nt, kind, **inputs):
    """Add a node, set inputs by name or index."""
    n = nt.nodes.new(kind)
    for k, v in inputs.items():
        if k.startswith("p_"):
            setattr(n, k[2:], v)
        else:
            sock = n.inputs.get(k.replace("_", " "))
            if sock is None:
                raise KeyError(f"{kind} has no input {k}: {[i.name for i in n.inputs]}")
            if isinstance(v, bpy.types.NodeSocket):
                nt.links.new(v, sock)
            else:
                sock.default_value = v
    return n


def L(nt, a, b):
    nt.links.new(a, b)


def img(path, colorspace="sRGB"):
    im = bpy.data.images.load(path, check_existing=True)
    im.colorspace_settings.name = colorspace
    return im


def tex(nt, path, vec, colorspace="sRGB", proj="FLAT", blend=0.0, ext="REPEAT", interp="Linear"):
    n = nt.nodes.new("ShaderNodeTexImage")
    n.image = img(path, colorspace)
    n.projection = proj
    n.projection_blend = blend
    n.extension = ext
    n.interpolation = interp
    L(nt, vec, n.inputs["Vector"])
    return n


def mix_rgb(nt, fac, a, b, blend="MIX"):
    n = nt.nodes.new("ShaderNodeMix")
    n.data_type = "RGBA"
    n.blend_type = blend
    n.clamp_factor = True
    for sock, val in ((n.inputs["Factor"] if "Factor" in n.inputs else n.inputs[0], fac), (n.inputs[6], a), (n.inputs[7], b)):
        if isinstance(val, bpy.types.NodeSocket):
            L(nt, val, sock)
        else:
            sock.default_value = val
    return n.outputs[2]


def mix_float(nt, fac, a, b):
    n = nt.nodes.new("ShaderNodeMix")
    n.data_type = "FLOAT"
    n.clamp_factor = True
    for sock, val in ((n.inputs[0], fac), (n.inputs[2], a), (n.inputs[3], b)):
        if isinstance(val, bpy.types.NodeSocket):
            L(nt, val, sock)
        else:
            sock.default_value = val
    return n.outputs[0]


def math_node(nt, op, a, b=None, clamp=False):
    n = nt.nodes.new("ShaderNodeMath")
    n.operation = op
    n.use_clamp = clamp
    for i, val in enumerate((a, b)):
        if val is None:
            continue
        if isinstance(val, bpy.types.NodeSocket):
            L(nt, val, n.inputs[i])
        else:
            n.inputs[i].default_value = val
    return n.outputs[0]


def maprange(nt, v, a, b, c=0.0, d=1.0, smooth=True):
    n = nt.nodes.new("ShaderNodeMapRange")
    n.interpolation_type = "SMOOTHSTEP" if smooth else "LINEAR"
    if isinstance(v, bpy.types.NodeSocket):
        L(nt, v, n.inputs[0])
    else:
        n.inputs[0].default_value = v
    n.inputs[1].default_value = a
    n.inputs[2].default_value = b
    n.inputs[3].default_value = c
    n.inputs[4].default_value = d
    return n.outputs[0]


def attr(nt, name, kind="GEOMETRY"):
    n = nt.nodes.new("ShaderNodeAttribute")
    n.attribute_type = kind
    n.attribute_name = name
    return n


def rgba(r, g, b):
    return (r, g, b, 1.0)


NEEDLE = {
    # base (old-to-mid needle colour), tip (new growth), translucent colour, wax bloom (sheen weight). Linear
    # reflectance, deliberately low: real conifer foliage is near 4-8 % in the visible, which is why forests read
    # dark and not lawn-green. Spruce and fir carry white stomatal wax, which turns them blue-green at a glance.
    "engelmann": ((0.012, 0.027, 0.024), (0.030, 0.060, 0.055), (0.06, 0.10, 0.03), 0.15),
    "subalpine": ((0.010, 0.026, 0.019), (0.028, 0.062, 0.046), (0.05, 0.10, 0.025), 0.12),
    "lodgepole": ((0.022, 0.040, 0.016), (0.050, 0.078, 0.026), (0.10, 0.13, 0.03), 0.1),
    "douglas": ((0.018, 0.040, 0.023), (0.050, 0.098, 0.040), (0.08, 0.13, 0.03), 0.15),
    "larch": ((0.36, 0.20, 0.022), (0.55, 0.33, 0.035), (0.75, 0.42, 0.03), 0.0),
    "aspen": ((0.40, 0.27, 0.035), (0.46, 0.34, 0.05), (0.72, 0.50, 0.07), 0.0),
    "grass": ((0.20, 0.155, 0.065), (0.085, 0.11, 0.035), (0.34, 0.30, 0.10), 0.0),
    "shrub": ((0.070, 0.070, 0.022), (0.18, 0.11, 0.024), (0.28, 0.16, 0.03), 0.0),
}
BARK = {
    # texture, target mean linear albedo, saturation kept from the scan, tiles per 0.9 m of trunk, scan mean luminance.
    # Engelmann spruce: thin loose grey-brown scales. Subalpine fir: smooth ash-grey with resin blisters. Lodgepole
    # pine: thin, finely scaly grey-brown. Targets are field-guide colours; the scans only supply structure.
    "engelmann": ("bark_willow_02", (0.042, 0.039, 0.039), 0.25, 1.7, 0.198),
    "subalpine": ("chinese_hackberry_bark", (0.060, 0.060, 0.059), 0.15, 1.0, 0.0674),
    "lodgepole": ("pine_bark", (0.050, 0.045, 0.040), 0.30, 2.2, 0.096),
    "douglas": ("knotted_pine_bark", (0.085, 0.066, 0.052), 0.60, 1.0, 0.059),
    "larch": ("pine_bark", (0.120, 0.082, 0.062), 0.60, 1.6, 0.096),
    "snag": ("tree_bark_03", (0.16, 0.16, 0.16), 0.2, 1.0, 0.25),
    "aspen": ("tree_bark_03", (0.45, 0.44, 0.38), 0.5, 1.0, 0.25),
    "rock": ("marble_cliff_05", (0.115, 0.113, 0.108), 0.35, 1.0, 0.272),
}


def needle_material(species, lod):
    name = f"needle_{species}_{lod}"
    m = bpy.data.materials.get(name)
    if m:
        return m
    m, nt = new_mat(name)
    base, tip, trans, bloom = NEEDLE[species]
    age = attr(nt, "age").outputs["Fac"]
    obj = nt.nodes.new("ShaderNodeObjectInfo")
    # per-tree brightness and hue drift; every stand has lighter and darker individuals
    rnd = obj.outputs["Random"]
    drift = math_node(nt, "MULTIPLY_ADD", rnd, 0.55)
    nt.nodes[-1].inputs[2].default_value = 0.72
    warm = maprange(nt, rnd, 0.35, 1.0, 0.0, 0.5, smooth=False)
    col = mix_rgb(nt, age, rgba(*tip), rgba(*base))
    col = mix_rgb(nt, warm, col, (base[0] * 1.5, base[1] * 1.15, base[2] * 0.75, 1.0))
    gain = {0: 1.0, 1: 0.95, 2: 0.74, 3: 0.56}[lod]
    col_gain = N(nt, "ShaderNodeVectorMath", p_operation="MULTIPLY")
    L(nt, col, col_gain.inputs[0])
    combine = nt.nodes.new("ShaderNodeCombineXYZ")
    v = math_node(nt, "MULTIPLY", drift, gain)
    for i in range(3):
        L(nt, v, combine.inputs[i])
    L(nt, combine.outputs[0], col_gain.inputs[1])
    # waxy needle surface; the glint in low sun matters as much as the albedo
    # far LODs: fewer, larger blades would turn the same gloss into isolated white glints on backlit crowns
    p = N(nt, "ShaderNodeBsdfPrincipled", Roughness=0.42 if lod == 0 else 0.7)
    p.inputs["Specular IOR Level"].default_value = 0.5 if lod == 0 else 0.12
    if bloom > 0 and lod == 0:
        # stomatal wax: a pale blue-grey scatter that grows toward grazing angles. Hero range only: on the sparse
        # enlarged needles of the far LODs it turns backlit crown edges into white flecks that read as snow.
        p.inputs["Sheen Weight"].default_value = bloom * 0.6
        p.inputs["Sheen Roughness"].default_value = 0.45
        p.inputs["Sheen Tint"].default_value = (0.42, 0.60, 0.72, 1.0)
    L(nt, col_gain.outputs[0], p.inputs["Base Color"])
    tr = N(nt, "ShaderNodeBsdfTranslucent")
    trcol = mix_rgb(nt, age, rgba(*trans), (trans[0] * 0.45, trans[1] * 0.45, trans[2] * 0.45, 1))
    L(nt, trcol, tr.inputs["Color"])
    mixs = N(nt, "ShaderNodeMixShader")
    # thick waxy conifer needles pass little light; broad leaves and grass blades pass more
    thin = species in ("aspen", "larch", "grass", "shrub")
    mixs.inputs[0].default_value = (0.28 if thin else 0.13) * {0: 1.0, 1: 1.0, 2: 0.75, 3: 0.55}[lod]
    L(nt, p.outputs[0], mixs.inputs[1])
    L(nt, tr.outputs[0], mixs.inputs[2])
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    L(nt, mixs.outputs[0], out.inputs["Surface"])
    return m


def bark_material(species):
    name = f"bark_{species}"
    m = bpy.data.materials.get(name)
    if m:
        return m
    m, nt = new_mat(name)
    tid, target, sat, rep, lum = BARK[species]
    mp = N(nt, "ShaderNodeMapping")
    L(nt, attr(nt, "buv").outputs["Vector"], mp.inputs["Vector"])
    mp.inputs["Scale"].default_value = (rep, rep, 1.0)
    uv = mp.outputs[0]
    d = tex(nt, f"{PH}/{tid}/diff.jpg", uv)
    h = tex(nt, f"{PH}/{tid}/disp.jpg" if os.path.exists(f"{PH}/{tid}/disp.jpg") else f"{PH}/{tid}/disp.png", uv, "Non-Color")
    r = tex(nt, f"{PH}/{tid}/rough.jpg", uv, "Non-Color") if os.path.exists(f"{PH}/{tid}/rough.jpg") else None
    hs = N(nt, "ShaderNodeHueSaturation", Saturation=sat)
    L(nt, d.outputs[0], hs.inputs["Color"])
    col = mix_rgb(nt, 1.0, hs.outputs[0], rgba(*(t / lum for t in target)), "MULTIPLY")
    # branches without bark uv (uv is zero there) still sample a real texel; only the trunk gets the fine bump.
    p = N(nt, "ShaderNodeBsdfPrincipled")
    L(nt, col, p.inputs["Base Color"])
    if r:
        L(nt, r.outputs[0], p.inputs["Roughness"])
    p.inputs["Specular IOR Level"].default_value = 0.3
    bump = N(nt, "ShaderNodeBump", Strength=1.0, Distance=0.025)
    L(nt, h.outputs[0], bump.inputs["Height"])
    L(nt, bump.outputs[0], p.inputs["Normal"])
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    L(nt, p.outputs[0], out.inputs["Surface"])
    return m


def _grain_wood(name, grain, noise_scale, dark, light, rough, bump_dist):
    """Weathered wood: noise stretched along the grain (object axes scaled by `grain`) through a two-stop ramp."""
    m = bpy.data.materials.get(name)
    if m:
        return m
    m, nt = new_mat(name)
    co = nt.nodes.new("ShaderNodeTexCoord").outputs["Object"]
    mp = N(nt, "ShaderNodeMapping")
    L(nt, co, mp.inputs["Vector"])
    mp.inputs["Scale"].default_value = grain
    n1 = N(nt, "ShaderNodeTexNoise", Scale=noise_scale, Detail=9.0, Roughness=0.62)
    L(nt, mp.outputs[0], n1.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    for el, (pos_, col) in zip(ramp.color_ramp.elements, (dark, light)):
        el.position = pos_
        el.color = (*col, 1)
    L(nt, n1.outputs[0], ramp.inputs[0])
    p = N(nt, "ShaderNodeBsdfPrincipled", Roughness=rough)
    L(nt, ramp.outputs[0], p.inputs["Base Color"])
    bump = N(nt, "ShaderNodeBump", Strength=0.85, Distance=bump_dist)
    L(nt, n1.outputs[0], bump.inputs["Height"])
    L(nt, bump.outputs[0], p.inputs["Normal"])
    L(nt, p.outputs[0], nt.nodes.new("ShaderNodeOutputMaterial").inputs["Surface"])
    return m


def dead_wood_material():
    """Grey-brown branch wood and snags; the grain runs along object Z."""
    return _grain_wood("dead_wood", (14, 14, 0.8), 2.0, (0.33, (0.016, 0.014, 0.012)), (0.68, (0.056, 0.051, 0.046)), 0.8, 0.02)


def driftwood_material():
    """Bleached, sun-silvered stranded logs with long checks; the grain runs along object X."""
    return _grain_wood("driftwood", (0.6, 14, 14), 3.0, (0.30, (0.07, 0.066, 0.06)), (0.55, (0.26, 0.25, 0.235)), 0.85, 0.01)


def _shore_wet(nt):
    """0..1 wetness from the lidar shore distance at the shaded point (world position): the band the river wets."""
    import config as C
    ter = os.path.join(OUT, "terrain")
    if not os.path.exists(f"{ter}/shore_l0.png"):
        return None
    e0 = json.load(open(f"{ter}/meta.json"))["l0_ext"]
    pos = nt.nodes.new("ShaderNodeNewGeometry").outputs["Position"]
    m0 = N(nt, "ShaderNodeMapping")
    L(nt, pos, m0.inputs["Vector"])
    m0.inputs["Scale"].default_value = (1 / (e0[2] - e0[0]), 1 / (e0[3] - e0[1]), 0)
    m0.inputs["Location"].default_value = (-e0[0] / (e0[2] - e0[0]), -e0[1] / (e0[3] - e0[1]), 0)
    t = tex(nt, f"{ter}/shore_l0.png", m0.outputs[0], "Non-Color", ext="CLIP", interp="Cubic")
    d = math_node(nt, "MULTIPLY", mix_float(nt, t.outputs["Alpha"], 1.0, t.outputs["Color"]), C.SHORE_RANGE)
    return maprange(nt, d, 1.5, 0.5)


def cobble_material():
    """Bow River clasts: mostly grey limestone and dolomite, some buff, pink and maroon quartzite from the main ranges,
    a few dark siltstones and white quartz. The per-stone random in `age` picks the lithology; stones in the wet band
    by the waterline go dark and glossy."""
    m = bpy.data.materials.get("cobble")
    if m:
        return m
    m, nt = new_mat("cobble")
    pick = attr(nt, "age").outputs["Fac"]
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.interpolation = "CONSTANT"
    cols = [(0.0, (0.20, 0.20, 0.195)), (0.30, (0.14, 0.14, 0.138)), (0.55, (0.085, 0.085, 0.085)), (0.68, (0.19, 0.165, 0.13)),
            (0.80, (0.17, 0.12, 0.105)), (0.88, (0.10, 0.065, 0.06)), (0.94, (0.055, 0.058, 0.055)), (0.97, (0.30, 0.295, 0.285))]
    els = ramp.color_ramp.elements
    els[0].position, els[0].color = cols[0][0], (*cols[0][1], 1)
    els[1].position, els[1].color = cols[1][0], (*cols[1][1], 1)
    for pos_, c in cols[2:]:
        e = els.new(pos_)
        e.color = (*c, 1)
    L(nt, pick, ramp.inputs[0])
    co = nt.nodes.new("ShaderNodeTexCoord").outputs["Object"]
    n1 = N(nt, "ShaderNodeTexNoise", Scale=60.0, Detail=6.0, Roughness=0.6)
    L(nt, co, n1.inputs["Vector"])
    speck = maprange(nt, n1.outputs[0], 0.3, 0.7, 0.8, 1.15)
    col = mix_rgb(nt, 1.0, ramp.outputs[0], (1, 1, 1, 1), "MULTIPLY")
    c3 = nt.nodes.new("ShaderNodeCombineXYZ")
    for i in range(3):
        L(nt, speck, c3.inputs[i])
    cm = N(nt, "ShaderNodeVectorMath", p_operation="MULTIPLY")
    L(nt, col, cm.inputs[0])
    L(nt, c3.outputs[0], cm.inputs[1])
    col = cm.outputs[0]
    p = N(nt, "ShaderNodeBsdfPrincipled", Roughness=0.6)
    p.inputs["Specular IOR Level"].default_value = 0.4
    wet = _shore_wet(nt)
    if wet is not None:
        col = mix_rgb(nt, wet, col, mix_rgb(nt, 1.0, col, rgba(0.45, 0.46, 0.42), "MULTIPLY"))
        L(nt, mix_float(nt, wet, 0.6, 0.15), p.inputs["Roughness"])
    L(nt, col, p.inputs["Base Color"])
    bump = N(nt, "ShaderNodeBump", Strength=0.3, Distance=0.004)
    L(nt, n1.outputs[0], bump.inputs["Height"])
    L(nt, bump.outputs[0], p.inputs["Normal"])
    L(nt, p.outputs[0], nt.nodes.new("ShaderNodeOutputMaterial").inputs["Surface"])
    return m


def tree_materials(species, lod):
    bark = dead_wood_material() if species == "snag" else bark_material(species)
    if species == "snag":
        return [bark, bark, bark]
    return [bark, needle_material(species, lod), dead_wood_material()]


def cached_arrays(stem, key, build):
    """Load out/env/cache/trees/<stem>_<key>.npz or build and save it, dropping stale keys of the same stem."""
    import glob
    cdir = os.path.join(OUT, "cache/trees")
    os.makedirs(cdir, exist_ok=True)
    p = os.path.join(cdir, f"{stem}_{key}.npz")
    if os.path.exists(p):
        z = np.load(p)
        return {k: z[k] for k in z.files}
    arr = build()
    np.savez(p + ".tmp.npz", **arr)
    os.replace(p + ".tmp.npz", p)
    for old in glob.glob(os.path.join(cdir, f"{stem}.npz")) + glob.glob(os.path.join(cdir, f"{stem}_??????????.npz")):
        if old != p:
            os.remove(old)
    return arr


def build_under_mesh(name, kind, seed):
    import ground
    arr = cached_arrays(name, ground.cache_key(kind), lambda: ground.build_under(kind, seed))
    if kind == "sapling":
        mats = tree_materials("subalpine", 0)
    elif kind == "grass":
        mats = [dead_wood_material(), needle_material("grass", 0), dead_wood_material()]
    elif kind == "shrub":
        mats = [dead_wood_material(), needle_material("shrub", 0), dead_wood_material()]
    elif kind == "meadow":
        mats = [dead_wood_material(), needle_material("grass", 0), needle_material("shrub", 0)]
    elif kind == "cobbles":
        mats = [cobble_material()] * 3
    elif kind == "drift":
        mats = [driftwood_material()] * 3
    elif kind == "rock":
        mats = [bark_material("rock"), dead_wood_material(), dead_wood_material()]
    else:
        mats = [dead_wood_material(), dead_wood_material(), dead_wood_material()]
    return make_mesh(name, arr["verts"], arr["faces"], mats, arr["mat"],
                     attrs={"age": ("FLOAT", "POINT", arr["age"]), "buv": ("FLOAT2", "POINT", arr["buv"])},
                     flat_mask=arr["mat"] == 1)


def build_tree_mesh(name, species, H, cr, seed, lod, cache=True):
    """Geometry cached on disk because the hero LOD is several million triangles."""
    import trees
    if cache:
        arr = cached_arrays(f"{name}_l{lod}", trees.cache_key(species, lod), lambda: trees.build_tree(species, H, cr, seed, lod))
    else:
        arr = trees.build_tree(species, H, cr, seed, lod)
    me = make_mesh(f"{name}_l{lod}", arr["verts"], arr["faces"], tree_materials(species, lod), arr["mat"],
                   attrs={"age": ("FLOAT", "POINT", arr["age"]), "buv": ("FLOAT2", "POINT", arr["buv"])},
                   flat_mask=arr["mat"] == 1)
    return me


def sun_lamp(direction, strength=4.0, angle_deg=0.54, color=(1, 0.8, 0.6)):
    """direction: unit vector in Blender axes pointing from the ground to the sun."""
    from mathutils import Vector
    d = bpy.data.lights.new("sun", "SUN")
    d.energy = strength
    d.angle = math.radians(angle_deg)
    d.color = color
    ob = bpy.data.objects.new("sun", d)
    bpy.context.scene.collection.objects.link(ob)
    ob.rotation_mode = "QUATERNION"
    ob.rotation_quaternion = Vector(direction).to_track_quat("Z", "Y")
    return ob


def world_hdri(path, rot_z_rad=0.0, strength=1.0, below_col=(0.20, 0.20, 0.22)):
    w = bpy.data.worlds.new("world")
    bpy.context.scene.world = w
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    co = nt.nodes.new("ShaderNodeTexCoord").outputs["Generated"]
    mp = N(nt, "ShaderNodeMapping")
    L(nt, co, mp.inputs["Vector"])
    mp.inputs["Rotation"].default_value = (0, 0, rot_z_rad)
    e = nt.nodes.new("ShaderNodeTexEnvironment")
    e.image = img(path, "Linear Rec.709")
    L(nt, mp.outputs[0], e.inputs["Vector"])
    bg = N(nt, "ShaderNodeBackground", Strength=strength)
    L(nt, e.outputs[0], bg.inputs["Color"])
    out = nt.nodes.new("ShaderNodeOutputWorld")
    L(nt, bg.outputs[0], out.inputs["Surface"])
    return w


def camera(name, loc, target, lens=35.0, sensor=36.0):
    from mathutils import Vector
    c = bpy.data.cameras.new(name)
    c.lens = lens
    c.sensor_width = sensor
    c.clip_start = 0.05
    c.clip_end = 200000.0
    ob = bpy.data.objects.new(name, c)
    bpy.context.scene.collection.objects.link(ob)
    ob.location = loc
    ob.rotation_mode = "QUATERNION"
    ob.rotation_quaternion = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y")
    bpy.context.scene.camera = ob
    return ob


def render_to(path):
    s = bpy.context.scene
    s.render.filepath = os.path.abspath(path)
    t0 = time.time()
    bpy.ops.render.render(write_still=True)
    dt = time.time() - t0
    log("render", os.path.basename(path), "%.1fs" % dt)
    return dt


def rss_gb():
    import resource
    # ru_maxrss is bytes on macOS
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e9
