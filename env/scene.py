"""Build the valley in Blender: terrain, terrain material, forest instancing, sky, sun, haze.

Used by still and shot scripts; nothing here renders. All data comes from out/env/terrain (see build_terrain.py,
build_forest.py). Axes are Blender: X east, Y north, Z up, origin at config.ORIGIN_UTM and the origin elevation.
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
import config as C
from blib import L, N, attr, img, log, mix_float, mix_rgb, maprange, math_node, new_mat, rgba, tex
from solar import solar_az_el
import ground as G
import trees as T

TER = os.path.join(blib.OUT, "terrain")
EARTH_R = 6371000.0 * 1.15  # refraction-corrected curvature


def sun_vector(mdt=C.SUN_MDT_HOURS):
    y, m, d = C.SUN_DATE
    az, el = solar_az_el(C.LAT_LON[0], C.LAT_LON[1], y, m, d, mdt + 6.0)
    a, e = math.radians(az), math.radians(el)
    return np.array([math.cos(e) * math.sin(a), math.cos(e) * math.cos(a), math.sin(e)]), az, el


class Ground:
    """Bilinear height query on the finest level that covers a point."""

    def __init__(self):
        self.levels = []
        for lev in ("l0", "l1", "l2", "l3"):
            z = np.load(f"{TER}/level_{lev}.npz")
            self.levels.append((z["ext"], float(z["res"]), z["z"]))

    def __call__(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        out = np.full(x.shape, np.nan)
        for ext, res, z in self.levels:
            ins = np.isnan(out) & (x > ext[0]) & (x < ext[2] - res) & (y > ext[1] + res) & (y < ext[3])
            if not ins.any():
                continue
            c = (x[ins] - ext[0]) / res
            r = (ext[3] - y[ins]) / res
            c0, r0 = c.astype(int), r.astype(int)
            fc, fr = c - c0, r - r0
            out[ins] = z[r0, c0] * (1 - fc) * (1 - fr) + z[r0, c0 + 1] * fc * (1 - fr) + z[r0 + 1, c0] * (1 - fc) * fr + z[r0 + 1, c0 + 1] * fc * fr
        return out


def grid_mesh(name, ext, res, z, hole_ext=None, mats=()):
    h, w = z.shape
    xs = ext[0] + np.arange(w) * res
    ys = ext[3] - np.arange(h) * res
    xx, yy = np.meshgrid(xs, ys)
    zc = z - (xx * xx + yy * yy) / (2 * EARTH_R)
    verts = np.dstack([xx, yy, zc]).reshape(-1, 3).astype(np.float32)
    idx = np.arange(h * w, dtype=np.int64).reshape(h, w)
    a, b, c, d = idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:], idx[1:, :-1]
    keep = np.ones(a.shape, bool)
    if hole_ext is not None:
        cx = 0.5 * (xx[:-1, :-1] + xx[1:, 1:])
        cy = 0.5 * (yy[:-1, :-1] + yy[1:, 1:])
        keep = ~((cx > hole_ext[0]) & (cx < hole_ext[2]) & (cy > hole_ext[1]) & (cy < hole_ext[3]))
    faces = np.concatenate([np.stack([a[keep], d[keep], c[keep]], -1), np.stack([a[keep], c[keep], b[keep]], -1)])
    return blib.make_mesh(name, verts, faces, list(mats))


def inset(ext, m):
    return (ext[0] + m, ext[1] + m, ext[2] - m, ext[3] - m)


def _detail(nt, pos, path, scale, proj_blend=0.25, colorspace="sRGB"):
    mp = N(nt, "ShaderNodeMapping")
    L(nt, pos, mp.inputs["Vector"])
    mp.inputs["Scale"].default_value = (1 / scale, 1 / scale, 1 / scale)
    n = tex(nt, path, mp.outputs[0], colorspace, proj="BOX", blend=proj_blend)
    return n.outputs[0]


# Bedding-plane normal. The Rundle thrust sheet's Palaeozoic limestones dip about 55 degrees to the south-west, so the
# north-east face toward the valley is an escarpment cutting across the beds.
BED_N = (-0.579, -0.579, 0.574)
BED_DIP_DEG = 55.0


def _noise(nt, vec, size, detail=4.0, rough=0.55, w=None):
    """Noise with features of about `size` metres; 1D along the scalar w when given."""
    n = nt.nodes.new("ShaderNodeTexNoise")
    n.inputs["Detail"].default_value = detail
    n.inputs["Roughness"].default_value = rough
    n.inputs["Scale"].default_value = 1.0 / size
    if w is not None:
        n.noise_dimensions = "1D"
        L(nt, w, n.inputs["W"])
    else:
        L(nt, vec, n.inputs["Vector"])
    return n.outputs[0]


def _scale(nt, col, f):
    """Colour times a scalar socket."""
    c = nt.nodes.new("ShaderNodeCombineXYZ")
    for i in range(3):
        if isinstance(f, bpy.types.NodeSocket):
            L(nt, f, c.inputs[i])
        else:
            c.inputs[i].default_value = f
    v = N(nt, "ShaderNodeVectorMath", p_operation="MULTIPLY")
    if isinstance(col, bpy.types.NodeSocket):
        L(nt, col, v.inputs[0])
    else:
        v.inputs[0].default_value = col[:3]
    L(nt, c.outputs[0], v.inputs[1])
    return v.outputs[0]


def _desat(nt, col, sat):
    h = N(nt, "ShaderNodeHueSaturation", Saturation=sat)
    L(nt, col, h.inputs["Color"])
    return h.outputs[0]


def _centred(nt, x, amp):
    """(x - 0.5) * amp"""
    return math_node(nt, "MULTIPLY", math_node(nt, "SUBTRACT", x, 0.5), amp)


def _rock(nt, pos, nz, D):
    """Grey Palaeozoic limestone from a rock-only model: no satellite colour. Returns colour and three bump heights.

    Bedding (formation members at ~70 m, bed sets at ~12 m, beds at ~2.4 m) sets light massive limestone against
    darker thin-bedded units and buff silty beds; joints cut across the beds; water streaks run down steep faces;
    debris lightens ledges. Colours are field-guide choices for weathered limestone, not measurements.
    """
    dot = N(nt, "ShaderNodeVectorMath", p_operation="DOT_PRODUCT")
    L(nt, pos, dot.inputs[0])
    dot.inputs[1].default_value = BED_N
    warp = math_node(nt, "ADD", _centred(nt, _noise(nt, pos, 700.0, 3.0), 60.0), _centred(nt, _noise(nt, pos, 140.0, 3.0), 12.0))
    bed = math_node(nt, "ADD", dot.outputs["Value"], warp)
    u1 = _noise(nt, None, 70.0, 3.0, 0.5, w=bed)
    u2 = _noise(nt, None, 18.0, 4.0, 0.55, w=bed)
    u3 = _noise(nt, None, 2.4, 3.0, 0.5, w=bed)
    massive = maprange(nt, u1, 0.40, 0.60)
    col = mix_rgb(nt, massive, rgba(0.068, 0.070, 0.073), rgba(0.150, 0.150, 0.146))
    buff = math_node(nt, "MULTIPLY", maprange(nt, _noise(nt, None, 38.0, 2.0, 0.5, w=math_node(nt, "ADD", bed, 17.0)), 0.56, 0.68), 0.7)
    col = mix_rgb(nt, buff, col, rgba(0.170, 0.145, 0.110))
    # bands pinch and swell along strike and vanish under debris in patches, so they never read as contour lines
    swell = maprange(nt, _noise(nt, pos, 260.0, 2.0), 0.3, 0.7, 0.15, 0.6)
    swell = math_node(nt, "MULTIPLY", swell, maprange(nt, _noise(nt, pos, 120.0, 3.0), 0.38, 0.62, 0.0, 1.0))
    g2 = math_node(nt, "ADD", 1.0, math_node(nt, "MULTIPLY", _centred(nt, u2, 1.0), swell))
    near3 = maprange(nt, D, 300.0, 2500.0, 0.5, 0.0)
    g3 = math_node(nt, "ADD", 1.0, math_node(nt, "MULTIPLY", _centred(nt, u3, 1.0), near3))
    # joints: blocky fracture sets normal to the beds
    rot = N(nt, "ShaderNodeVectorRotate", p_rotation_type="AXIS_ANGLE", Angle=math.radians(BED_DIP_DEG))
    L(nt, pos, rot.inputs["Vector"])
    rot.inputs["Axis"].default_value = (0.7071, -0.7071, 0.0)
    jm = N(nt, "ShaderNodeMapping")
    L(nt, rot.outputs[0], jm.inputs["Vector"])
    jm.inputs["Scale"].default_value = (1 / 24.0, 1 / 9.0, 1 / 30.0)
    jv = nt.nodes.new("ShaderNodeTexVoronoi")
    jv.feature = "DISTANCE_TO_EDGE"
    jv.inputs["Randomness"].default_value = 0.85
    L(nt, jm.outputs[0], jv.inputs["Vector"])
    jfade = maprange(nt, D, 1500.0, 6000.0, 1.0, 0.35)
    joints = mix_float(nt, jfade, 1.0, maprange(nt, jv.outputs["Distance"], 0.0, 0.06, 0.62, 1.0))
    # water and lichen streaks down steep faces: noise stretched along world Z
    st_m = N(nt, "ShaderNodeMapping")
    L(nt, pos, st_m.inputs["Vector"])
    st_m.inputs["Scale"].default_value = (1 / 7.0, 1 / 7.0, 1 / 70.0)
    st_n = N(nt, "ShaderNodeTexNoise", Scale=1.0, Detail=6.0, Roughness=0.6)
    L(nt, st_m.outputs[0], st_n.inputs["Vector"])
    steep = maprange(nt, nz, 0.75, 0.45)
    streak = mix_float(nt, steep, 1.0, maprange(nt, st_n.outputs[0], 0.36, 0.64, 0.5, 1.12))
    patina = maprange(nt, _noise(nt, pos, 6.0, 5.0, 0.6), 0.3, 0.7, 0.8, 1.06)
    # near range: a real grey limestone scan for the centimetre-to-metre texture
    near = _desat(nt, _detail(nt, pos, f"{blib.PH}/marble_cliff_05/diff.jpg", 12.0), 0.35)
    nbw = N(nt, "ShaderNodeRGBToBW")
    L(nt, near, nbw.inputs[0])
    f_near = maprange(nt, D, 250.0, 1600.0, 1.0, 0.0)
    det = mix_float(nt, f_near, 1.0, math_node(nt, "DIVIDE", nbw.outputs[0], 0.27))
    gain = math_node(nt, "MULTIPLY", math_node(nt, "MULTIPLY", g2, g3), math_node(nt, "MULTIPLY", joints, streak))
    gain = math_node(nt, "MULTIPLY", gain, math_node(nt, "MULTIPLY", patina, det))
    rock = _scale(nt, col, gain)
    # ledges collect light debris; talus aprons at the angle of repose are pale broken limestone
    scree_n = maprange(nt, _noise(nt, pos, 3.0, 4.0, 0.6), 0.3, 0.7, 0.82, 1.12)
    scree_tex = _desat(nt, _detail(nt, pos, f"{blib.PH}/rocks_ground_02/diff.jpg", 4.0), 0.25)
    sbw = N(nt, "ShaderNodeRGBToBW")
    L(nt, scree_tex, sbw.inputs[0])
    scree_det = mix_float(nt, maprange(nt, D, 120.0, 700.0, 1.0, 0.0), 1.0, math_node(nt, "DIVIDE", sbw.outputs[0], 0.14))
    scree = _scale(nt, rgba(0.122, 0.113, 0.098), math_node(nt, "MULTIPLY", scree_n, scree_det))
    ledge = math_node(nt, "MULTIPLY", maprange(nt, nz, 0.52, 0.72), 0.45)
    rock = mix_rgb(nt, ledge, rock, scree)
    talus = math_node(nt, "MULTIPLY", maprange(nt, nz, 0.76, 0.81), maprange(nt, nz, 0.93, 0.89))
    talus = math_node(nt, "MULTIPLY", talus, maprange(nt, _noise(nt, pos, 90.0, 2.0), 0.35, 0.5))
    rock = mix_rgb(nt, talus, rock, scree)
    h_near = _detail(nt, pos, f"{blib.PH}/marble_cliff_05/disp.jpg", 12.0, colorspace="Non-Color")
    return rock, (math_node(nt, "ADD", u2, math_node(nt, "MULTIPLY", u1, 0.6)), jv.outputs["Distance"], h_near), steep, f_near, talus


def terrain_material():
    m, nt = new_mat("terrain")
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    cam = nt.nodes.new("ShaderNodeCameraData")
    pos = geo.outputs["Position"]
    D = cam.outputs["View Distance"]
    # map coordinates
    uvm = N(nt, "ShaderNodeMapping")
    L(nt, pos, uvm.inputs["Vector"])
    w0, s0, e0, n0 = C.AOI
    size = float(e0 - w0)
    uvm.inputs["Scale"].default_value = (1 / size, 1 / size, 0)
    uvm.inputs["Location"].default_value = ((C.ORIGIN_UTM[0] - w0) / size, (C.ORIGIN_UTM[1] - s0) / size, 0)
    alb = tex(nt, f"{TER}/albedo.png", uvm.outputs[0], "sRGB", ext="EXTEND", interp="Cubic")
    maskt = tex(nt, f"{TER}/masks.png", uvm.outputs[0], "Non-Color", ext="EXTEND", interp="Cubic")
    sep = nt.nodes.new("ShaderNodeSeparateColor")
    L(nt, maskt.outputs[0], sep.inputs[0])
    snow, veg, water = sep.outputs[0], sep.outputs[1], sep.outputs[2]
    # 2 m lidar water inside the L0 area replaces the 10 m Sentinel mask there
    meta = json.load(open(f"{TER}/meta.json"))
    e0 = meta["l0_ext"]
    m0 = N(nt, "ShaderNodeMapping")
    L(nt, pos, m0.inputs["Vector"])
    m0.inputs["Scale"].default_value = (1 / (e0[2] - e0[0]), 1 / (e0[3] - e0[1]), 0)
    m0.inputs["Location"].default_value = (-e0[0] / (e0[2] - e0[0]), -e0[1] / (e0[3] - e0[1]), 0)
    w0 = tex(nt, f"{TER}/water_l0.png", m0.outputs[0], "Non-Color", ext="CLIP", interp="Linear")
    water = mix_float(nt, w0.outputs["Alpha"], water, w0.outputs["Color"])
    # distance to the waterline on land, metres / SHORE_RANGE (build_terrain.py); 1 outside the lidar area
    if os.path.exists(f"{TER}/shore_l0.png"):
        sh = tex(nt, f"{TER}/shore_l0.png", m0.outputs[0], "Non-Color", ext="CLIP", interp="Cubic")
        shore = math_node(nt, "MULTIPLY", mix_float(nt, sh.outputs["Alpha"], 1.0, sh.outputs["Color"]), C.SHORE_RANGE)
    else:
        shore = math_node(nt, "MULTIPLY", math_node(nt, "SUBTRACT", 1.0, water), C.SHORE_RANGE)
    canopy = tex(nt, f"{TER}/canopy.png", uvm.outputs[0], "Non-Color", ext="EXTEND", interp="Linear").outputs[0]
    nzs = nt.nodes.new("ShaderNodeSeparateXYZ")
    L(nt, geo.outputs["Normal"], nzs.inputs[0])
    nz = nzs.outputs[2]
    no2 = _noise(nt, pos, 3.0, 5.0, 0.55)

    rock, (h_bed, h_joint, h_near), steep, f_near, talus = _rock(nt, pos, nz, D)

    # --- forest floor and meadow, near range only ---
    floor = _detail(nt, pos, f"{blib.PH}/forest_ground_04/diff.jpg", 3.2)
    # meadow: an aerial grass texture carries the cured grass and small stones between the instanced grass
    mg = _detail(nt, pos, f"{blib.PH}/aerial_grass_rock/diff.jpg", 9.0)
    mcol = mix_rgb(nt, 0.6, alb.outputs[0], rgba(0.115, 0.095, 0.04))
    mcol = mix_rgb(nt, maprange(nt, D, 60.0, 900.0, 0.85, 0.0), mcol, mix_rgb(nt, 1.0, mcol, mg, "OVERLAY"))
    # under the instanced grass: a cured-grass thatch, not bare dark soil
    thatch = _scale(nt, _desat(nt, _detail(nt, pos, f"{blib.PH}/aerial_grass_rock/diff.jpg", 2.5), 0.55), 0.55)
    mcol = mix_rgb(nt, maprange(nt, D, 30.0, 140.0, 0.75, 0.0), mcol, mix_rgb(nt, 0.55, thatch, rgba(0.095, 0.075, 0.038)))
    meadow = _scale(nt, mcol, maprange(nt, no2, 0.3, 0.7, 0.62, 1.32))
    # shade forest floor by its canopy: darker where canopy is dense
    floor_s = _scale(nt, mix_rgb(nt, 1.0, floor, rgba(0.26, 0.24, 0.17), "MULTIPLY"), maprange(nt, veg, 0.25, 0.8, 1.0, 0.45))
    # canopy-covered ground far from any instanced tree stays map coloured (dark forest).
    r_len = N(nt, "ShaderNodeVectorMath", p_operation="LENGTH")
    L(nt, pos, r_len.inputs[0])
    in_forest = maprange(nt, r_len.outputs["Value"], 11500.0, 13000.0, 1.0, 0.0)
    near_ground = maprange(nt, D, 350.0, 1100.0, 1.0, 0.0)
    veg_hi = maprange(nt, canopy, 0.30, 0.65)
    dark_map = mix_rgb(nt, 1.0, alb.outputs[0], rgba(0.5, 0.5, 0.5), "MULTIPLY")
    under = mix_rgb(nt, near_ground, dark_map, floor_s)
    veg_col = mix_rgb(nt, math_node(nt, "MULTIPLY", veg_hi, in_forest), meadow, under)
    # Rock or vegetation, crisply: a noisy threshold on the 10 m cover map gives a hard, ragged edge at 5-40 m
    # scales instead of a 10 m blur; cliffs steeper than about 52 degrees hold no vegetation.
    thr = math_node(nt, "ADD", 0.21, math_node(nt, "ADD", _centred(nt, _noise(nt, pos, 35.0, 3.0), 0.26), _centred(nt, _noise(nt, pos, 7.0, 3.0), 0.10)))
    vm = maprange(nt, math_node(nt, "SUBTRACT", veg, thr), -0.025, 0.025)
    vm = math_node(nt, "MULTIPLY", vm, maprange(nt, nz, 0.50, 0.62))
    # inside the L1 area the same edge comes from cover_l1.png, which build_forest.py also scatters stems from
    if os.path.exists(f"{TER}/cover_l1.png"):
        cm = json.load(open(f"{TER}/cover_l1.json"))["ext"]
        mc = N(nt, "ShaderNodeMapping")
        L(nt, pos, mc.inputs["Vector"])
        mc.inputs["Scale"].default_value = (1 / (cm[2] - cm[0]), 1 / (cm[3] - cm[1]), 0)
        mc.inputs["Location"].default_value = (-cm[0] / (cm[2] - cm[0]), -cm[1] / (cm[3] - cm[1]), 0)
        cv = tex(nt, f"{TER}/cover_l1.png", mc.outputs[0], "Non-Color", ext="CLIP", interp="Cubic")
        vm = mix_float(nt, cv.outputs["Alpha"], vm, maprange(nt, cv.outputs["Color"], 0.40, 0.60))
    # Limestone only where ground is actually rocky: steeper than about 35 degrees, or high on the walls. Bare gaps
    # on the forested valley slopes are soil and needle litter, not cliff.
    pz = nt.nodes.new("ShaderNodeSeparateXYZ")
    L(nt, pos, pz.inputs[0])
    rocky = math_node(nt, "MAXIMUM", maprange(nt, nz, 0.82, 0.62),
                      math_node(nt, "MULTIPLY", maprange(nt, pz.outputs[2], 110.0, 280.0), maprange(nt, nz, 0.97, 0.90)))
    vm_r = math_node(nt, "MAXIMUM", vm, math_node(nt, "SUBTRACT", 1.0, rocky))
    base = mix_rgb(nt, vm_r, rock, veg_col)
    # bare flat valley floor (gravel bars, dry channels): rounded mixed river cobble beside the water, finer gravel
    # further back, the measured satellite colour beyond texture range. Not the limestone wall model.
    gb = math_node(nt, "MULTIPLY", math_node(nt, "SUBTRACT", 1.0, vm), maprange(nt, nz, 0.86, 0.95))
    # the same cover threshold build_forest.py uses for cobble bars, so painted gravel always carries its stones
    gb = math_node(nt, "MAXIMUM", gb, math_node(nt, "MULTIPLY", maprange(nt, shore, 26.0, 12.0), maprange(nt, veg, 0.24, 0.17)))
    cob = _desat(nt, _detail(nt, pos, f"{blib.PH}/ganges_river_pebbles/diff.jpg", 1.6), 0.55)
    fine = _desat(nt, _detail(nt, pos, f"{blib.PH}/pebble_ground_01/diff.jpg", 1.1), 0.5)
    gsort = math_node(nt, "MAXIMUM", maprange(nt, shore, 26.0, 12.0), maprange(nt, _noise(nt, pos, 6.0, 3.0), 0.45, 0.6))
    gravel_col = _scale(nt, mix_rgb(nt, gsort, fine, cob), 0.5)
    far_bare = _scale(nt, _desat(nt, alb.outputs[0], 0.7), 0.72)
    base = mix_rgb(nt, gb, base, mix_rgb(nt, maprange(nt, D, 150.0, 600.0, 0.0, 1.0), gravel_col, far_bare))
    wob = _centred(nt, _noise(nt, pos, 1.5, 3.0), 0.9)
    wet = maprange(nt, math_node(nt, "ADD", shore, wob), 2.6, 0.5)
    wet = math_node(nt, "MULTIPLY", wet, math_node(nt, "SUBTRACT", 1.0, vm))
    base = mix_rgb(nt, wet, base, mix_rgb(nt, 1.0, base, rgba(0.42, 0.45, 0.40), "MULTIPLY"))
    # snow
    sn = maprange(nt, snow, 0.35, 0.65)
    sn_tex = maprange(nt, no2, 0.35, 0.65, 0.85, 1.0)
    sn_col = mix_rgb(nt, sn_tex, rgba(0.62, 0.68, 0.78), rgba(0.86, 0.88, 0.92))
    base = mix_rgb(nt, sn, base, sn_col)

    p = N(nt, "ShaderNodeBsdfPrincipled")
    L(nt, base, p.inputs["Base Color"])
    rough = mix_float(nt, wet, mix_float(nt, sn, 0.84, 0.55), 0.22)
    L(nt, rough, p.inputs["Roughness"])
    # rough rock and grass scatter back toward the light rather than shading like smooth clay
    L(nt, mix_float(nt, sn, mix_float(nt, vm_r, 0.3, 0.75), 0.2), p.inputs["Diffuse Roughness"])
    p.inputs["Specular IOR Level"].default_value = 0.35

    # relief below the mesh resolution: bedding ledges, joints and near rock texture
    rockiness = math_node(nt, "MULTIPLY", math_node(nt, "SUBTRACT", 1.0, vm_r), math_node(nt, "SUBTRACT", 1.0, sn))
    cliffy = math_node(nt, "MULTIPLY", rockiness, math_node(nt, "SUBTRACT", 1.0, talus))
    b1 = N(nt, "ShaderNodeBump", Distance=1.2)
    L(nt, h_near, b1.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", cliffy, f_near), b1.inputs["Strength"])
    b2 = N(nt, "ShaderNodeBump", Distance=5.0)
    L(nt, h_bed, b2.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", cliffy, maprange(nt, nz, 0.9, 0.6, 0.15, 0.6)), b2.inputs["Strength"])
    L(nt, b1.outputs[0], b2.inputs["Normal"])
    b3 = N(nt, "ShaderNodeBump", Distance=2.5)
    L(nt, h_joint, b3.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", cliffy, maprange(nt, D, 800.0, 6000.0, 1.0, 0.3)), b3.inputs["Strength"])
    L(nt, b2.outputs[0], b3.inputs["Normal"])
    fl_h = _detail(nt, pos, f"{blib.PH}/forest_ground_04/disp.jpg", 3.2, colorspace="Non-Color")
    b4 = N(nt, "ShaderNodeBump", Strength=0.0, Distance=0.14)
    L(nt, fl_h, b4.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", vm, maprange(nt, D, 30.0, 300.0, 1.0, 0.0)), b4.inputs["Strength"])
    L(nt, b3.outputs[0], b4.inputs["Normal"])
    cob_h = _detail(nt, pos, f"{blib.PH}/ganges_river_pebbles/disp.jpg", 1.6, colorspace="Non-Color")
    b5 = N(nt, "ShaderNodeBump", Distance=0.05)
    L(nt, cob_h, b5.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", math_node(nt, "MULTIPLY", gb, maprange(nt, shore, 26.0, 12.0)), maprange(nt, D, 20.0, 150.0, 1.0, 0.0)), b5.inputs["Strength"])
    L(nt, b4.outputs[0], b5.inputs["Normal"])
    L(nt, b5.outputs[0], p.inputs["Normal"])

    # water: glacial flour scatters turquoise under a dielectric surface
    w_sel = maprange(nt, water, 0.42, 0.58)
    wn1 = N(nt, "ShaderNodeTexNoise", Scale=0.9, Detail=4.0, Roughness=0.55)
    wn2 = N(nt, "ShaderNodeTexNoise", Scale=5.5, Detail=3.0, Roughness=0.5)
    mw = N(nt, "ShaderNodeMapping")
    L(nt, pos, mw.inputs["Vector"])
    mw.inputs["Scale"].default_value = (1.0, 1.0, 0.0)
    L(nt, mw.outputs[0], wn1.inputs["Vector"])
    L(nt, mw.outputs[0], wn2.inputs["Vector"])
    wb = N(nt, "ShaderNodeBump", Strength=0.35, Distance=0.05)
    wave = math_node(nt, "ADD", wn1.outputs[0], math_node(nt, "MULTIPLY", wn2.outputs[0], 0.35))
    L(nt, wave, wb.inputs["Height"])
    # a real surface up close, an unresolved rough glint from far away
    wrough = maprange(nt, D, 80.0, 3000.0, 0.035, 0.13)
    wp = N(nt, "ShaderNodeBsdfPrincipled", IOR=1.333)
    # measured on the Bow beside the corridor (Sentinel-2 BOA): blue 0.034 green 0.051 red 0.035, i.e. green glacial water
    wcol = mix_rgb(nt, 0.5, alb.outputs[0], rgba(0.026, 0.062, 0.050))
    L(nt, wcol, wp.inputs["Base Color"])
    L(nt, wrough, wp.inputs["Roughness"])
    wp.inputs["Specular IOR Level"].default_value = 0.5
    L(nt, wb.outputs[0], wp.inputs["Normal"])
    sw = N(nt, "ShaderNodeMixShader")
    L(nt, w_sel, sw.inputs[0])
    L(nt, p.outputs[0], sw.inputs[1])
    L(nt, wp.outputs[0], sw.inputs[2])
    L(nt, sw.outputs[0], nt.nodes.new("ShaderNodeOutputMaterial").inputs["Surface"])
    return m


def build_terrain():
    t0 = time.time()
    mat = terrain_material()
    levels = {}
    for lev in ("l0", "l1", "l2", "l3"):
        z = np.load(f"{TER}/level_{lev}.npz")
        levels[lev] = (tuple(z["ext"].tolist()), float(z["res"]), z["z"])
    col = bpy.data.collections.new("terrain")
    bpy.context.scene.collection.children.link(col)
    order = ["l0", "l1", "l2", "l3"]
    for i, lev in enumerate(order):
        ext, res, z = levels[lev]
        hole = None
        if i > 0:
            finer = levels[order[i - 1]]
            hole = inset(finer[0], max(finer[1] * 3, res * 2.5))
        me = grid_mesh(f"terrain_{lev}", ext, res, z, hole, [mat])
        ob = bpy.data.objects.new(f"terrain_{lev}", me)
        col.objects.link(ob)
        ob.cycles.is_caustics_receiver = False
    log("terrain built %.1fs, %d tris" % (time.time() - t0, sum(len(o.data.polygons) for o in col.objects)))


def setup_world(sky="evening_road_01_puresky", world_gain=0.7, sun_gain=11.0, sun_rgb=(1.0, 0.73, 0.50), mdt=C.SUN_MDT_HOURS, sky_sat=1.0):
    meta = json.load(open(os.path.join(blib.OUT, "review/skies.json")))[sky]
    sun, az, el = sun_vector(mdt)
    phi_sun = math.degrees(math.atan2(sun[1], sun[0]))
    phi_tex = (0.5 - meta["az_tex"] / 360) * 360
    rot = math.radians(phi_tex - phi_sun)
    log("sun az %.1f el %.1f sky rot %.1f" % (az, el, math.degrees(rot)))
    w = blib.world_hdri(os.path.join(blib.PH, sky, "sky.exr"), rot, world_gain)
    blib.sun_lamp(sun, strength=sun_gain, color=sun_rgb)
    return sun, az, el


def _instance_group(name, target):
    ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
    ng.interface.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    ng.interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    gi = ng.nodes.new("NodeGroupInput")
    go = ng.nodes.new("NodeGroupOutput")
    iop = ng.nodes.new("GeometryNodeInstanceOnPoints")
    oi = ng.nodes.new("GeometryNodeObjectInfo")
    oi.transform_space = "ORIGINAL"
    oi.inputs["Object"].default_value = target
    oi.inputs["As Instance"].default_value = True
    na_r = ng.nodes.new("GeometryNodeInputNamedAttribute")
    na_r.data_type = "FLOAT"
    na_r.inputs["Name"].default_value = "rotz"
    na_s = ng.nodes.new("GeometryNodeInputNamedAttribute")
    na_s.data_type = "FLOAT"
    na_s.inputs["Name"].default_value = "scl"
    comb = ng.nodes.new("ShaderNodeCombineXYZ")
    ng.links.new(na_r.outputs["Attribute"], comb.inputs["Z"])
    comb_s = ng.nodes.new("ShaderNodeCombineXYZ")
    for k in "XYZ":
        ng.links.new(na_s.outputs["Attribute"], comb_s.inputs[k])
    ng.links.new(gi.outputs[0], iop.inputs["Points"])
    ng.links.new(oi.outputs["Geometry"], iop.inputs["Instance"])
    ng.links.new(comb.outputs[0], iop.inputs["Rotation"])
    ng.links.new(comb_s.outputs[0], iop.inputs["Scale"])
    ng.links.new(iop.outputs[0], go.inputs[0])
    return ng


LOD_DIST = (34.0, 230.0, 1000.0)


class Forest:
    def __init__(self):
        z = np.load(f"{TER}/forest.npz")
        self.x, self.y, self.z = z["x"], z["y"], z["z"]
        self.h, self.var, self.scale, self.rot = z["h"], z["var"], z["scale"], z["rot"]
        self.objects = {}
        u = np.load(f"{TER}/under.npz")
        self.ux, self.uy, self.uz, self.ukind, self.uscale, self.urot = u["x"], u["y"], u["z"], u["kind"], u["scale"], u["rot"]
        col = bpy.data.collections.new("forest")
        bpy.context.scene.collection.children.link(col)
        self.col = col
        lib = bpy.data.collections.new("tree_lib")
        bpy.context.scene.collection.children.link(lib)
        lib.hide_render = True
        lib.hide_viewport = True
        self.lib = lib

    def tree_object(self, vi, lod):
        key = (vi, lod)
        if key in self.objects:
            return self.objects[key]
        name, sp, H, cr, seed = T.VARIANTS[vi]
        me = blib.build_tree_mesh(name, sp, H, cr, seed, lod)
        ob = bpy.data.objects.new(f"lib_{name}_l{lod}", me)
        self.lib.objects.link(ob)
        self.objects[key] = ob
        return ob

    def scatter(self, cams, fov_deg=60.0, margin_deg=14.0, keep_near=700.0, max_hero=26, lod_dist=LOD_DIST):
        """cams: list of (position, forward unit vector, half-fov radians) in Blender axes, covering one still or shot."""
        t0 = time.time()
        P = np.stack([self.x, self.y, self.z], 1).astype(np.float64)
        d_min = np.full(len(P), np.inf)
        keep = np.zeros(len(P), bool)
        for pos, fwd, half in cams:
            v = P - np.asarray(pos)
            d = np.linalg.norm(v, axis=1) + 1e-6
            d_min = np.minimum(d_min, d)
            cosang = (v @ np.asarray(fwd)) / d
            keep |= (cosang > math.cos(half + math.radians(margin_deg))) | (d < keep_near)
        # very far small stems add nothing and cost BVH nodes
        keep &= ~((d_min > 4000) & (self.h < 6))
        idx = np.nonzero(keep)[0]
        lod = np.digitize(d_min[idx], lod_dist).astype(np.int8)
        # hero budget: only the closest trees get real needles
        if (lod == 0).sum() > max_hero:
            hero = idx[lod == 0]
            order = np.argsort(d_min[hero])
            demote = hero[order[max_hero:]]
            lod[np.isin(idx, demote)] = 1
        total = 0
        for ob in list(self.col.objects):
            bpy.data.objects.remove(ob)
        var = self.var[idx]
        for vi in np.unique(var):
            for ld in range(4):
                sel = idx[(var == vi) & (lod == ld)]
                if len(sel) == 0:
                    continue
                target = self.tree_object(int(vi), ld)
                me = bpy.data.meshes.new(f"pts_{vi}_{ld}")
                me.vertices.add(len(sel))
                me.vertices.foreach_set("co", P[sel].astype(np.float32).ravel())
                a = me.attributes.new("rotz", "FLOAT", "POINT")
                a.data.foreach_set("value", self.rot[sel])
                a = me.attributes.new("scl", "FLOAT", "POINT")
                a.data.foreach_set("value", self.scale[sel])
                me.update()
                ob = bpy.data.objects.new(f"pts_{vi}_{ld}", me)
                self.col.objects.link(ob)
                mod = ob.modifiers.new("inst", "NODES")
                mod.node_group = _instance_group(f"inst_{vi}_{ld}", target)
                total += len(sel)
        total_u = self._scatter_under(cams)
        counts = np.bincount(lod, minlength=4)
        log("understorey instances", total_u)
        log("forest scatter %.1fs: %d instances, lods %s" % (time.time() - t0, total, counts.tolist()))
        return counts


UNDER_RANGE = {"meadow": (95.0, 300.0), "cobbles": (55.0, 230.0), "drift": (400.0, 700.0), "grass": (120.0, 500.0)}


def _under_scatter(self, cams, full=170.0, tail=650.0, margin_deg=18.0):
    Pu = np.stack([self.ux, self.uy, self.uz], 1).astype(np.float64)
    d_min = np.full(len(Pu), np.inf)
    vis = np.zeros(len(Pu), bool)
    for pos, fwd, half in cams:
        v = Pu - np.asarray(pos)
        d = np.linalg.norm(v, axis=1) + 1e-6
        d_min = np.minimum(d_min, d)
        vis |= ((v @ np.asarray(fwd)) / d > math.cos(half + math.radians(margin_deg))) | (d < 45.0)
    # thin with distance: at 600 m a tuft is sub-pixel. Meadow carpets and cobble clusters must stay complete near
    # the camera (a gap in a carpet is a hole in the meadow) and hand over to the ground texture further out.
    kinds = np.array([G.UNDER[int(k)][1] for k in range(len(G.UNDER))])[self.ukind]
    f_k = np.full(len(Pu), full)
    t_k = np.full(len(Pu), tail)
    for kind, (fk, tk) in UNDER_RANGE.items():
        f_k[kinds == kind], t_k[kinds == kind] = fk, tk
    rnd = (self.urot * 12345.678) % 1.0
    keep = vis & (d_min < t_k) & (rnd < np.clip((f_k / np.maximum(d_min, 1.0)) ** 1.7, 0, 1))
    total = 0
    for k in np.unique(self.ukind[keep]):
        sel = np.nonzero(keep & (self.ukind == k))[0]
        key = ("u", int(k))
        if key not in self.objects:
            name, kind, seed = G.UNDER[int(k)]
            ob = bpy.data.objects.new(f"lib_{name}", blib.build_under_mesh(name, kind, seed))
            self.lib.objects.link(ob)
            self.objects[key] = ob
        me = bpy.data.meshes.new(f"upts_{k}")
        me.vertices.add(len(sel))
        me.vertices.foreach_set("co", Pu[sel].astype(np.float32).ravel())
        me.attributes.new("rotz", "FLOAT", "POINT").data.foreach_set("value", self.urot[sel])
        me.attributes.new("scl", "FLOAT", "POINT").data.foreach_set("value", self.uscale[sel])
        me.update()
        ob = bpy.data.objects.new(f"upts_{k}", me)
        self.col.objects.link(ob)
        ob.modifiers.new("inst", "NODES").node_group = _instance_group(f"uinst_{k}", self.objects[key])
        total += len(sel)
    return total


Forest._scatter_under = _under_scatter


def add_haze(density=3e-5, color=(0.86, 0.90, 1.0), aniso=0.35, top=1300.0, half=50000.0):
    """Homogeneous valley air. Beam shadows from ridges are real because this is volumetric."""
    m, nt = new_mat("haze")
    v = N(nt, "ShaderNodeVolumeScatter", Density=density, Anisotropy=aniso)
    v.inputs["Color"].default_value = (*color, 1)
    L(nt, v.outputs[0], nt.nodes.new("ShaderNodeOutputMaterial").inputs["Volume"])
    verts = np.array([[-half, -half, -600], [half, -half, -600], [half, half, -600], [-half, half, -600],
                      [-half, -half, top], [half, -half, top], [half, half, top], [-half, half, top]], np.float32)
    faces = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    me = blib.make_mesh("haze", verts, faces, [m])
    ob = bpy.data.objects.new("haze", me)
    bpy.context.scene.collection.objects.link(ob)
    ob.visible_shadow = False
    return ob


def cam_tuple(cam_obj):
    from mathutils import Vector
    fwd = (cam_obj.rotation_quaternion @ Vector((0, 0, -1)))
    half = 0.5 * 2 * math.atan(cam_obj.data.sensor_width / (2 * cam_obj.data.lens))
    return (np.array(cam_obj.location), np.array(fwd), half)
