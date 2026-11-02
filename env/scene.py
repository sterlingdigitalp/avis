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
    canopy = tex(nt, f"{TER}/canopy.png", uvm.outputs[0], "Non-Color", ext="EXTEND", interp="Linear").outputs[0]
    # slope
    nz = nt.nodes.new("ShaderNodeSeparateXYZ")
    L(nt, geo.outputs["Normal"], nz.inputs[0])
    steep = maprange(nt, nz.outputs[2], 0.93, 0.62)

    # fine noise used to break map texels up near the camera
    no = N(nt, "ShaderNodeTexNoise", Scale=0.028, Detail=6.0, Roughness=0.6)
    L(nt, pos, no.inputs["Vector"])
    no2 = N(nt, "ShaderNodeTexNoise", Scale=0.33, Detail=5.0, Roughness=0.55)
    L(nt, pos, no2.inputs["Vector"])

    # --- rock ---
    rock_near = _detail(nt, pos, f"{blib.PH}/cliff_side/diff.jpg", 14.0)
    bw_near = N(nt, "ShaderNodeRGBToBW")
    L(nt, rock_near, bw_near.inputs[0])
    det_near = math_node(nt, "DIVIDE", bw_near.outputs[0], 0.27)
    f_near = maprange(nt, D, 250.0, 1800.0, 1.0, 0.0)
    f_mid = maprange(nt, D, 1500.0, 16000.0, 1.0, 0.25)
    near = mix_float(nt, f_near, 1.0, det_near)
    # Mid-range rock has no image texture, so it cannot tile. Noise in bedding-aligned coordinates: thin across the
    # beds, long along them, which is how tilted limestone actually weathers.
    strata_p = N(nt, "ShaderNodeVectorRotate", p_rotation_type="AXIS_ANGLE", Angle=math.radians(45.0))
    L(nt, pos, strata_p.inputs["Vector"])
    strata_p.inputs["Axis"].default_value = (0.7071, -0.7071, 0.0)
    strata_m = N(nt, "ShaderNodeMapping")
    L(nt, strata_p.outputs[0], strata_m.inputs["Vector"])
    strata_m.inputs["Scale"].default_value = (1 / 44.0, 1 / 44.0, 1 / 10.0)
    n_s1 = N(nt, "ShaderNodeTexNoise", Scale=1.0, Detail=8.0, Roughness=0.66, Lacunarity=2.4)
    L(nt, strata_m.outputs[0], n_s1.inputs["Vector"])
    # blocky jointing: limestone breaks along joints, the dark lines that make a cliff read as rock
    joint = nt.nodes.new("ShaderNodeTexVoronoi")
    joint.feature = "DISTANCE_TO_EDGE"
    joint.inputs["Scale"].default_value = 1.0
    jm = N(nt, "ShaderNodeMapping")
    L(nt, strata_p.outputs[0], jm.inputs["Vector"])
    jm.inputs["Scale"].default_value = (1 / 26.0, 1 / 26.0, 1 / 16.0)
    L(nt, jm.outputs[0], joint.inputs["Vector"])
    joints = maprange(nt, joint.outputs["Distance"], 0.0, 0.07, 0.82, 1.0)
    mid = maprange(nt, n_s1.outputs[0], 0.34, 0.66, 0.55, 1.25)
    mid = math_node(nt, "MULTIPLY", mid, mix_float(nt, maprange(nt, D, 1200.0, 4000.0, 1.0, 0.0), 1.0, joints))
    mid = mix_float(nt, f_mid, 1.0, mid)
    rock_det = math_node(nt, "MULTIPLY", near, mid)
    # bedding: thin tilted strata, visible as banding on the walls across the valley
    bed = N(nt, "ShaderNodeVectorMath", p_operation="DOT_PRODUCT")
    L(nt, pos, bed.inputs[0])
    bed.inputs[1].default_value = (-0.5, -0.5, 0.707)
    war = N(nt, "ShaderNodeTexNoise", Scale=0.004, Detail=4.0, Roughness=0.5)
    L(nt, pos, war.inputs["Vector"])
    warp = math_node(nt, "MULTIPLY", math_node(nt, "SUBTRACT", war.outputs[0], 0.5), 90.0)
    bed_coord = math_node(nt, "ADD", nt.nodes[-3].outputs["Value"] if False else bed.outputs["Value"], warp)
    bandc = nt.nodes.new("ShaderNodeCombineXYZ")
    L(nt, math_node(nt, "DIVIDE", bed_coord, 34.0), bandc.inputs[0])
    band_noise = N(nt, "ShaderNodeTexNoise", Scale=1.0, Detail=5.0, Roughness=0.55, p_noise_dimensions="1D") if False else None
    band_noise = nt.nodes.new("ShaderNodeTexNoise")
    band_noise.noise_dimensions = "1D"
    band_noise.inputs["Detail"].default_value = 6.0
    band_noise.inputs["Roughness"].default_value = 0.6
    band_noise.inputs["Scale"].default_value = 1.0
    L(nt, bandc.outputs[0], band_noise.inputs["W"])
    # Grey cliff bands and tan recessive beds. These two colours are artistic, guided by the formation notes.
    bands = maprange(nt, band_noise.outputs[0], 0.28, 0.72, 0.42, 0.86, smooth=True)
    tint = mix_rgb(nt, maprange(nt, band_noise.outputs[0], 0.35, 0.65), rgba(0.74, 0.75, 0.78), rgba(0.90, 0.82, 0.70))
    rock_col = mix_rgb(nt, 1.0, alb.outputs[0], tint, "MULTIPLY")
    # Weathering streaks: dark water and lichen stains run down steep faces, so stretch noise along world Z.
    st_m = N(nt, "ShaderNodeMapping")
    L(nt, pos, st_m.inputs["Vector"])
    st_m.inputs["Scale"].default_value = (1 / 9.0, 1 / 9.0, 1 / 85.0)
    st_n = N(nt, "ShaderNodeTexNoise", Scale=1.0, Detail=6.0, Roughness=0.6)
    L(nt, st_m.outputs[0], st_n.inputs["Vector"])
    streak = mix_float(nt, steep, 1.0, maprange(nt, st_n.outputs[0], 0.38, 0.62, 0.55, 1.12))
    gain = math_node(nt, "MULTIPLY", math_node(nt, "MULTIPLY", rock_det, bands), math_node(nt, "MULTIPLY", streak, 0.78))
    g3 = nt.nodes.new("ShaderNodeCombineXYZ")
    for i in range(3):
        L(nt, gain, g3.inputs[i])
    rock_mul = N(nt, "ShaderNodeVectorMath", p_operation="MULTIPLY")
    L(nt, rock_col, rock_mul.inputs[0])
    L(nt, g3.outputs[0], rock_mul.inputs[1])
    rock = rock_mul.outputs[0]

    # --- forest floor and meadow, near range only ---
    floor = _detail(nt, pos, f"{blib.PH}/forest_ground_04/diff.jpg", 3.2)
    gravel = _detail(nt, pos, f"{blib.PH}/gravel_stones/diff.jpg", 2.2)
    meadow_tex = N(nt, "ShaderNodeBrightContrast", Bright=0.0, Contrast=0.0)
    # meadow colour from the map, broken with noise so the 10 m texels vanish
    # meadow: an aerial grass texture carries the cured grass and small stones between the instanced tufts
    mg = _detail(nt, pos, f"{blib.PH}/aerial_grass_rock/diff.jpg", 9.0)
    mg_bw = N(nt, "ShaderNodeRGBToBW")
    L(nt, mg, mg_bw.inputs[0])
    mcol = mix_rgb(nt, 0.6, alb.outputs[0], rgba(0.115, 0.095, 0.04))
    mcol = mix_rgb(nt, maprange(nt, D, 60.0, 900.0, 0.85, 0.0), mcol, mix_rgb(nt, 1.0, mcol, mg, "OVERLAY"))
    mscale = maprange(nt, no2.outputs[0], 0.3, 0.7, 0.55, 1.4)
    g4 = nt.nodes.new("ShaderNodeCombineXYZ")
    for i in range(3):
        L(nt, mscale, g4.inputs[i])
    meadow = N(nt, "ShaderNodeVectorMath", p_operation="MULTIPLY")
    L(nt, mcol, meadow.inputs[0])
    L(nt, g4.outputs[0], meadow.inputs[1])
    # shade forest floor by its canopy: darker where canopy is dense
    vg = maprange(nt, veg, 0.25, 0.8, 1.0, 0.45)
    fl3 = nt.nodes.new("ShaderNodeCombineXYZ")
    for i in range(3):
        L(nt, vg, fl3.inputs[i])
    floor_s = N(nt, "ShaderNodeVectorMath", p_operation="MULTIPLY")
    L(nt, mix_rgb(nt, 1.0, floor, rgba(0.26, 0.24, 0.17), "MULTIPLY"), floor_s.inputs[0])
    L(nt, fl3.outputs[0], floor_s.inputs[1])
    # canopy-covered ground far from any instanced tree stays map coloured (dark forest).
    r_len = N(nt, "ShaderNodeVectorMath", p_operation="LENGTH")
    L(nt, pos, r_len.inputs[0])
    in_forest = maprange(nt, r_len.outputs["Value"], 11500.0, 13000.0, 1.0, 0.0)
    near_ground = maprange(nt, D, 350.0, 1100.0, 1.0, 0.0)
    veg_hi = maprange(nt, canopy, 0.30, 0.65)
    # ground under stems: blend floor near the camera, darkened map further out
    dark_map = mix_rgb(nt, 1.0, alb.outputs[0], rgba(0.5, 0.5, 0.5), "MULTIPLY")
    under = mix_rgb(nt, near_ground, dark_map, floor_s.outputs[0])
    veg_col = mix_rgb(nt, math_node(nt, "MULTIPLY", veg_hi, in_forest), meadow.outputs[0], under)
    # low vegetation fades to rock with steepness
    vm = math_node(nt, "MULTIPLY", maprange(nt, veg, 0.10, 0.30), maprange(nt, nz.outputs[2], 0.55, 0.75))
    base = mix_rgb(nt, vm, rock, veg_col)
    # gravel bars and low rock near the camera
    gb = math_node(nt, "MULTIPLY", maprange(nt, D, 400.0, 1600.0, 1.0, 0.0), math_node(nt, "SUBTRACT", 1.0, vm))
    gb = math_node(nt, "MULTIPLY", gb, maprange(nt, nz.outputs[2], 0.85, 0.97))
    gravel_col = mix_rgb(nt, 0.35, mix_rgb(nt, 1.0, gravel, rgba(1.25, 1.2, 1.12), "MULTIPLY"), alb.outputs[0])
    base = mix_rgb(nt, math_node(nt, "MULTIPLY", gb, 0.85), base, gravel_col)
    # snow
    sn = maprange(nt, snow, 0.35, 0.65)
    sn_tex = maprange(nt, no2.outputs[0], 0.35, 0.65, 0.85, 1.0)
    sn_col = mix_rgb(nt, sn_tex, rgba(0.62, 0.68, 0.78), rgba(0.86, 0.88, 0.92))
    base = mix_rgb(nt, sn, base, sn_col)

    p = N(nt, "ShaderNodeBsdfPrincipled")
    L(nt, base, p.inputs["Base Color"])
    rough = mix_float(nt, sn, 0.82, 0.55)
    L(nt, rough, p.inputs["Roughness"])
    p.inputs["Specular IOR Level"].default_value = 0.35

    # relief below the mesh resolution: rock texture displacement and bedding
    h_near = _detail(nt, pos, f"{blib.PH}/cliff_side/disp.jpg", 14.0, colorspace="Non-Color")
    rockiness = math_node(nt, "MULTIPLY", math_node(nt, "SUBTRACT", 1.0, vm), math_node(nt, "SUBTRACT", 1.0, sn))
    b1 = N(nt, "ShaderNodeBump", Strength=1.0, Distance=1.5)
    L(nt, h_near, b1.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", rockiness, f_near), b1.inputs["Strength"])
    b2 = N(nt, "ShaderNodeBump", Strength=1.0, Distance=9.0)
    L(nt, n_s1.outputs[0], b2.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", rockiness, maprange(nt, D, 800.0, 20000.0, 1.0, 0.5)), b2.inputs["Strength"])
    L(nt, b1.outputs[0], b2.inputs["Normal"])
    b3 = N(nt, "ShaderNodeBump", Strength=1.0, Distance=9.0)
    L(nt, band_noise.outputs[0], b3.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", rockiness, steep), b3.inputs["Strength"])
    L(nt, b2.outputs[0], b3.inputs["Normal"])
    fl_h = _detail(nt, pos, f"{blib.PH}/forest_ground_04/disp.jpg", 3.2, colorspace="Non-Color")
    b4 = N(nt, "ShaderNodeBump", Strength=0.0, Distance=0.14)
    L(nt, fl_h, b4.inputs["Height"])
    L(nt, math_node(nt, "MULTIPLY", vm, maprange(nt, D, 30.0, 300.0, 1.0, 0.0)), b4.inputs["Strength"])
    L(nt, b3.outputs[0], b4.inputs["Normal"])
    L(nt, b4.outputs[0], p.inputs["Normal"])

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
    # from the 10 m map: keep the real satellite tint, push it toward measured glacial turquoise
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


def _under_scatter(self, cams, full=170.0, tail=650.0, margin_deg=18.0):
    Pu = np.stack([self.ux, self.uy, self.uz], 1).astype(np.float64)
    d_min = np.full(len(Pu), np.inf)
    vis = np.zeros(len(Pu), bool)
    for pos, fwd, half in cams:
        v = Pu - np.asarray(pos)
        d = np.linalg.norm(v, axis=1) + 1e-6
        d_min = np.minimum(d_min, d)
        vis |= ((v @ np.asarray(fwd)) / d > math.cos(half + math.radians(margin_deg))) | (d < 45.0)
    # thin with distance: at 600 m a tuft is sub-pixel
    rnd = (self.urot * 12345.678) % 1.0
    keep = vis & (d_min < tail) & (rnd < np.clip((full / np.maximum(d_min, 1.0)) ** 1.7, 0, 1))
    total = 0
    for k in np.unique(self.ukind[keep]):
        sel = np.nonzero(keep & (self.ukind == k))[0]
        key = ("u", int(k))
        if key not in self.objects:
            name, kind, seed = T.UNDER[int(k)]
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
