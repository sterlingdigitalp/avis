"""Place a bird on a flight log, every frame, with a mechanically posed wing. Imported by env/shots.py; usable alone.

load_flight(path)         read out/flight/<scenario>.json (SPEC format), convert to Blender axes, resample to the fps
corridor_flight(speed)    the same structure along data/env/corridor.json, for layout before the physics log exists
make_proxy()              gull-sized placeholder: three spanwise segments a side, driven like the real wing
import_glb(path)          real bird from the film workstream; node names below select the moving parts
apply_frame(rig, flight, i)  pose one frame

Axis conversion (SPEC): world (x, y, z) -> Blender (x, -z, y). A body-to-world quaternion [x, y, z, w] becomes
Blender (w, x, -z, y). The GLB importer maps its model axes the same way, so a model built in the body frame
(x forward, y up, z right) lands correctly with this rotation applied.

Wing pose follows src/physics/kinematics.js conventions: phase 0 is top of stroke, phi = mean + amp cos(phase), raising the
tip, rotation about body x at the shoulder; outer segments add gain*(phi - phi_ref); twist theta > 0 is nose up on either
wing. A flight log carries phase, amplitude, twist_l and twist_r, so the pose is a pure function of the log.
GLB node convention: wing_R_0..n and wing_L_0..n, each with its origin on the joint, parented in chain; body node is the root.
A design with a crank-rocker law should pass its phi table in `phi_of` so this reuses the linkage output and not a sine.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
import numpy as np
from mathutils import Euler, Matrix, Quaternion, Vector

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def world_to_blender_pos(p):
    p = np.asarray(p, float)
    return np.stack([p[..., 0], -p[..., 2], p[..., 1]], -1)


def blender_quat_from_log(q):
    """Log [x, y, z, w] -> Blender (w, x, y, z) in Blender axes."""
    q = np.asarray(q, float)
    return np.stack([q[..., 3], q[..., 0], -q[..., 2], q[..., 1]], -1)


def _resample(t, y, tq):
    y = np.asarray(y, float)
    if y.ndim == 1:
        return np.interp(tq, t, y)
    return np.stack([np.interp(tq, t, y[:, i]) for i in range(y.shape[1])], 1)


def load_flight(path, fps=30.0, t_start=0.0, t_end=None):
    d = json.load(open(path))
    c = d["columns"]
    t = np.asarray(c["t"], float)
    t1 = t[-1] if t_end is None else min(t_end, t[-1])
    tq = np.arange(max(t_start, t[0]), t1, 1.0 / fps)
    pos = world_to_blender_pos(_resample(t, c["pos"], tq))
    # Nearest-neighbour hemisphere fix, then normalised lerp: at 120 Hz this is indistinguishable from slerp.
    q = np.asarray(c["quat"], float).copy()
    dots = np.einsum("ij,ij->i", q[:-1], q[1:])
    flips = np.cumprod(np.where(dots < 0, -1.0, 1.0))
    q[1:] *= flips[:, None]
    q = _resample(t, q, tq)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    out = {"design": d.get("design"), "scenario": d.get("scenario"), "fps": fps, "t": tq, "pos": pos,
           "quat": blender_quat_from_log(q), "vel": world_to_blender_pos(_resample(t, c["vel"], tq))}
    # Phase wraps: unwrap before interpolating.
    out["phase"] = np.interp(tq, t, np.unwrap(np.asarray(c["phase"], float)))
    for k in ("amplitude", "twist_l", "twist_r", "tail"):
        out[k] = _resample(t, c[k], tq) if k in c and len(c[k]) else np.zeros_like(tq)
    return out


def corridor_points(n_per_m=1.0):
    c = json.load(open(os.path.join(ROOT, "data/env/corridor.json")))
    w = world_to_blender_pos(np.array(c["waypoints"]))
    seg = np.linalg.norm(np.diff(w, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    return w, s, c


def corridor_flight(speed=12.0, fps=30.0, flap_hz=3.0, amplitude=math.radians(28.0)):
    """Layout stand-in only: constant speed along the corridor with a nominal flap. Never presented as a flight result."""
    w, s, _ = corridor_points()
    tq = np.arange(0, s[-1] / speed, 1.0 / fps)
    d = tq * speed
    pos = np.stack([np.interp(d, s, w[:, i]) for i in range(3)], 1)
    vel = np.gradient(pos, 1.0 / fps, axis=0)
    vel = np.stack([np.convolve(vel[:, i], np.ones(31) / 31, "same") for i in range(3)], 1)
    vel[:15] = vel[15]
    vel[-15:] = vel[-16]
    quats = []
    for v, acc in zip(vel, np.gradient(vel, 1.0 / fps, axis=0)):
        # body x forward, y up, z right; banked into the turn. In Blender axes body (x, y, z) maps to (x, -z, y).
        f = Vector(v).normalized()
        up0 = Vector((0, 0, 1))
        right = f.cross(up0).normalized()
        lat = right.dot(Vector(acc))
        bank = math.atan2(lat, 9.81)
        up = (up0 - f * up0.dot(f)).normalized()
        up = (up * math.cos(bank) + right * math.sin(bank)).normalized()
        right = f.cross(up).normalized()
        # model-space axes after import: X forward, -Y right (body z), Z up
        m = Matrix((f, -right, up)).transposed().to_quaternion()
        quats.append((m.w, m.x, m.y, m.z))
    return {"design": "layout-proxy", "scenario": "corridor-layout", "fps": fps, "t": tq, "pos": pos, "quat": np.array(quats), "vel": vel,
            "phase": 2 * math.pi * flap_hz * tq, "amplitude": np.full_like(tq, amplitude),
            "twist_l": np.radians(10.0) * np.sin(2 * math.pi * flap_hz * tq), "twist_r": np.radians(10.0) * np.sin(2 * math.pi * flap_hz * tq),
            "tail": np.zeros_like(tq)}


def _proxy_mat(name, color, metallic, roughness):
    m = bpy.data.materials.get(name)
    if m:
        return m
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    p.inputs["Base Color"].default_value = (*color, 1)
    p.inputs["Metallic"].default_value = metallic
    p.inputs["Roughness"].default_value = roughness
    return m


def _metal_mat():
    return _proxy_mat("proxy_metal", (0.42, 0.44, 0.47), 0.65, 0.42)


def _film_mat():
    return _proxy_mat("proxy_film", (0.82, 0.84, 0.86), 0.0, 0.3)


def _box_mesh(name, x0, x1, y0, y1, z0, z1, taper=(1.0, 1.0)):
    v = [(x0, y0 * taper[0], z0 * taper[0]), (x1, y0 * taper[1], z0 * taper[1]), (x1, y1 * taper[1], z0 * taper[1]), (x0, y1 * taper[0], z0 * taper[0]),
         (x0, y0 * taper[0], z1 * taper[0]), (x1, y0 * taper[1], z1 * taper[1]), (x1, y1 * taper[1], z1 * taper[1]), (x0, y1 * taper[0], z1 * taper[0])]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    me.update()
    return me


def _wing_seg(name, length, c0, c1, side):
    """Thin ribbed panel in model space: X forward, Y left (+), Z up. Origin on the joint at the spar."""
    y = np.linspace(0, length, 5) * side
    chord = np.linspace(c0, c1, 5)
    v = []
    for yi, c in zip(y, chord):
        v += [(0.25 * c, yi, 0.006), (-0.75 * c, yi, 0.003), (0.25 * c, yi, -0.006), (-0.75 * c, yi, -0.003)]
    f = []
    for i in range(4):
        a = i * 4
        b = a + 4
        f += [(a, a + 1, b + 1, b), (a + 2, b + 2, b + 3, a + 3), (a, b, b + 2, a + 2), (a + 1, a + 3, b + 3, b + 1)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    me.update()
    me.materials.append(_film_mat())
    return me


def make_proxy(span=1.8, body_len=0.52):
    """Gull-sized mechanical placeholder. Clearly a proxy: the real bird is the film workstream's GLB."""
    col = bpy.data.collections.new("bird")
    bpy.context.scene.collection.children.link(col)
    body = bpy.data.objects.new("bird_body", _box_mesh("bird_body", -0.34, 0.20, -0.045, 0.045, -0.05, 0.05, (0.55, 0.7)))
    body.data.materials.append(_metal_mat())
    col.objects.link(body)
    tail = bpy.data.objects.new("bird_tail", _box_mesh("bird_tail", -0.52, -0.32, -0.09, 0.09, -0.004, 0.004, (1.0, 0.4)))
    tail.data.materials.append(_film_mat())
    col.objects.link(tail)
    tail.parent = body
    rig = {"body": body, "right": [], "left": [], "gain": [0.0, 0.35, 0.8], "phi_ref": 0.0}
    half = span / 2 - 0.04
    lens = [half * 0.38, half * 0.34, half * 0.28]
    chords = [(0.26, 0.24), (0.24, 0.19), (0.19, 0.06)]
    for label, side, store in (("R", -1, rig["right"]), ("L", 1, rig["left"])):
        parent = body
        offset = (0.03, side * 0.045, 0.03)
        for k in range(3):
            ob = bpy.data.objects.new(f"wing_{label}_{k}", _wing_seg(f"wing_{label}_{k}", lens[k], chords[k][0], chords[k][1], side))
            col.objects.link(ob)
            ob.parent = parent
            ob.location = offset
            ob.rotation_mode = "XYZ"
            store.append(ob)
            parent = ob
            offset = (0, side * lens[k], 0)
    body.rotation_mode = "QUATERNION"
    return rig


def import_glb(path):
    """Import the real bird. Expects nodes wing_R_* / wing_L_* (see the module docstring)."""
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    new = [o for o in bpy.data.objects if o not in before]
    roots = [o for o in new if o.parent is None]
    body = roots[0]
    body.rotation_mode = "QUATERNION"
    rig = {"body": body, "right": [], "left": [], "gain": [], "phi_ref": 0.0}
    for side, key in (("R", "right"), ("L", "left")):
        rig[key] = sorted([o for o in new if o.name.startswith(f"wing_{side}_")], key=lambda o: o.name)
        for o in rig[key]:
            o.rotation_mode = "XYZ"
    rig["gain"] = [0.0] * len(rig["right"])
    return rig


def stroke_phi(phase, amp, mean=0.0):
    """phi = mean + amp cos(phase): the sine law in src/physics/kinematics.js strokeAngle(). Phase 0 is top of stroke."""
    return mean + amp * math.cos(phase)


def apply_frame(rig, flight, i, phi_of=None):
    body = rig["body"]
    body.location = tuple(flight["pos"][i])
    body.rotation_quaternion = tuple(flight["quat"][i])
    phase, amp = float(flight["phase"][i]), float(flight["amplitude"][i])
    phi = phi_of(phase, amp) if phi_of else stroke_phi(phase, amp)
    for segs, twist, side in ((rig["right"], float(flight["twist_r"][i]), -1), (rig["left"], float(flight["twist_l"][i]), 1)):
        prev = 0.0
        for k, ob in enumerate(segs):
            # absolute segment angle from kinematics.segmentAngles: phi + gain*(phi - phi_ref). Chain is parented, so store relative.
            abs_ang = phi + rig["gain"][k] * (phi - rig["phi_ref"]) if rig["gain"] else phi
            rel = abs_ang - prev
            prev = abs_ang
            # Phi raises the tip. Model axes: right wing along -Y; rotate about X. Twist nose up about the spar axis (Y).
            ob.rotation_euler = Euler((side * rel, -twist * (k + 1) / len(segs) if side > 0 else twist * (k + 1) / len(segs), 0.0), "XYZ")


def key_all(rig, flight, frame_start=1, phi_of=None):
    """Bake the log to keyframes so saved .blend files play back with no handler. Generated, never hand-keyed."""
    objs = [rig["body"]] + rig["right"] + rig["left"]
    n = len(flight["t"])
    for i in range(n):
        apply_frame(rig, flight, i, phi_of)
        f = frame_start + i
        rig["body"].keyframe_insert("location", frame=f)
        rig["body"].keyframe_insert("rotation_quaternion", frame=f)
        for ob in objs[1:]:
            ob.keyframe_insert("rotation_euler", frame=f)
