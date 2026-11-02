"""Axis-conversion checks for env/import_flight.py against the SPEC definitions. No GPU, no render.

    Blender --background --factory-startup --python-exit-code 1 --python env/test_import_flight.py
"""
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
import numpy as np
from mathutils import Quaternion, Vector

import import_flight as IF

fails = []


def check(name, cond, info=""):
    print(("PASS " if cond else "FAIL ") + name, info, flush=True)
    if not cond:
        fails.append(name)


# SPEC: world (x, y, z) -> Blender (x, -z, y)
p = IF.world_to_blender_pos(np.array([10.0, 5.0, -20.0]))
check("position axes", np.allclose(p, [10.0, 20.0, 5.0]), p)

# identity attitude: body x forward = world +x (east) -> Blender +X; body y up -> +Z; body z right = world +z (south) -> Blender -Y
q = Quaternion(IF.blender_quat_from_log(np.array([0.0, 0.0, 0.0, 1.0])))
check("identity forward", (q @ Vector((1, 0, 0)) - Vector((1, 0, 0))).length < 1e-9)

# Yaw +90 deg about world up (y). In the world frame this takes +x to -z, which is north; in Blender that is +Y.
s = math.sin(math.radians(45))
q = Quaternion(IF.blender_quat_from_log(np.array([0.0, s, 0.0, s])))
check("yaw left 90 faces north", (q @ Vector((1, 0, 0)) - Vector((0, 1, 0))).length < 1e-6, q @ Vector((1, 0, 0)))

# Roll +30 deg about body x (right-handed): body up (0, 1, 0) goes to (0, cos, sin) in world = toward +z (south, the right side
# when facing east). A right-handed roll about +x lowers... check against the explicit world rotation instead of intuition.
a = math.radians(30)
qw = np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)])
up_world = np.array([0.0, math.cos(a), math.sin(a)])
q = Quaternion(IF.blender_quat_from_log(qw))
got = q @ Vector((0, 0, 1))
want = Vector(IF.world_to_blender_pos(up_world))
check("roll maps body up consistently", (got - want).length < 1e-6, (tuple(got), tuple(want)))

# A log round trip through load_flight: resampling, hemisphere fix and unwrapped phase.
t = np.arange(0, 2.0, 1 / 120)
quat = np.tile([0.0, 0.0, 0.0, 1.0], (len(t), 1))
quat[len(t) // 2:] *= -1  # same rotation, opposite hemisphere: must not cause a spin when interpolated
log = {"design": "test", "scenario": "axis", "dt": 1 / 120, "columns": {
    "t": t.tolist(), "pos": np.stack([12 * t, np.full_like(t, 30.0), -3 * t], 1).tolist(), "quat": quat.tolist(),
    "vel": np.tile([12.0, 0.0, -3.0], (len(t), 1)).tolist(), "phase": ((2 * math.pi * 3 * t) % (2 * math.pi)).tolist(),
    "amplitude": np.full_like(t, 0.5).tolist(), "twist_l": np.zeros_like(t).tolist(), "twist_r": np.zeros_like(t).tolist(),
    "tail": np.zeros_like(t).tolist(), "airspeed": np.full_like(t, 12.4).tolist(), "power_w": np.zeros_like(t).tolist(),
    "battery_pct": np.zeros_like(t).tolist(), "wind": np.zeros((len(t), 3)).tolist()}}
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "axis.json")
    json.dump(log, open(path, "w"))
    fl = IF.load_flight(path, fps=30)
check("frame count at 30 fps", len(fl["t"]) == 60, len(fl["t"]))
check("blender position at t=1", np.allclose(fl["pos"][30], [12.0, 3.0, 30.0], atol=1e-6), fl["pos"][30])
check("no hemisphere spin", all(abs(abs(qq[0]) - 1) < 1e-6 for qq in fl["quat"]))
check("phase unwrapped and monotonic", np.all(np.diff(fl["phase"]) > 0))

# Wing pose: phase 0 is the top of the stroke, so the left tip is higher at phase 0 than at phase pi.
rig = IF.make_proxy()
fl["phase"][:] = 0.0
IF.apply_frame(rig, fl, 0)
bpy.context.view_layer.update()
top = (rig["left"][-1].matrix_world @ Vector((0, 0.3, 0))).z
fl["phase"][:] = math.pi
IF.apply_frame(rig, fl, 0)
bpy.context.view_layer.update()
bottom = (rig["left"][-1].matrix_world @ Vector((0, 0.3, 0))).z
check("stroke: phase 0 is top", top > bottom + 0.2, (top, bottom))
r_top = None
fl["phase"][:] = 0.0
IF.apply_frame(rig, fl, 0)
bpy.context.view_layer.update()
r_top = (rig["right"][-1].matrix_world @ Vector((0, -0.3, 0))).z
check("stroke symmetric left/right", abs(r_top - top) < 1e-4, (r_top, top))

print("RESULT", "FAIL" if fails else "PASS", fails, flush=True)
sys.exit(1 if fails else 0)
