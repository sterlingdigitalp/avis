"""Camera rigs: every camera is a pure function of a flight log, the shot's span and the terrain. No Blender.

A rig is `rig(fl, surface, span) -> dict(frames=(a, b), cam=(N, 3), target=(N, 3), lens=mm)` where `fl` is a
flight from import_flight.load_flight (Blender axes, resampled at fl["fps"]), `surface` a Surface, and `span` the
whole shot as frame indices into `fl` (b exclusive). `cam` and `target` cover the whole log, so any hold inside the
span reads the same camera; locked-off cameras anchor on the bird's smoothed path inside the span, never on a
fixed time or world position. Distances are metres, lenses millimetres on a 36 mm wide sensor (Blender's default,
sensor fit AUTO at 16:9), the camera looks from `cam` at `target` with world Z up and no roll.
"""
import json
import math
import os
import struct

import numpy as np

import geo

ROOT = geo.ROOT
SENSOR_W = 36.0
ASPECT = 1080.0 / 1920.0


def smooth(a, win):
    """Moving average with edge hold, so cameras glide over the wingbeat and do not shake with the body."""
    if win <= 1:
        return a
    k = np.ones(win) / win
    pad = win // 2
    if a.ndim == 1:
        return np.convolve(np.pad(a, pad, mode="edge"), k, "valid")[: len(a)]
    return np.stack([smooth(a[:, i], win) for i in range(a.shape[1])], 1)


def frames_per(fl, seconds):
    """Odd window of about `seconds` at the log's sample rate: an even window would shift smooth() by half a sample."""
    return 2 * int(round(seconds * fl.get("fps", 30.0) / 2)) + 1


class Surface:
    """Ground + lidar canopy in Blender axes, read from data/env/terrain_height.bin with env/terrain.js's format."""

    def __init__(self):
        with open(os.path.join(ROOT, "data/env/terrain_height.bin"), "rb") as fh:
            raw = fh.read()
        _, _, off, scale, nx, nz, x0, z0, dx, fx, fz, fx0, fz0, fdx = struct.unpack("<4sIffIIfffIIfff", raw[:56])
        self.off, self.scale = off, scale
        n = nx * nz
        self.near = (np.frombuffer(raw, "<u2", n, 56).reshape(nz, nx), x0, z0, dx)
        self.can = np.frombuffer(raw, np.uint8, n, 56 + 2 * n).reshape(nz, nx)
        self.far = (np.frombuffer(raw, "<u2", fx * fz, 56 + 3 * n).reshape(fz, fx), fx0, fz0, fdx)

    def _grid(self, grid, x, z, data=None):
        h, x0, z0, dx = grid
        d = h if data is None else data
        c = np.clip((x - x0) / dx, 0, d.shape[1] - 1.001)
        r = np.clip((z - z0) / dx, 0, d.shape[0] - 1.001)
        c0, r0 = c.astype(int), r.astype(int)
        fc, fr = c - c0, r - r0
        return d[r0, c0] * (1 - fc) * (1 - fr) + d[r0, c0 + 1] * fc * (1 - fr) + d[r0 + 1, c0] * (1 - fc) * fr + d[r0 + 1, c0 + 1] * fc * fr

    def top(self, bx, by):
        """Canopy top height at Blender (x, y)."""
        z = -np.asarray(by, float)
        x = np.asarray(bx, float)
        g = self._grid(self.near, x, z) * self.scale + self.off
        c = self._grid(self.near, x, z, self.can) * 0.25
        return g + c


# --- frames and camera geometry ---

def frames_of(fl):
    """Path frame per sample: forward along the smoothed velocity, right horizontal, up completing the triad."""
    v = smooth(fl["vel"], 45)
    f = v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-3)
    right = np.stack([f[:, 1], -f[:, 0], np.zeros(len(f))], 1)
    right /= np.maximum(np.linalg.norm(right, axis=1, keepdims=True), 1e-3)
    up = np.cross(right, f)
    return f, right, up


def chord_frame(p, a, b):
    """Horizontal forward and right of the path's chord over the span, and world up."""
    d = p[b - 1] - p[a]
    f = np.array([d[0], d[1], 0.0])
    f /= max(np.linalg.norm(f), 1e-6)
    return f, np.array([f[1], -f[0], 0.0]), np.array([0.0, 0.0, 1.0])


def sun_side(fl, right):
    sun, _, _ = geo.sun_vector()
    return np.sign(right[len(right) // 2] @ sun) or 1.0


def lift_clear(cam, surface, margin=3.0):
    t = surface.top(cam[:, 0], cam[:, 1]) + margin
    out = cam.copy()
    out[:, 2] = np.maximum(out[:, 2], smooth(t, 31))
    return out


def clear_sight(cam, targets, surface, margin=2.0, near=4.0):
    """Lift a fixed camera just enough that its line of sight to every target clears the lidar canopy by `margin`.
    Samples each sight line from `near` metres out to 85% of the way to the target."""
    c = np.array(cam, float)
    need = c[2]
    for t in targets[:: max(1, len(targets) // 24)]:
        d = np.linalg.norm(t[:2] - c[:2])
        if d <= near:
            continue
        s = np.linspace(near / d, 0.85, 40)
        xy = c[None, :2] + s[:, None] * (t[None, :2] - c[None, :2])
        top = surface.top(xy[:, 0], xy[:, 1]) + margin
        need = max(need, float(np.max((top - s * t[2]) / (1 - s))))
    c[2] = need
    return c


def default_window(fl, t0, t1):
    """Frame range for fractions (<= 1) or seconds of the log: the shot's span when the caller gives none."""
    t = fl["t"]
    span = t[-1]
    a = np.searchsorted(t, t0 * span if t0 <= 1 else t0)
    b = np.searchsorted(t, t1 * span if t1 <= 1 else t1)
    return a, max(b, a + 2)


def look_basis(cam, tgt):
    """Camera forward, right and up for Blender's to_track_quat('-Z', 'Y'): no roll, image up toward world Z."""
    fwd = np.asarray(tgt, float) - np.asarray(cam, float)
    fwd = fwd / np.linalg.norm(fwd, axis=-1, keepdims=True)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right = right / np.maximum(np.linalg.norm(right, axis=-1, keepdims=True), 1e-9)
    return fwd, right, np.cross(right, fwd)


def project(cam, tgt, lens, pts):
    """Pinhole projection of world points to image coordinates normalised so the frame edges are +-1 on both axes,
    plus depth along the view axis (negative = behind the camera) as the third column. One camera for many points,
    or one camera, target and point per row."""
    fwd, right, up = look_basis(cam, tgt)
    d = np.asarray(pts, float) - np.asarray(cam, float)
    z = (d * fwd).sum(-1)
    k = lens / (SENSOR_W / 2)
    zs = np.where(np.abs(z) < 1e-9, 1e-9, z)
    return np.stack([(d * right).sum(-1) / zs * k, (d * up).sum(-1) / zs * k / ASPECT, z], -1)


# --- the bird's extent, from the design file ---

def load_design(name):
    with open(os.path.join(ROOT, "data/designs/%s.json" % name)) as fh:
        return json.load(fh)


def bird_points(design, phi):
    """Extremities of the bird in the body frame (x forward, y up, z right) at stroke angle phi (rad, raising the tip):
    nose, tail end, tail tips, and leading and trailing edge at every wing joint on both sides. Outer segments add
    gain * (phi - phi_ref) as in src/physics/kinematics.js."""
    w = design["wing"]
    hx, hy, hz = w["hinge"]
    secs = design["body"]["loft"]["sections"]
    tail = design["tail"]
    xs = [s["x_m"] for s in secs]
    t_te = tail["x_m"] - tail["chord_root_m"]
    ty = tail["y_m"] + tail["span_each_m"] * math.sin(tail["dihedral_rad"])
    tz = tail["span_each_m"] * math.cos(tail["dihedral_rad"])
    pts = [(max(xs), 0.0, 0.0), (min(min(xs), t_te), 0.0, 0.0), (t_te, ty, tz), (t_te, ty, -tz)]
    for side in (1.0, -1.0):
        y, z = hy, side * hz
        for sg in w["segments"]:
            ang = phi + sg["gain"] * (phi - w.get("phi_ref", 0.0))
            y += sg["length"] * math.sin(ang)
            z += side * sg["length"] * math.cos(ang)
            pts += [(sg["xle_out"], y, z), (sg["xle_out"] - sg["chord_out"], y, z)]
    return np.array(pts)


def stroke_envelope(design, amp):
    """Body points at both ends of the stroke for the largest logged amplitude, plus a few degrees for the
    crank-rocker's offset mean; the tests check the real per-frame law."""
    lim = float(amp) + math.radians(6.0)
    return np.vstack([bird_points(design, lim), bird_points(design, -lim), bird_points(design, 0.0)])


def body_centre(design):
    """Midpoint of nose and tail on the body axis, body frame: where the eye puts the bird, wings aside."""
    xs = [sec["x_m"] for sec in design["body"]["loft"]["sections"]]
    return np.array([0.5 * (max(xs) + min(xs)), 0.0, 0.0])


def fit_distance(local_pts, offset_dir, lens, fill, centre, axis="max"):
    """Distance along `offset_dir` (unit, path frame; camera = centre + offset_dir * D, looking at `centre`) at
    which the points' projection reaches `fill` of the frame: axis 'max' fits the largest excursion from the frame
    centre on either axis, 'width' sets the full width. Bisection on the real perspective projection."""

    def size(D):
        q = project(centre + offset_dir * D, centre, lens, local_pts)
        if axis == "width":
            return 0.5 * (q[:, 0].max() - q[:, 0].min())
        return np.abs(q[:, :2]).max()

    lo, hi = 0.05, 500.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if size(mid) > fill else (lo, mid)
    return hi


def to_world(local, f, r, u):
    """Path-frame vectors (forward, right, up) to world, per sample: local (3,) or (N, 3) with frames (N, 3)."""
    local = np.asarray(local, float)
    return local[..., 0:1] * f + local[..., 1:2] * r + local[..., 2:3] * u


def body_axes(fl, win):
    """Smoothed body forward, right and up per sample in Blender axes, from the logged attitude: the bird crabs into
    the wind and banks, so its body is not aligned with its path."""
    q = fl["quat"]
    w, v = q[:, :1], q[:, 1:]

    def rot(x):
        t = 2 * np.cross(v, x)
        return x + w * t + np.cross(v, t)

    # body x forward, y up, z right -> Blender model axes (x, -z, y)
    fwd = smooth(rot(np.array([1.0, 0.0, 0.0])), win)
    right = smooth(rot(np.array([0.0, -1.0, 0.0])), win)
    fwd /= np.linalg.norm(fwd, axis=1, keepdims=True)
    right -= (right * fwd).sum(1, keepdims=True) * fwd
    right /= np.linalg.norm(right, axis=1, keepdims=True)
    return fwd, right, np.cross(right, fwd)


def body_offset(axes, pts, i=slice(None)):
    """Body-frame points (x fwd, y up, z right) as world offsets at sample(s) i."""
    fwd, right, up = (ax[i] for ax in axes)
    pts = np.asarray(pts, float)
    return pts[..., 0:1] * fwd + pts[..., 1:2] * up + pts[..., 2:3] * right


def beat_window(fl, a, b):
    """Odd window of one wingbeat at the span's mean flap rate: a moving average one period long cancels the beat's
    heave and surge, and bends the path less than a longer one."""
    hz = (fl["phase"][b - 1] - fl["phase"][a]) / (2 * math.pi * max(fl["t"][b - 1] - fl["t"][a], 1e-6))
    return frames_per(fl, float(np.clip(1.0 / max(hz, 1e-3), 0.25, 1.2)))


def on_bird(fl, s, span, offset, lens, fits, centre_body, clearance):
    """A camera that rides with the bird: `offset` (path frame forward, right, up; its direction only) from the
    point `centre_body` of the bird, at the distance where the bird's extremities over the span meet every
    (fill, axis) in `fits` (see fit_distance), looking at that point. Position and attitude are averaged over one
    wingbeat so the camera glides over it."""
    a, b = span
    win = beat_window(fl, a, b)
    p = smooth(fl["pos"], win)
    f, r, u = frames_of(fl)
    axes = body_axes(fl, win)
    env = stroke_envelope(load_design(fl["design"]), fl["amplitude"][a:b].max())
    # the bird's extremities and aim point in path coordinates over the span: it crabs and banks
    idx = np.linspace(a, b - 1, 6).astype(int)
    local = np.concatenate([body_offset(axes, env, i) @ np.stack([f[i], r[i], u[i]], 1) for i in idx])
    centre = np.mean([body_offset(axes, centre_body, i) @ np.stack([f[i], r[i], u[i]], 1) for i in idx], 0)
    o = offset / np.linalg.norm(offset)
    D = max(fit_distance(local, o, lens, fill, centre, axis) for fill, axis in fits)
    tgt = p + body_offset(axes, centre_body)
    return dict(frames=span, cam=lift_clear(tgt + to_world(o * D, f, r, u), s, margin=clearance), target=tgt, lens=lens)


# --- water and banks from the 2 m lidar (out/env/terrain, written by build_terrain.py) ---

def read_png(path):
    """8-bit PNG as a numpy array (rows north to south). PIL outside Blender, OpenImageIO inside it."""
    try:
        from PIL import Image
        return np.asarray(Image.open(path))
    except ImportError:
        import OpenImageIO as oiio
        return np.asarray(oiio.ImageBuf(path).get_pixels(oiio.UINT8))


class Banks:
    """Lidar waterline in the L0 window: water_l0.png (L = water fraction) and shore_l0.png (L = distance on land
    to the waterline / SHORE_RANGE), pixel (r, c) at x = e0[0] + 2c, y = e0[3] - 2r."""

    def __init__(self):
        import config as C
        ter = geo.TER
        with open(os.path.join(ter, "meta.json")) as fh:
            self.e0 = json.load(fh)["l0_ext"]
        self.water = read_png(os.path.join(ter, "water_l0.png"))[..., 0] / 255.0
        sh = read_png(os.path.join(ter, "shore_l0.png"))
        self.shore = sh[..., 0] / 255.0 * C.SHORE_RANGE
        self.valid = sh[..., 1] > 0
        self.res = (self.e0[2] - self.e0[0]) / (self.water.shape[1] - 1)

    def rc(self, x, y):
        c = np.rint((np.asarray(x) - self.e0[0]) / self.res).astype(int)
        r = np.rint((self.e0[3] - np.asarray(y)) / self.res).astype(int)
        ok = (r >= 0) & (r < self.water.shape[0]) & (c >= 0) & (c < self.water.shape[1])
        return np.clip(r, 0, self.water.shape[0] - 1), np.clip(c, 0, self.water.shape[1] - 1), ok

    def is_water(self, x, y):
        r, c, ok = self.rc(x, y)
        return ok & (self.water[r, c] > 0.5)

    def bank_points(self, cx, cy, radius, near=(2.5, 5.0)):
        """Land points `near` metres from the waterline within `radius` of (cx, cy)."""
        r0, c0, _ = self.rc(cx, cy)
        n = int(radius / self.res)
        rs = slice(max(r0 - n, 0), r0 + n + 1)
        cs = slice(max(c0 - n, 0), c0 + n + 1)
        d = self.shore[rs, cs]
        ok = self.valid[rs, cs] & (self.water[rs, cs] < 0.1) & (d >= near[0]) & (d <= near[1])
        rr, cc = np.nonzero(ok)
        x = self.e0[0] + (cc + cs.start) * self.res
        y = self.e0[3] - (rr + rs.start) * self.res
        keep = np.hypot(x - cx, y - cy) <= radius
        return x[keep], y[keep]


_BANKS = []


def banks():
    if not _BANKS:
        _BANKS.append(Banks())
    return _BANKS[0]


def _angle_deg(a, b):
    return np.abs((np.degrees(a - b) + 180.0) % 360.0 - 180.0)


def pick_bank(p_span, lens, ground):
    """The river-bank camera point for a low shot of the bird over the water: on land 2.5-5 m from the lidar
    waterline within 250 m of the bird's mean ground point, 60-220 m from it, the sun at least 60 degrees off the
    view axis (out of frame with room for flare), the bird inside the frame on every sample, and as much water as possible in the near foreground."""
    pm = p_span.mean(0)
    bx, by = banks().bank_points(pm[0], pm[1], 250.0)
    if not len(bx):
        raise ValueError("river_low: no lidar river bank within 250 m of the bird's path over this span")
    vx, vy = pm[0] - bx, pm[1] - by
    dist = np.hypot(vx, vy)
    az = np.arctan2(vx, vy)
    _, sun_az, _ = geo.sun_vector()
    ok = (dist > 60.0) & (dist < 220.0) & (_angle_deg(az, math.radians(sun_az)) > 60.0)
    half = math.atan(SENSOR_W / 2 / lens)
    rays = np.array([-0.7, -0.35, 0.0, 0.35, 0.7]) * half
    wet = np.zeros(len(bx))
    for da in rays:
        for t in (5.0, 10.0, 16.0, 24.0, 34.0, 46.0):
            wet += banks().is_water(bx + t * np.sin(az + da), by + t * np.cos(az + da))
    score = wet / (len(rays) * 6) - 0.002 * np.abs(dist - 110.0)
    order = np.argsort(-np.where(ok, score, -np.inf))
    for k in order[: 400]:
        if not ok[k]:
            break
        cam = np.array([bx[k], by[k], float(ground(np.array([bx[k]]), np.array([by[k]]))[0]) + 2.2])
        tgt = river_target(cam, p_span, pm)
        q = project(cam, tgt, lens, p_span)
        if (q[:, 2] > 0).all() and (np.abs(q[:, :2]) < 0.85).all():
            return cam
    raise ValueError("river_low: no bank point keeps the bird in frame over this span")


def river_target(cam, p, pm):
    """Aim between the bird `p` and a far point on the bearing of the bird's span mean `pm`, so the horizon stays in frame."""
    h = np.array([pm[0] - cam[0], pm[1] - cam[1], 0.0])
    far = cam + h / np.linalg.norm(h) * 3000.0 + np.array([0.0, 0.0, 260.0])
    return 0.45 * p + 0.55 * far


# --- shot definitions ---

def track_side(fl, s, span):
    p = smooth(fl["pos"], 31)
    f, r, u = frames_of(fl)
    side = sun_side(fl, r)
    # On the shadow side looking back into the lit bird and the sun-lit wall is flat; shoot with the sun over the shoulder.
    cam = p + r * side * 15.0 + u * 2.5 - f * 3.0
    return dict(frames=span, cam=lift_clear(cam, s), target=p + f * 2.5, lens=62.0)


TRACK_LONG_LENS = 100.0
TRACK_LONG_FILL = 0.36  # the bird's projected width over the frame width


def track_side_long(fl, s, span):
    """track_side's angle (sun over the shoulder, a little behind and above) on a long lens with the body centred:
    the distance comes from the design's real geometry so the bird spans TRACK_LONG_FILL of the frame width."""
    side = sun_side(fl, frames_of(fl)[1])
    design = load_design(fl["design"])
    return on_bird(fl, s, span, np.array([-3.0, 15.0 * side, 2.5]), TRACK_LONG_LENS, [(TRACK_LONG_FILL, "width"), (0.7, "max")],
                   body_centre(design), 3.0)


def treetop_pass(fl, s, span):
    a, b = span
    m = (a + b) // 2
    p = smooth(fl["pos"], 15)
    f, r, u = frames_of(fl)
    c = p[m] + r[m] * 9.0 - f[m] * 2.0 - u[m] * 3.0
    c[2] = max(c[2], float(s.top(np.array([c[0]]), np.array([c[1]]))[0]) + 1.5)
    c = clear_sight(c, p[a:b], s, margin=1.0)
    return dict(frames=span, cam=np.tile(c, (len(p), 1)), target=p, lens=26.0)


def wide_valley(fl, s, span):
    """From high on the far bank, 1.24 km back along the bird's path: the whole wall of Rundle behind a speck of a bird."""
    a, b = span
    p = smooth(fl["pos"], 61)
    f, r, _ = chord_frame(p, a, b)
    pm = p[(a + b) // 2]
    c = pm - f * 1240.0 - r * 190.0
    c[2] = float(s.top(np.array([c[0]]), np.array([c[1]]))[0]) + 90.0
    aim = pm + f * 1600.0 - r * 47.0
    aim[2] = p[a:b, 2].mean() + 276.0
    tgt = 0.82 * aim + 0.18 * p
    return dict(frames=span, cam=np.tile(c, (len(p), 1)), target=tgt, lens=42.0)


def under_wing(fl, s, span):
    p = smooth(fl["pos"], 9)
    f, r, u = frames_of(fl)
    side = sun_side(fl, r)
    cam = p - f * 1.05 + r * side * 0.75 - u * 0.62
    tgt = p + r * side * 0.75 + f * 1.2 + u * 0.18
    return dict(frames=span, cam=cam, target=tgt, lens=20.0)


UNDER_FULL_LENS = 24.0
UNDER_FULL_FILL = 0.85  # the stroke envelope's half-extent over the frame's half-extent


def under_wing_full(fl, s, span):
    """Below and behind the sun-side wing looking up along the body, close, with both wingtips at either end of the
    stroke, the nose and the tail inside the frame on a normal lens. Distance from the design's real geometry."""
    side = sun_side(fl, frames_of(fl)[1])
    env = stroke_envelope(load_design(fl["design"]), fl["amplitude"][span[0]:span[1]].max())
    return on_bird(fl, s, span, np.array([-0.62, 0.38 * side, -0.69]), UNDER_FULL_LENS, [(UNDER_FULL_FILL, "max")],
                   0.5 * (env.min(0) + env.max(0)), 1.0)


def lead_front(fl, s, span):
    p = smooth(fl["pos"], 21)
    f, r, u = frames_of(fl)
    cam = p + f * 10.0 + u * 0.8 + r * 2.2
    return dict(frames=span, cam=lift_clear(cam, s), target=p, lens=45.0)


def ridge_rise(fl, s, span):
    a, b = span
    p = smooth(fl["pos"], 25)
    f, r, u = frames_of(fl)
    end = p[b - 1]
    # Fixed on the east shoulder of the ridge; the bird climbs along the crest with the wall of Rundle filling the frame.
    c = end - f[b - 1] * 210.0 - r[b - 1] * 80.0
    c[2] = float(s.top(np.array([c[0]]), np.array([c[1]]))[0]) + 6.0
    tgt = p + np.array([0, 0, 2.0])
    c = clear_sight(c, p[a:b], s)  # nearby crowns on the slope must not wall off the bird
    return dict(frames=span, cam=np.tile(c, (len(p), 1)), target=tgt, lens=85.0)


def top_down(fl, s, span):
    p = smooth(fl["pos"], 25)
    f, r, u = frames_of(fl)
    cam = p + u * 11.0 - f * 7.0 + r * 1.5
    return dict(frames=span, cam=cam, target=p + f * 3.0, lens=30.0)


RIVER_LENS = 20.0


def river_low(fl, s, span):
    """Locked off on the Bow's bank, 2.2 m above the ground, the bird over the water (see pick_bank)."""
    a, b = span
    p = smooth(fl["pos"], 25)
    c = pick_bank(p[a:b], RIVER_LENS, geo.Ground())
    return dict(frames=span, cam=np.tile(c, (len(p), 1)), target=river_target(c, p, p[a:b].mean(0)), lens=RIVER_LENS)


SHOTS = dict(track_side=track_side, track_side_long=track_side_long, treetop_pass=treetop_pass, wide_valley=wide_valley,
             under_wing=under_wing, under_wing_full=under_wing_full, lead_front=lead_front, ridge_rise=ridge_rise,
             top_down=top_down, river_low=river_low)

# span used when the caller gives none: fractions of the log, for env's own layout stills
DEFAULT_WINDOW = dict(track_side=(0.05, 0.40), track_side_long=(0.05, 0.40), treetop_pass=(0.10, 0.30), wide_valley=(0.0, 1.0),
                      under_wing=(0.35, 0.55), under_wing_full=(0.35, 0.55), lead_front=(0.25, 0.50), ridge_rise=(0.60, 1.0),
                      top_down=(0.05, 0.30), river_low=(0.0, 0.28))
