"""Camera rigs keep the bird in frame. Pure numpy, no Blender.

    out/env/.venv/bin/python -I -m unittest discover -s env -p "test_shots.py"

Every Blender segment of film/edit.json is checked on the frames film/render.mjs renders (i0 = round(start_s * fps *
slow), n = round(dur * fps), the log resampled at fps * slow), with the lens the edit passes. The bird is placed from
the log alone (position, attitude quaternion, crank-rocker stroke table, design geometry) and projected with an
independent pinhole: Blender's 36 mm sensor, sensor fit AUTO at 1920x1080, view from cam to target with no roll.
"""
import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import geo
import import_flight as IF
import rigs as R

ROOT = geo.ROOT
with open(os.path.join(ROOT, "film/edit.json")) as _fh:
    EDIT = json.load(_fh)
SEGS = [s for s in EDIT["segments"] if s.get("kind") == "bl"]
LOG = {}
SURFACE = []


def log_json(scenario):
    if scenario not in LOG:
        with open(os.path.join(ROOT, "out/flight/%s.json" % scenario)) as fh:
            LOG[scenario] = json.load(fh)
    return LOG[scenario]


def flight(scenario, fps):
    return IF.load_flight(os.path.join(ROOT, "out/flight/%s.json" % scenario), fps=fps)


def surface():
    if not SURFACE:
        SURFACE.append(R.Surface())
    return SURFACE[0]


def pixel_xy(cam, tgt, lens, pts):
    """Pixel coordinates (origin top left) and depth: an independent pinhole, not rigs.project."""
    w, h = 1920.0, 1080.0
    fwd = (tgt - cam) / np.linalg.norm(tgt - cam)
    side = np.array([fwd[1], -fwd[0], 0.0]) / math.hypot(fwd[0], fwd[1])
    up = np.cross(side, fwd)
    rel = pts - cam
    depth = rel @ fwd
    f_px = lens / 36.0 * w
    return w / 2 + f_px * (rel @ side) / depth, h / 2 - f_px * (rel @ up) / depth, depth


def stroke_phi(law, amp, phase):
    """Bilinear lookup of the log's crank-rocker table phi[amplitude][phase]."""
    A, P, T = np.array(law["amplitude"]), np.array(law["phase"]), np.array(law["phi"])
    ph = np.mod(phase, 2 * math.pi)
    j = np.clip(np.searchsorted(A, amp) - 1, 0, len(A) - 2)
    i = np.clip(np.searchsorted(P, ph) - 1, 0, len(P) - 2)
    ta = np.clip((amp - A[j]) / (A[j + 1] - A[j]), 0, 1)
    tp = np.clip((ph - P[i]) / (P[i + 1] - P[i]), 0, 1)
    top = T[j, i] * (1 - tp) + T[j, i + 1] * tp
    bot = T[j + 1, i] * (1 - tp) + T[j + 1, i + 1] * tp
    return top * (1 - ta) + bot * ta


def quat_rotate(q, v):
    """Rotate vectors v (N, K, 3) by unit quaternions q (N, 4) as (w, x, y, z)."""
    w, u = q[:, None, :1], q[:, None, 1:]
    t = 2 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def bird_world(fl, frames, law, design):
    """(N, K, 3) Blender points per frame: nose, tail, both wingtips (leading and trailing edge) at the logged stroke."""
    fr = np.asarray(frames)
    phi = stroke_phi(law, fl["amplitude"][fr], fl["phase"][fr])
    wing = design["wing"]
    hx, hy, hz = wing["hinge"]
    y = np.full(len(fr), hy)
    zr = np.full(len(fr), hz)
    for sg in wing["segments"]:
        ang = phi * (1 + sg["gain"]) - sg["gain"] * wing.get("phi_ref", 0.0)
        y = y + sg["length"] * np.sin(ang)
        zr = zr + sg["length"] * np.cos(ang)
    tip = wing["segments"][-1]
    xs = [s["x_m"] for s in design["body"]["loft"]["sections"]]
    n = len(fr)
    pts = [np.stack([np.full(n, max(xs)), np.zeros(n), np.zeros(n)], 1), np.stack([np.full(n, min(xs)), np.zeros(n), np.zeros(n)], 1)]
    for side in (1, -1):
        for x in (tip["xle_out"], tip["xle_out"] - tip["chord_out"]):
            pts.append(np.stack([np.full(n, x), y, side * zr], 1))
    body = np.stack(pts, 1)  # (N, K, 3) body frame: x fwd, y up, z right
    model = np.stack([body[..., 0], -body[..., 2], body[..., 1]], -1)  # SPEC: (x, y, z) -> Blender (x, -z, y)
    return quat_rotate(fl["quat"][fr], model) + fl["pos"][fr][:, None, :]


def plate(seg):
    fps = EDIT["fps"]
    slow = seg.get("slow", 1)
    i0 = round(seg["start_s"] * fps * slow)
    return fps * slow, (i0, i0 + round(seg["dur"] * fps))


def shot(fl, name, span):
    return R.SHOTS[name](fl, surface(), span)


class FilmShotsKeepTheBird(unittest.TestCase):
    """Every flight shot of today's edit, with the edit's lens: the bird's centre inside the frame on every frame."""

    MARGIN_PX = 24

    def test_each_segment(self):
        ground = geo.Ground()
        for seg in SEGS:
            with self.subTest(seg=seg["id"], shot=seg["shot"]):
                fps, (a, b) = plate(seg)
                fl = flight(seg["scenario"], fps)
                sh = shot(fl, seg["shot"], (a, b))
                lens = seg.get("lens") or sh["lens"]
                for i in range(a, b):
                    x, y, d = pixel_xy(sh["cam"][i], sh["target"][i], lens, fl["pos"][i][None])
                    self.assertGreater(d[0], 0.5, "frame %d: bird behind the camera" % i)
                    self.assertTrue(self.MARGIN_PX <= x[0] <= 1920 - self.MARGIN_PX and self.MARGIN_PX <= y[0] <= 1080 - self.MARGIN_PX,
                                    "frame %d: bird at (%.0f, %.0f) px" % (i, x[0], y[0]))
                cams = sh["cam"][a:b]
                self.assertTrue((cams[:, 2] > ground(cams[:, 0], cams[:, 1]) + 1.0).all(), "camera below ground")
                if np.ptp(cams, axis=0).max() < 1e-6:
                    self.assertSightClear(cams[0], fl["pos"][a:b])

    def assertSightClear(self, cam, birds, near=5.0):
        """A locked-off camera must see the bird over the lidar canopy, not through the crowns in front of it."""
        for bird in birds[::6]:
            d = np.linalg.norm(bird[:2] - cam[:2])
            s = np.linspace(near / max(d, near), 0.85, 60)
            pts = cam[None] + s[:, None] * (bird - cam)[None]
            top = surface().top(pts[:, 0], pts[:, 1])
            self.assertTrue((pts[:, 2] > top).all(), "canopy blocks the sight line %.0f m out" % (s[np.argmin(pts[:, 2] - top)] * d))


def spans_across(fl, seconds=4.0, step=12.0):
    """4 s spans every 12 s through the whole log: a rig must hold on any stretch of it, not just today's."""
    fps = fl["fps"]
    n = int(round(seconds * fps))
    return [(int(t * fps), int(t * fps) + n) for t in np.arange(2.0, fl["t"][-1] - seconds, step)]


class TrackSideLong(unittest.TestCase):
    """Centred under a long lens: the body's midpoint (halfway from nose to tail) within 4% of frame width / 7% of
    frame height of the centre, the whole bird inside the frame, and the bird 20-50% of the frame width, on every
    frame of every span."""

    def check(self, fl, span, law, design):
        sh = shot(fl, "track_side_long", span)
        self.assertGreaterEqual(sh["lens"], 85.0)
        a, b = span
        pts = bird_world(fl, range(a, b), law, design)
        for k, i in enumerate(range(a, b)):
            mid = 0.5 * (pts[k, 0] + pts[k, 1])  # nose and tail
            x, y, d = pixel_xy(sh["cam"][i], sh["target"][i], sh["lens"], np.vstack([mid[None], pts[k]]))
            self.assertTrue((d > 0).all())
            self.assertLess(abs(x[0] - 960), 0.04 * 1920, "frame %d: centre off by %.0f px" % (i, x[0] - 960))
            self.assertLess(abs(y[0] - 540), 0.07 * 1080, "frame %d: centre off by %.0f px" % (i, y[0] - 540))
            self.assertTrue((x > 0).all() and (x < 1920).all() and (y > 0).all() and (y < 1080).all(), "frame %d: bird cut" % i)
            width = (x[1:].max() - x[1:].min()) / 1920
            self.assertTrue(0.20 <= width <= 0.50, "frame %d: bird is %.2f of the width" % (i, width))

    def test_film_span_and_whole_log(self):
        seg = next(s for s in SEGS if s["shot"].startswith("track_side"))
        law = log_json(seg["scenario"])["stroke_law"]
        fps, span = plate(seg)
        fl = flight(seg["scenario"], fps)
        design = R.load_design(fl["design"])
        for sp in [span] + spans_across(fl):
            with self.subTest(span=sp):
                self.check(fl, sp, law, design)


class UnderWingFull(unittest.TestCase):
    """The whole bird (nose, tail, both wingtips at the logged stroke) inside the frame with 2% margin on a lens of
    at least 20 mm, the camera within 4 m of the bird (close) and below it, on every frame of every span."""

    def check(self, fl, span, law, design):
        sh = shot(fl, "under_wing_full", span)
        self.assertGreaterEqual(sh["lens"], 20.0)
        a, b = span
        pts = bird_world(fl, range(a, b), law, design)
        for k, i in enumerate(range(a, b)):
            x, y, d = pixel_xy(sh["cam"][i], sh["target"][i], sh["lens"], pts[k])
            self.assertTrue((d > 0).all())
            m = 0.02
            self.assertTrue((x > m * 1920).all() and (x < (1 - m) * 1920).all() and (y > m * 1080).all() and (y < (1 - m) * 1080).all(),
                            "frame %d: bird cut, x %.0f..%.0f y %.0f..%.0f px" % (i, x.min(), x.max(), y.min(), y.max()))
            off = fl["pos"][i] - sh["cam"][i]
            self.assertLess(np.linalg.norm(off), 4.0)
            self.assertGreater(off[2], 0.3, "camera not under the bird")

    def test_film_span_and_whole_log(self):
        seg = next(s for s in SEGS if s["shot"].startswith("under_wing"))
        law = log_json(seg["scenario"])["stroke_law"]
        fps, span = plate(seg)
        fl = flight(seg["scenario"], fps)
        design = R.load_design(fl["design"])
        for sp in [span] + spans_across(fl):
            with self.subTest(span=sp):
                self.check(fl, sp, law, design)


class RelativeToTheLog(unittest.TestCase):
    """No rig hides a fixed world position: moving the whole log sideways moves every camera with it (terrain
    clearance aside), and a hold inside a span sees the same camera as the whole span."""

    def test_translation(self):
        seg = SEGS[0]
        fl = flight(seg["scenario"], EDIT["fps"])
        shift = np.array([37.0, -23.0, 0.0])
        moved = dict(fl, pos=fl["pos"] + shift)
        span = (900, 1020)
        for name in ("wide_valley", "treetop_pass", "ridge_rise", "track_side_long", "under_wing_full", "lead_front", "top_down"):
            with self.subTest(shot=name):
                c0 = shot(fl, name, span)["cam"][span[0]:span[1]]
                c1 = shot(moved, name, span)["cam"][span[0]:span[1]]
                self.assertTrue(np.allclose(c1[:, :2] - c0[:, :2], shift[:2], atol=1e-6))

    def test_locked_off_cameras_hold_still(self):
        seg = SEGS[0]
        fl = flight(seg["scenario"], EDIT["fps"])
        span = (600, 750)
        for name in ("wide_valley", "treetop_pass", "ridge_rise", "river_low"):
            with self.subTest(shot=name):
                cam = shot(fl, name, span)["cam"][span[0]:span[1]]
                self.assertTrue(np.allclose(cam, cam[0]))


class RiverBank(unittest.TestCase):
    """river_low stands on land 2-6 m from the lidar waterline, 1.5-3 m above the ground, with water in front."""

    def test_bank(self):
        seg = next(s for s in SEGS if s["shot"] == "river_low")
        fps, span = plate(seg)
        fl = flight(seg["scenario"], fps)
        cam = shot(fl, "river_low", span)["cam"][span[0]]
        bk = R.banks()
        r, c, ok = bk.rc(cam[0], cam[1])
        self.assertTrue(ok)
        self.assertLess(bk.water[r, c], 0.1)
        self.assertTrue(2.0 <= bk.shore[r, c] <= 6.0, bk.shore[r, c])
        g = float(geo.Ground()(np.array([cam[0]]), np.array([cam[1]]))[0])
        self.assertTrue(1.5 <= cam[2] - g <= 3.0)
        # water in the near foreground: at least a quarter of the ground 5-45 m out across the middle of the frame
        tgt = shot(fl, "river_low", span)["target"][span[0]]
        az = math.atan2(tgt[0] - cam[0], tgt[1] - cam[1])
        hits = [bk.is_water(cam[0] + t * math.sin(az + da), cam[1] + t * math.cos(az + da))
                for t in range(5, 46, 5) for da in np.radians(np.arange(-30, 31, 10))]
        self.assertGreater(np.mean(hits), 0.25)


if __name__ == "__main__":
    unittest.main()
