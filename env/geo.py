"""Pure-numpy terrain and sun queries, importable without Blender (tests, camera rigs).

Axes are Blender: X east, Y north, Z up, origin at config.ORIGIN_UTM and the origin elevation.
"""
import math
import os

import numpy as np

import config as C
from solar import solar_az_el

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TER = os.path.join(ROOT, "out/env/terrain")


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
