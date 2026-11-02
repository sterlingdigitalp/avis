#!/usr/bin/env python3
"""Flight corridor and terrain height query. Run in out/env/.venv with -I.

Writes data/env/corridor.json (SPEC format) and data/env/terrain_height.bin (format in env/terrain.js).
Plan-view waypoints are chosen by hand against the survey maps. Altitude is solved, not chosen: the lowest curve that
clears the real lidar canopy by the nominal clearance within the climb and sink limits a 2 m span bird can hold.
"""
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy import ndimage as ndi

import config as C
from solar import solar_az_el

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "out/env/raw")
TER = os.path.join(ROOT, "out/env/terrain")
OE, ON = C.ORIGIN_UTM
meta = json.load(open(f"{TER}/meta.json"))
origin_z = meta["origin_z"]

# Plan view, metres east and north of the origin. River crossing, forested bench, up onto the ridge, then along its crest.
PLAN = np.array([
    [1210.0, 900.0],
    [1130.0, 760.0],
    [1010.0, 560.0],
    [880.0, 340.0],
    [775.0, 140.0],
    [725.0, -30.0],
    [655.0, -165.0],
    [560.0, -220.0],
])
CLEAR_NOMINAL = 11.0  # metres over the canopy top
MAX_CLIMB = 0.095  # dz/ds: 1.14 m/s at 12 m/s
MAX_SINK = 0.06
START_ABOVE = 11.0


def catmull(P, n=40):
    Q = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = []
    for i in range(1, len(Q) - 2):
        t = np.linspace(0, 1, n, endpoint=False)[:, None]
        p0, p1, p2, p3 = Q[i - 1], Q[i], Q[i + 1], Q[i + 2]
        out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    out.append(P[-1:])
    return np.vstack(out)


def load_surface():
    hw, hs, he, hn = C.HR_BOX
    dtm = np.load(f"{RAW}/hrdem_site/dtm.npy", mmap_mode="r")
    dsm = np.load(f"{RAW}/hrdem_site/dsm.npy", mmap_mode="r")
    return dtm, dsm, (hw, hn)


def sample(g, xr, yr, nw, res=2.0):
    col = (xr + OE - nw[0]) / res - 0.5
    row = (nw[1] - (yr + ON)) / res - 0.5
    return ndi.map_coordinates(g, [row, col], order=1, mode="nearest")


def solve_altitude(s, top):
    z = top + CLEAR_NOMINAL
    ds = np.diff(s)
    # lift the curve until it respects the climb limit going forward and sink limit going back
    for i in range(1, len(z)):
        z[i] = max(z[i], z[i - 1] - MAX_SINK * ds[i - 1])
    for i in range(len(z) - 2, -1, -1):
        z[i] = max(z[i], z[i + 1] - MAX_CLIMB * ds[i])
    # gentle smoothing that never goes under the requirement
    for _ in range(40):
        zs = ndi.gaussian_filter1d(z, 8.0)
        z = np.maximum(zs, top + CLEAR_NOMINAL)
    for i in range(1, len(z)):
        z[i] = max(z[i], z[i - 1] - MAX_SINK * ds[i - 1])
    for i in range(len(z) - 2, -1, -1):
        z[i] = max(z[i], z[i + 1] - MAX_CLIMB * ds[i])
    return z


def corridor():
    dtm, dsm, nw = load_surface()
    path = catmull(PLAN, 60)
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    # resample every 5 m
    s5 = np.arange(0, s[-1], 5.0)
    xr = np.interp(s5, s, path[:, 0])
    yr = np.interp(s5, s, path[:, 1])
    # canopy top within 5 m of the line
    r0, c0 = int(max(0, (nw[1] - (yr.max() + ON)) / 2 - 30)), int(max(0, (xr.min() + OE - nw[0]) / 2 - 30))
    r1, c1 = int((nw[1] - (yr.min() + ON)) / 2 + 30), int((xr.max() + OE - nw[0]) / 2 + 30)
    sub_s = np.array(dsm[r0:r1, c0:c1], np.float32)
    sub_t = np.array(dtm[r0:r1, c0:c1], np.float32)
    top_map = ndi.maximum_filter(np.nan_to_num(sub_s, nan=0), size=5)
    col = (xr + OE - nw[0]) / 2 - 0.5 - c0
    row = (nw[1] - (yr + ON)) / 2 - 0.5 - r0
    top = ndi.map_coordinates(top_map, [row, col], order=1) - origin_z
    grd = ndi.map_coordinates(np.nan_to_num(sub_t, nan=0), [row, col], order=1) - origin_z
    alt = solve_altitude(s5, top.copy())
    clear = alt - top
    slope = np.diff(alt) / np.diff(s5)
    wp_step = 25
    idx = list(range(0, len(s5), wp_step // 5))
    if idx[-1] != len(s5) - 1:
        idx.append(len(s5) - 1)
    wp = [[float(xr[i]), float(alt[i]), float(-yr[i])] for i in idx]
    y, m, d = C.SUN_DATE
    az, el = solar_az_el(C.LAT_LON[0], C.LAT_LON[1], y, m, d, C.SUN_MDT_HOURS + 6.0)
    a, e = np.radians(az), np.radians(el)
    # world frame: x east, y up, z south
    sun = [float(np.cos(e) * np.sin(a)), float(np.sin(e)), float(-np.cos(e) * np.cos(a))]
    out = {
        "origin_utm": {"zone": "11N", "epsg": C.EPSG, "easting": OE, "northing": ON},
        "elevation_m_at_origin": round(origin_z, 2),
        "sun_dir": sun,
        "waypoints": [[round(v, 2) for v in w] for w in wp],
        "min_clearance_m": round(float(clear.min()), 2),
        "place": "Bow River and forested bench below the north-east wall of Mount Rundle, Bow Valley, Alberta, just outside the east gate of Banff National Park",
        "lat_lon_deg": C.LAT_LON,
        "elevation_range_asl_m": [round(float(alt.min() + origin_z), 1), round(float(alt.max() + origin_z), 1)],
        "length_m": round(float(s5[-1]), 1),
        "max_climb_gradient": round(float(slope.max()), 4),
        "max_sink_gradient": round(float(-slope.min()), 4),
        "nominal_speed_mps": [10, 14],
        "clearance_reference": "lidar DSM (HRDEM 2 m) maximum within 5 m of the path",
        "sun_time_local": "2022-09-21 %05.2f MDT, az %.1f deg, el %.1f deg" % (C.SUN_MDT_HOURS, az, el),
    }
    os.makedirs(f"{ROOT}/data/env", exist_ok=True)
    json.dump(out, open(f"{ROOT}/data/env/corridor.json", "w"), indent=1)
    np.savez(f"{TER}/corridor_profile.npz", s=s5, x=xr, y=yr, alt=alt, top=top, grd=grd)
    print("length %.0f m, alt ASL %.0f..%.0f, climb max %.3f, sink max %.3f, clearance min %.1f mean %.1f" % (
        s5[-1], alt.min() + origin_z, alt.max() + origin_z, slope.max(), -slope.min(), clear.min(), clear.mean()))
    for i in range(0, len(s5), 30):
        print("s %5.0f  xy (%6.0f %6.0f)  ground %6.1f  canopy %6.1f  alt %6.1f  clear %5.1f  agl %5.1f" % (
            s5[i], xr[i], yr[i], grd[i] + origin_z, top[i] + origin_z, alt[i] + origin_z, alt[i] - top[i], alt[i] - grd[i]))
    return xr, yr, alt


HEADER = struct.Struct("<4sIIIfffffIIfffff")


def height_bin(xr, yr):
    """Near grid 4 m with canopy heights over 3 km; far grid 40 m over 36 km. u16 heights relative to the origin."""
    h_off, h_scale = -500.0, 0.05
    cx, cy = float(xr.mean()), float(yr.mean())
    # near
    res_n, half_n = 4.0, 1500.0
    xs = np.arange(cx - half_n, cx + half_n, res_n)
    zs_s = np.arange(-(cy + half_n), -(cy - half_n), res_n)  # world z = -north, increasing to the south
    gx, gz = np.meshgrid(xs, zs_s)
    dtm, dsm, nw = load_surface()
    d = np.nan_to_num(np.array(dtm, np.float32), nan=0.0)
    h = sample(d, gx, -gz, nw) - origin_z
    surf = np.nan_to_num(np.array(dsm, np.float32), nan=0.0)
    top = ndi.maximum_filter(surf, size=3)
    t = sample(top, gx, -gz, nw) - origin_z
    can = np.clip(t - h, 0, 63.75)
    # far: fused 30 m
    z30 = np.load(f"{TER}/fused30.npy")
    W0, S0, E0, N0 = C.AOI
    res_f, half_f = 40.0, 18000.0
    xf = np.arange(cx - half_f, cx + half_f, res_f)
    zf = np.arange(-(cy + half_f), -(cy - half_f), res_f)
    fx, fz = np.meshgrid(xf, zf)
    col = (fx + OE - W0) / 30 - 0.5
    row = (N0 - (-fz + ON)) / 30 - 0.5
    hf = ndi.map_coordinates(z30, [row, col], order=1, mode="nearest") - origin_z
    enc = lambda a: np.clip((a - h_off) / h_scale + 0.5, 0, 65535).astype("<u2")
    near = enc(h)
    can8 = np.clip(can / 0.25 + 0.5, 0, 255).astype(np.uint8)
    far = enc(hf)
    hdr = struct.pack("<4sIffIIfffIIfff", b"AVTH", 1, h_off, h_scale,
                      near.shape[1], near.shape[0], float(xs[0]), float(zs_s[0]), res_n,
                      far.shape[1], far.shape[0], float(xf[0]), float(zf[0]), res_f)
    path = f"{ROOT}/data/env/terrain_height.bin"
    with open(path, "wb") as f:
        f.write(hdr)
        f.write(near.tobytes())
        f.write(can8.tobytes())
        f.write(far.tobytes())
    print("terrain_height.bin", os.path.getsize(path) / 1e6, "MB near", near.shape, "far", far.shape)
    # reference points read straight from the raw lidar, for the JS test fixture
    pts = []
    for (e, n) in [(610900, 5665860), (610700, 5665050), (609800, 5664400), (611600, 5665300), (610300, 5665800)]:
        x, y = e - OE, n - ON
        pts.append({"utm": [e, n], "world_xz": [x, -y], "raw_lidar_dtm_asl": float(sample(d, np.array([x]), np.array([y]), nw)[0])})
    json.dump(pts, open(f"{ROOT}/env/terrain.fixture.json", "w"), indent=1)


if __name__ == "__main__":
    xr, yr, alt = corridor()
    height_bin(xr, yr)
