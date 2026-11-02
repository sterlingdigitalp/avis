#!/usr/bin/env python3
"""Fuse MRDEM + HRDEM lidar + Sentinel-2 into the render inputs. Run in out/env/.venv with -I.

Writes to out/env/terrain/:
  fused30.npy     whole AOI DTM at 30 m (metres above sea level)
  level_*.npz     multi-resolution height grids in Blender axes, heights relative to the origin elevation
  albedo.png      Sentinel-2 surface reflectance with terrain illumination removed, sRGB encoded
  masks.png       R snow, G vegetation, B water, whole AOI at 10 m
  meta.json       extents and origin
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

import config as C

Image.MAX_IMAGE_PIXELS = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "out/env/raw")
OUT = os.path.join(ROOT, "out/env/terrain")
os.makedirs(OUT, exist_ok=True)
W0, S0, E0, N0 = C.AOI
OE, ON = C.ORIGIN_UTM


def fractal(shape, res, wavelengths, seed):
    """Sum of filtered Gaussian noise octaves; deterministic. Unit variance per octave times amplitude from the caller."""
    rng = np.random.default_rng(seed)
    out = np.zeros(shape, np.float32)
    for wl, amp in wavelengths:
        sig = wl / res / 2.4
        # coarse field upsampled: much faster than a huge Gaussian
        f = max(1, int(sig / 2))
        n = rng.standard_normal((shape[0] // f + 4, shape[1] // f + 4)).astype(np.float32)
        n = ndi.gaussian_filter(n, sig / f)
        n /= n.std() + 1e-9
        if f > 1:
            n = ndi.zoom(n, f, order=1)
        n = n[:shape[0], :shape[1]]
        out += amp * n
    return out


def feather(valid, res, width):
    d = ndi.distance_transform_edt(valid) * res
    return np.clip(d / width, 0, 1) ** 2 * (3 - 2 * np.clip(d / width, 0, 1))


def stream_power(z, res, mask, iters=12, k=0.12, max_delta=38.0):
    """Stream-power incision so synthetic relief drains the way the real DEM does. mask 0 keeps the data untouched."""
    H, W = z.shape
    z = z.astype(np.float64).copy()
    z0 = z.copy()
    nbr = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    dist = np.array([1.414, 1, 1.414, 1, 1, 1.414, 1, 1.414]) * res
    pad = np.pad(z, 1, mode="edge")
    for it in range(iters):
        pad[1:-1, 1:-1] = z
        drops = np.stack([(z - pad[1 + a:H + 1 + a, 1 + b:W + 1 + b]) / d for (a, b), d in zip(nbr, dist)], 0)
        rcv = np.argmax(drops, axis=0)
        slope = np.maximum(np.max(drops, axis=0), 0.0)
        order = np.argsort(-z, axis=None)
        off = np.array([a * W + b for a, b in nbr], np.int64)
        target = np.arange(H * W, dtype=np.int64) + off[rcv.ravel()]
        valid = (slope.ravel() > 0)
        # boundary guard: receivers outside the grid go nowhere
        rr = np.repeat(np.arange(H), W) + np.array([a for a, b in nbr])[rcv.ravel()]
        cc = np.tile(np.arange(W), H) + np.array([b for a, b in nbr])[rcv.ravel()]
        valid &= (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
        acc = [1.0] * (H * W)
        tg = target.tolist()
        vd = valid.tolist()
        for i in order.tolist():
            if vd[i]:
                acc[tg[i]] += acc[i]
        A = np.array(acc).reshape(H, W)
        dz = k * np.sqrt(A) * np.minimum(slope, 1.6) * mask
        z -= dz
        # talus: anything steeper than about 50 degrees sheds into the cell below
        pad[1:-1, 1:-1] = z
        for (a, b), d in zip(nbr[1::2], dist[1::2]):
            dif = z - pad[1 + a:H + 1 + a, 1 + b:W + 1 + b]
            over = np.clip(dif - 1.2 * d, 0, None) * 0.25 * mask
            z -= over
            z += np.roll(np.roll(over, -a, 0), -b, 1) * 0.9
        z = np.minimum(np.maximum(z, z0 - max_delta), z0 + max_delta * 0.4)
        # fill the one- and two-cell pits incision leaves behind; real slopes drain, they do not hold craters
        ring = np.ones((3, 3), bool)
        ring[1, 1] = False
        for _ in range(3):
            z = np.maximum(z, ndi.minimum_filter(z, footprint=ring) * mask + z * (1 - mask) - 0.05)
        print("  erosion iter", it, "mean delta %.2f max %.1f" % (np.mean(np.abs(z - z0)[mask > 0.5]) if (mask > 0.5).any() else 0, np.max(np.abs(z - z0))), flush=True)
    return z.astype(np.float32)


def rock_strata(xx, yy, z, rockmask, amp, period, seed):
    """Tilted limestone bedding: terraces along a plane striking NW-SE, dipping SW like the Rundle thrust sheets."""
    # bedding normal: strike 315 deg, dip 45 deg to the SW
    nx, ny, nz = -0.5, -0.5, 0.707
    warp = fractal(z.shape, abs(xx[0, 1] - xx[0, 0]), [(180, 6.0), (60, 2.0)], seed)
    coord = (nx * xx + ny * yy + nz * z + warp) / period
    frac = coord - np.floor(coord)
    # asymmetric ledge profile: long bench, short riser
    ledge = np.where(frac < 0.78, frac / 0.78, (1 - frac) / 0.22) - 0.5
    return z + amp * ledge * rockmask


def build_dem():
    z30 = np.load(f"{RAW}/mrdem/dtm30.npy").astype(np.float32)
    z30 = np.where(np.isfinite(z30), z30, np.nanmedian(z30))
    # On bare rock use Copernicus GLO-30, which keeps the cliff and gully structure MRDEM smooths away. It is a
    # surface model, so under forest it would carry the canopy: keep MRDEM there.
    cop = np.load(f"{RAW}/cop30/cop30.npy").astype(np.float32)
    m10 = np.load(f"{OUT}/masks.npy", mmap_mode="r") if os.path.exists(f"{OUT}/masks.npy") else None
    if m10 is not None:
        veg30 = ndi.zoom(np.array(m10[1::3, 1::3, 1], np.float32) / 255.0, 1, order=1)[: z30.shape[0], : z30.shape[1]]
        bare = np.clip(1.0 - veg30 / 0.25, 0, 1)
        bare = ndi.gaussian_filter(bare, 2.0)
        ok = np.isfinite(cop)
        diff = np.where(ok & (bare > 0.9), cop - z30, np.nan)
        bias = np.nanmedian(diff)
        w = np.where(ok, bare, 0.0)
        z30 = z30 * (1 - w) + np.where(ok, cop - bias, z30) * w
        print("GLO-30 on bare rock: weight mean %.3f, bias %.2f m" % (w.mean(), bias), flush=True)
    hr8 = np.load(f"{RAW}/hrdem_survey/dtm.npy").astype(np.float32)
    bw, bs, be, bn = C.HR8_BOX
    # lidar into the 30 m grid where it exists
    r0, c0 = (N0 - bn) // 30, (bw - W0) // 30
    H30 = (bn - bs) // 30
    W30 = (be - bw) // 30
    valid8 = np.isfinite(hr8)
    filled = np.where(valid8, hr8, 0)
    cov = ndi.uniform_filter(valid8.astype(np.float32), 30 / 8)
    val = ndi.uniform_filter(filled, 30 / 8) / np.maximum(cov, 1e-3)
    sy = np.linspace(0, hr8.shape[0] - 1, H30)
    sx = np.linspace(0, hr8.shape[1] - 1, W30)
    gy, gx = np.meshgrid(sy, sx, indexing="ij")
    v30 = ndi.map_coordinates(val, [gy, gx], order=1)
    c30 = ndi.map_coordinates(cov, [gy, gx], order=1) > 0.98
    w30 = feather(c30, 30.0, 300.0).astype(np.float32)
    patch = z30[r0:r0 + H30, c0:c0 + W30]
    diff = (v30 - patch)[c30]
    print("lidar-MRDEM diff mean %.2f std %.2f p99 %.1f" % (diff.mean(), diff.std(), np.percentile(np.abs(diff), 99)), flush=True)
    patch[:] = patch * (1 - w30) + v30 * w30
    np.save(f"{OUT}/fused30.npy", z30)
    return z30, hr8, valid8


def bilinear_from(z, x_utm, y_utm, box, res):
    """Sample grid z (north-up, origin at box north-west pixel centres) at UTM coordinates. Cubic spline."""
    w, s, e, n = box
    col = (x_utm - w) / res - 0.5
    row = (n - y_utm) / res - 0.5
    return ndi.map_coordinates(z, [row, col], order=3, mode="nearest", prefilter=True)


def build_levels(z30, hr8, valid8, l0_center, l0_half, masks):
    """Levels: L0 2 m (lidar), L1 8 m, L2 30 m, L3 120 m."""
    # surface classes on the 10 m map, sampled for rock detail
    origin_z = float(bilinear_from(z30, np.array([OE]), np.array([ON]), C.AOI, 30.0)[0])
    hr2 = np.load(f"{RAW}/hrdem_site/dtm.npy", mmap_mode="r")
    print("origin elevation (MRDEM fused) %.1f" % origin_z, flush=True)

    def rockmask_at(xx, yy):
        col = ((xx + OE - W0) / 10).astype(int).clip(0, masks.shape[1] - 1)
        row = ((N0 - (yy + ON)) / 10).astype(int).clip(0, masks.shape[0] - 1)
        m = masks[row, col]
        return np.clip(1.0 - m[..., 1] / 180.0 - m[..., 2] / 120.0, 0, 1).astype(np.float32)

    def flatten_water(xx, yy, zz, res, w=None):
        if w is None:
            col = ((xx + OE - W0) / 10).astype(int).clip(0, masks.shape[1] - 1)
            row = ((N0 - (yy + ON)) / 10).astype(int).clip(0, masks.shape[0] - 1)
            w = masks[row, col, 2].astype(np.float32) / 255.0
            if w.max() <= 0:
                return zz
            w = ndi.gaussian_filter(w, 6.0 / res)
        inside = w > 0.5
        # water surface: normalised convolution of bank heights so the surface is smooth and level across its width
        sig = 45.0 / res
        num = ndi.gaussian_filter((zz * inside).astype(np.float32), sig)
        den = ndi.gaussian_filter(inside.astype(np.float32), sig)
        surf = num / np.maximum(den, 1e-3)
        t = np.clip((w - 0.35) / 0.2, 0, 1)
        return zz * (1 - t) + (surf - 0.25) * t

    levels = []
    # L3: 120 m, whole AOI minus edge
    ext3 = (W0 + 240 - OE, S0 + 240 - ON, E0 - 240 - OE, N0 - 240 - ON)
    # L2: 30 m out to 24 km
    ext2 = (-24000.0, -24000.0, 24000.0, 24000.0)
    ext1 = (-6500.0, -6500.0, 6500.0, 6500.0)
    ext0 = (l0_center[0] - l0_half[0], l0_center[1] - l0_half[1], l0_center[0] + l0_half[0], l0_center[1] + l0_half[1])

    def grid(ext, res):
        xs = np.arange(ext[0], ext[2] + res / 2, res)
        ys = np.arange(ext[3], ext[1] - res / 2, -res)
        return np.meshgrid(xs, ys)

    # L3
    xx, yy = grid(ext3, 120.0)
    z = bilinear_from(z30, xx + OE, yy + ON, C.AOI, 30.0)
    levels.append(("l3", ext3, 120.0, flatten_water(xx, yy, z, 120.0) - 12.0 - origin_z))
    # L2
    xx, yy = grid(ext2, 30.0)
    z = bilinear_from(z30, xx + OE, yy + ON, C.AOI, 30.0)
    levels.append(("l2", ext2, 30.0, flatten_water(xx, yy, z, 30.0) - 5.0 - origin_z))
    # L1 8 m: lidar where present; elsewhere MRDEM, with fractal detail and bedding on rock
    xx, yy = grid(ext1, 8.0)
    z = bilinear_from(z30, xx + OE, yy + ON, C.AOI, 30.0).astype(np.float32)
    rock = rockmask_at(xx, yy)
    rock_s = ndi.gaussian_filter(rock, 2.0)
    slope = np.hypot(*np.gradient(z, 8.0))
    det = fractal(z.shape, 8.0, [(300, 8.0), (120, 5.0), (55, 2.6), (26, 1.3)], 5)
    z_syn = z + det * (0.2 + 0.8 * rock_s) * np.clip(slope / 0.35, 0.2, 1.2)
    # Incise only outside lidar. Inside it the real 8 m surface already has its gullies.
    bw_, bs_, be_, bn_ = C.HR8_BOX
    col_ = (xx + OE - bw_) / 8 - 0.5
    row_ = (bn_ - (yy + ON)) / 8 - 0.5
    lidar_w = feather(ndi.map_coordinates(valid8.astype(np.float32), [row_, col_], order=1, mode="constant") > 0.99, 8.0, 260.0)
    erode_mask = (rock_s * np.clip(slope / 0.45, 0, 1) * (1 - lidar_w)).astype(np.float32)
    # coarse pass carves the big couloirs, fine pass adds their tributaries
    co = z_syn[::2, ::2]
    co_e = stream_power(co, 16.0, erode_mask[::2, ::2], iters=10, k=0.16)
    delta = ndi.zoom(co_e - co, 2, order=1)[: z_syn.shape[0], : z_syn.shape[1]]
    pad_h, pad_w = z_syn.shape[0] - delta.shape[0], z_syn.shape[1] - delta.shape[1]
    delta = np.pad(delta, ((0, pad_h), (0, pad_w)), mode="edge")
    z_syn = z_syn + delta
    z_syn = stream_power(z_syn, 8.0, erode_mask, iters=4, k=0.08, max_delta=10.0)
    # Cliff bands: strong enough to read at 6 km on the Rundle wall; then a finer set.
    mask_b = rock_s * np.clip(slope / 0.5, 0, 1) * (1 - lidar_w)
    z_syn = rock_strata(xx, yy, z_syn, mask_b, 15.0, 92.0, 6)
    z_syn = rock_strata(xx, yy, z_syn, mask_b, 4.0, 23.0, 7)
    fin = np.nan_to_num(hr8, nan=0.0)
    bw, bs, be, bn = C.HR8_BOX
    col = (xx + OE - bw) / 8 - 0.5
    row = (bn - (yy + ON)) / 8 - 0.5
    zl = ndi.map_coordinates(fin, [row, col], order=1, mode="constant")
    vl = ndi.map_coordinates(valid8.astype(np.float32), [row, col], order=1, mode="constant") > 0.99
    w = feather(vl, 8.0, 260.0).astype(np.float32)
    z1 = z_syn * (1 - w) + zl * w
    z1 = flatten_water(xx, yy, z1, 8.0)
    levels.append(("l1", ext1, 8.0, z1 - 1.5 - origin_z))
    # L0 2 m lidar: exact DTM, gaps from L1
    xx, yy = grid(ext0, 2.0)
    hw, hs, he, hn = C.HR_BOX
    col = (xx + OE - hw) / 2 - 0.5
    row = (hn - (yy + ON)) / 2 - 0.5
    c0, c1 = int(max(col.min() - 2, 0)), int(col.max() + 3)
    r0, r1 = int(max(row.min() - 2, 0)), int(row.max() + 3)
    sub = np.array(hr2[r0:r1, c0:c1]).astype(np.float32)
    val = np.isfinite(sub)
    zl = ndi.map_coordinates(np.nan_to_num(sub, nan=0.0), [row - r0, col - c0], order=1, mode="constant")
    vl = ndi.map_coordinates(val.astype(np.float32), [row - r0, col - c0], order=1, mode="constant") > 0.99
    print("L0 lidar coverage %.3f" % vl.mean(), flush=True)
    col1 = (xx - ext1[0]) / 8
    row1 = (ext1[3] - yy) / 8
    zf = ndi.map_coordinates(z1, [row1, col1], order=3, mode="nearest")
    w = feather(vl, 2.0, 120.0).astype(np.float32)
    z0 = zf * (1 - w) + zl * w
    # Water from the lidar itself: river and pond surfaces are flat to millimetres and carry no canopy. Confirmed with
    # Sentinel-2 so flat gravel and meadows are not taken for water.
    sub_s = np.array(np.load(f"{RAW}/hrdem_site/dsm.npy", mmap_mode="r")[r0:r1, c0:c1]).astype(np.float32)
    zs = ndi.map_coordinates(np.nan_to_num(sub, nan=0.0), [row - r0, col - c0], order=1, mode="nearest")
    ss = ndi.map_coordinates(np.nan_to_num(sub_s, nan=0.0), [row - r0, col - c0], order=1, mode="nearest")
    zc = zs.astype(np.float64) - float(np.median(zs))  # centred in float64: float32 z*z cancels to noise at 1300 m
    mean = ndi.uniform_filter(zc, 5)
    std = np.sqrt(np.maximum(ndi.uniform_filter(zc * zc, 5) - mean * mean, 0))
    flat = (std < 0.03) & (ss - zs < 0.4)
    s2col = ((xx + OE - W0) / 10).astype(int).clip(0, masks.shape[1] - 1)
    s2row = ((N0 - (yy + ON)) / 10).astype(int).clip(0, masks.shape[0] - 1)
    s2w = ndi.maximum_filter(masks[..., 2], size=5)[s2row, s2col] > 100 if False else None
    nd = np.load(f"{OUT}/ndwi.npy", mmap_mode="r")
    ndw = ndi.map_coordinates(np.array(nd, np.float32), [(N0 - (yy + ON)) / 10 - 0.5, (xx + OE - W0) / 10 - 0.5], order=1, mode="nearest")
    wmask = flat & (ndw > -0.25) & vl
    wmask = ndi.binary_opening(wmask, iterations=2)
    wmask = ndi.binary_closing(wmask, iterations=3)
    lab, nl = ndi.label(wmask)
    if nl:
        sizes = ndi.sum(wmask, lab, np.arange(1, nl + 1))
        keep = np.zeros(nl + 1, bool)
        keep[1:] = sizes > 150
        wmask = keep[lab]
    wsoft = ndi.gaussian_filter(wmask.astype(np.float32), 1.0)
    la = np.dstack([(wsoft * 255).astype(np.uint8), np.full(wsoft.shape, 255, np.uint8)])
    Image.fromarray(la, "LA").save(f"{OUT}/water_l0.png")
    print("L0 lidar water fraction %.4f" % wmask.mean(), flush=True)
    z0 = flatten_water(xx, yy, z0, 2.0, wsoft)
    levels.append(("l0", ext0, 2.0, z0 - origin_z))
    for name, ext, res, zz in levels:
        tmp = f"{OUT}/level_{name}.tmp.npz"
        np.savez(tmp, ext=np.array(ext), res=res, z=zz.astype(np.float32))
        os.replace(tmp, f"{OUT}/level_{name}.npz")  # atomic: a render that is loading never sees half a file
        print(name, zz.shape, "z", float(np.nanmin(zz)), float(np.nanmax(zz)), flush=True)
    json.dump({"origin_utm": C.ORIGIN_UTM, "origin_z": origin_z, "epsg": C.EPSG, "aoi": C.AOI, "l0_ext": ext0},
              open(f"{OUT}/meta.json", "w"))
    return origin_z


def load_s2():
    bands = {}
    for b in ("red", "green", "blue", "nir", "swir16"):
        a = np.load(f"{RAW}/s2/{b}.npy").astype(np.float32)
        fill = np.load(f"{RAW}/s2_20230921/{b}.npy").astype(np.float32)
        a = np.where(a > 0, a, fill)
        if b == "swir16":
            a = np.repeat(np.repeat(a, 2, axis=0), 2, axis=1)
        bands[b] = a / 10000.0
    scl = np.load(f"{RAW}/s2/scl.npy")
    scl = np.repeat(np.repeat(scl, 2, axis=0), 2, axis=1)
    return bands, scl


def build_maps(z30):
    """Albedo and masks at 10 m over the AOI."""
    p_m = f"{OUT}/masks.npy"
    bands, scl = load_s2()
    z10 = ndi.zoom(z30, 3, order=1)
    gy, gx = np.gradient(z10, 10.0)
    slope = np.hypot(gx, gy)
    n = np.dstack([-gx, gy, np.ones_like(gx)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    az, el = np.radians(169.7), np.radians(39.1)  # from the item metadata
    s = np.array([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)])
    cosi = (n * s).sum(2)
    # NDVI, NDSI, NDWI
    eps = 1e-3
    r, g, b, nir, sw = bands["red"], bands["green"], bands["blue"], bands["nir"], bands["swir16"]
    ndvi = (nir - r) / (nir + r + eps)
    ndsi = (g - sw) / (g + sw + eps)
    ndwi = (g - nir) / (g + nir + eps)
    # real snow is bright in green and dark in swir; shadowed rock can fool NDSI so require brightness
    snow = np.clip((ndsi - 0.35) / 0.2, 0, 1) * np.clip((g - 0.12) / 0.15, 0, 1)
    snow = ndi.gaussian_filter(snow, 0.7)
    flat = slope < 0.09
    # Glacial river water measured on the Bow beside the corridor: NDWI 0.67-0.76, NIR 0.006-0.010, SWIR 0.012-0.032
    water = (ndwi > 0.25) & (nir < 0.06) & (sw < 0.05) & flat & (snow < 0.3) & (ndvi < 0.1)
    water = ndi.binary_closing(water, iterations=2)
    lab, nlab = ndi.label(water)
    sizes = ndi.sum(water, lab, np.arange(1, nlab + 1))
    keep = np.zeros(nlab + 1, bool)
    keep[1:] = sizes > 40
    water = keep[lab]
    # vegetation cover: 0 rock, 1 closed canopy
    veg = np.clip((ndvi - 0.28) / 0.34, 0, 1)
    veg = veg * (1 - snow) * (~water)
    # illumination normalisation with a C-correction; cast shadows have no direct term so lift them with the sky only
    c = 0.45
    ratio = (np.sin(el) + c) / (np.clip(cosi, 0.0, 1) + c)
    shadowed = (scl == 2) | (scl == 3)
    ratio = np.where(shadowed, np.maximum(ratio, 1.7), ratio)
    # Low-pass the correction. Macro illumination goes, but small relief shading stays in the albedo: at range that
    # painted-in crag detail is the only real rock structure finer than the 30 m DEM.
    ratio = np.clip(ndi.gaussian_filter(ratio, 7.0), 0.7, 1.9)
    alb = np.dstack([r, g, b]) * ratio[..., None]
    # bare rock: unsharp mask brings back the crag and gully structure that the 10 m pixels and the correction soften
    bare = np.clip(1 - np.clip((ndvi - 0.2) / 0.3, 0, 1) - snow, 0, 1)[..., None]
    blur = np.dstack([ndi.gaussian_filter(alb[..., i], 2.5) for i in range(3)])
    alb = alb + bare * 0.9 * (alb - blur)
    alb = np.maximum(alb, 0.004)
    # snow stays snow
    alb = np.minimum(alb, 0.92)
    # built-up ground (town, highway, rail): replace by neighbourhood vegetation so the valley reads as wild
    low = z10 < 1560
    built = (ndvi < 0.42) & low & (slope < 0.25) & ~water
    dist_w = ndi.distance_transform_edt(~water) * 10
    built &= dist_w > 40  # keep natural gravel bars beside the channels
    # natural scree fans on the mountain flanks are steep; flat bright land is what we remove
    built = ndi.binary_dilation(ndi.binary_opening(built, iterations=1), iterations=2)
    good = ~built & (ndvi > 0.45) & ~water
    fill = np.zeros_like(alb)
    wgt = ndi.gaussian_filter(good.astype(np.float32), 25)
    for i in range(3):
        fill[..., i] = ndi.gaussian_filter((alb[..., i] * good).astype(np.float32), 25) / np.maximum(wgt, 1e-3)
    rng = np.random.default_rng(3)
    tex = ndi.gaussian_filter(rng.standard_normal(alb.shape[:2]).astype(np.float32), 1.0)
    tex = 1 + 0.8 * tex / tex.std() * 0.45
    fill *= np.clip(tex, 0.5, 1.6)[..., None]
    alb = np.where(built[..., None], fill, alb)
    veg = np.where(built, 0.82, veg)
    print("frac water %.4f snow %.4f veg %.3f built %.4f" % (water.mean(), snow.mean(), veg.mean(), built.mean()), flush=True)
    srgb = (np.clip(alb, 0, 1) ** (1 / 2.2) * 255 + 0.5).astype(np.uint8)
    Image.fromarray(srgb).save(f"{OUT}/albedo.png", compress_level=1)
    masks = np.dstack([snow * 255, veg * 255, water * 255]).astype(np.uint8)
    # detail layer: S2 high-pass in the luminance, stored beside the masks for the terrain bump
    Image.fromarray(masks).save(f"{OUT}/masks.png", compress_level=1)
    np.save(p_m, masks)
    # Canopy cover at the 10 m map scale: lidar where it exists, Sentinel vegetation elsewhere
    can = veg.copy()
    hr_d = np.load(f"{RAW}/hrdem_site/dtm.npy", mmap_mode="r")
    hr_s = np.load(f"{RAW}/hrdem_site/dsm.npy", mmap_mode="r")
    hw, hs, he, hn = C.HR_BOX
    chm = np.array(hr_s, np.float32) - np.array(hr_d, np.float32)
    val = np.isfinite(chm)
    chm = np.clip(np.nan_to_num(chm), 0, 40)
    cov = ndi.uniform_filter(np.clip((chm - 3) / 6, 0, 1), 7)[2::5, 2::5]
    v10 = val[2::5, 2::5]
    r0, c0 = (N0 - hn) // 10, (hw - W0) // 10
    sub = can[r0:r0 + cov.shape[0], c0:c0 + cov.shape[1]]
    sub[v10] = cov[v10]
    Image.fromarray((np.clip(can, 0, 1) * 255).astype(np.uint8)).save(f"{OUT}/canopy.png", compress_level=1)
    np.save(f"{OUT}/built.npy", built)
    np.save(f"{OUT}/ndvi.npy", ndvi.astype(np.float16))
    np.save(f"{OUT}/ndwi.npy", ndwi.astype(np.float16))
    return masks


def main():
    l0c = tuple(float(v) for v in sys.argv[1:3]) if len(sys.argv) > 2 else (-500.0, -300.0)
    l0h = tuple(float(v) for v in sys.argv[3:5]) if len(sys.argv) > 4 else (1500.0, 1500.0)
    z30, hr8, valid8 = build_dem()
    if os.path.exists(f"{OUT}/masks.npy") and "--remap" not in sys.argv:
        masks = np.load(f"{OUT}/masks.npy")
    else:
        masks = build_maps(z30)
    build_levels(z30, hr8, valid8, l0c, l0h, masks)


if __name__ == "__main__":
    main()
