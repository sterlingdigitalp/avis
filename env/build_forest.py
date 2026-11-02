#!/usr/bin/env python3
"""Tree list for the valley. Run in out/env/.venv with -I. Writes out/env/terrain/forest.npz.

Inside lidar coverage every tree top is a real local maximum of the canopy height model (DSM minus DTM).
Outside lidar, stems are scattered from Sentinel-2 vegetation cover with height driven by elevation.
Species follow elevation, aspect and stand-scale noise. Trees never stand on water, snow or bare rock.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy import ndimage as ndi

import config as C
import ground
import trees

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "out/env/raw")
OUT = os.path.join(ROOT, "out/env/terrain")
OE, ON = C.ORIGIN_UTM
W0, S0, E0, N0 = C.AOI
RADIUS = 13000.0  # stems beyond this radius are terrain colour only
rng = np.random.default_rng(2022)

masks = np.load(f"{OUT}/masks.npy", mmap_mode="r")
built = np.load(f"{OUT}/built.npy", mmap_mode="r")
z30 = np.load(f"{OUT}/fused30.npy")
meta_z = __import__("json").load(open(f"{OUT}/meta.json"))["origin_z"]


def sample_masks(e, n):
    col = ((e - W0) / 10).astype(int).clip(0, masks.shape[1] - 1)
    row = ((N0 - n) / 10).astype(int).clip(0, masks.shape[0] - 1)
    return masks[row, col], built[row, col]


def dem(e, n):
    col = (e - W0) / 30 - 0.5
    row = (N0 - n) / 30 - 0.5
    return ndi.map_coordinates(z30, [row, col], order=1, mode="nearest")


def lidar_tops():
    dtm = np.load(f"{RAW}/hrdem_site/dtm.npy", mmap_mode="r")
    dsm = np.load(f"{RAW}/hrdem_site/dsm.npy", mmap_mode="r")
    hw, hs, he, hn = C.HR_BOX
    es, ns, hs_, zs = [], [], [], []
    tile = 1500
    H, W = dtm.shape
    for r0 in range(0, H, tile):
        for c0 in range(0, W, tile):
            r1, c1 = min(r0 + tile + 12, H), min(c0 + tile + 12, W)
            a = np.array(dtm[r0:r1, c0:c1], dtype=np.float32)
            b = np.array(dsm[r0:r1, c0:c1], dtype=np.float32)
            if not np.isfinite(a).any():
                continue
            chm = np.nan_to_num(b - a, nan=0.0).clip(0, 60)
            sm = ndi.gaussian_filter(chm, 0.8)
            # search window grows with height, as crowns do
            maxf_small = ndi.maximum_filter(sm, size=3)
            maxf_big = ndi.maximum_filter(sm, size=5)
            peak = (sm == maxf_small) & (sm > 2.0) & ((sm >= maxf_big) | (sm < 12))
            rr, cc = np.nonzero(peak)
            inside = (rr < tile) & (cc < tile)
            rr, cc = rr[inside], cc[inside]
            es.append(hw + (c0 + cc + 0.5) * 2 + rng.uniform(-1, 1, len(rr)))
            ns.append(hn - (r0 + rr + 0.5) * 2 + rng.uniform(-1, 1, len(rr)))
            hs_.append(sm[rr, cc])
            zs.append(a[rr, cc])
            # The 2 m mosaic merges neighbouring crowns, so its maxima give only the dominant stems. Infill the
            # co-dominant stems on a jittered 4 m lattice wherever the lidar shows canopy, at the height it measured.
            ee = hw + (c0 + 0.5) * 2
            nn = hn - (r0 + 0.5) * 2
            dist = np.hypot(ee + (min(tile, c1 - c0)) - OE, nn - (min(tile, r1 - r0)) - ON)
            target = np.interp(dist, [1500, 3500, 6000, 11000], [560, 430, 260, 170])  # stems per hectare
            lat_r, lat_c = np.mgrid[0:min(tile, r1 - r0):2, 0:min(tile, c1 - c0):2]
            lat_r, lat_c = lat_r.ravel(), lat_c.ravel()
            canopy = np.clip((sm[lat_r, lat_c] - 3.5) / 6.0, 0, 1)
            pick = rng.random(lat_r.shape) < canopy * target * 16e-4
            pr, pc = lat_r[pick] + rng.integers(0, 2, pick.sum()), lat_c[pick] + rng.integers(0, 2, pick.sum())
            pr, pc = pr.clip(0, sm.shape[0] - 1), pc.clip(0, sm.shape[1] - 1)
            es.append(hw + (c0 + pc + 0.5) * 2 + rng.uniform(-1.5, 1.5, len(pr)))
            ns.append(hn - (r0 + pr + 0.5) * 2 + rng.uniform(-1.5, 1.5, len(pr)))
            hs_.append(sm[pr, pc] * rng.uniform(0.62, 0.98, len(pr)))
            zs.append(a[pr, pc])
    e, n, h, z = np.concatenate(es), np.concatenate(ns), np.concatenate(hs_), np.concatenate(zs)
    ok = np.isfinite(z)
    e, n, h, z = e[ok], n[ok], h[ok], z[ok]
    # 2 m smoothing clips narrow spires by a metre or two: lidar returns rarely hit the very tip
    h = h * 1.07 + 0.6
    print("lidar tops", len(e), "height p50/p95", np.percentile(h, [50, 95]), flush=True)
    return e, n, h, z


def lidar_valid_map():
    dtm = np.load(f"{RAW}/hrdem_site/dtm.npy", mmap_mode="r")
    return np.isfinite(np.array(dtm[::5, ::5]))  # 10 m


def cover_at(x, y):
    """Crisp cover (0-1) from cover_l1.png at Blender (x, y); NaN outside its extent."""
    from PIL import Image
    import json
    cm = json.load(open(f"{OUT}/cover_l1.json"))
    img = np.asarray(Image.open(f"{OUT}/cover_l1.png"), np.float32) / 255.0
    ext, res = cm["ext"], cm["res"]
    c = (x - ext[0]) / res - 0.5
    r = (ext[3] - y) / res - 0.5
    v = ndi.map_coordinates(img, [r, c], order=1, mode="nearest")
    inside = (x > ext[0]) & (x < ext[2]) & (y > ext[1]) & (y < ext[3])
    return np.where(inside, v, np.nan)


def noise_field(e, n, wl, seed):
    """Smooth stand-scale noise at scattered points, from a coarse random lattice."""
    r = np.random.default_rng(seed)
    grid = r.standard_normal((400, 400))
    grid = ndi.gaussian_filter(grid, 1.2, mode="wrap")
    grid /= grid.std()
    return ndi.map_coordinates(grid, [(e / wl) % 400, (n / wl) % 400], order=1, mode="wrap")


def scatter(valid_lidar):
    """Stems outside the lidar, inside RADIUS: inhomogeneous Poisson from vegetation cover, thinned into clumps."""
    box = (OE - RADIUS, ON - RADIUS, OE + RADIUS, ON + RADIUS)
    cell = 10.0
    r0 = int((N0 - box[3]) / cell)
    r1 = int((N0 - box[1]) / cell)
    c0 = int((box[0] - W0) / cell)
    c1 = int((box[2] - W0) / cell)
    veg = masks[r0:r1, c0:c1, 1].astype(np.float32) / 255.0
    ys, xs = np.mgrid[0:veg.shape[0], 0:veg.shape[1]]
    e = box[0] + (xs + 0.5) * cell
    n = box[3] - (ys + 0.5) * cell
    # exclude the lidar window where lidar is valid
    hw, hs, he, hn = C.HR_BOX
    li = (e > hw) & (e < he) & (n > hs) & (n < hn)
    hc = ((e[li] - hw) / 10).astype(int).clip(0, valid_lidar.shape[1] - 1)
    hr = ((hn - n[li]) / 10).astype(int).clip(0, valid_lidar.shape[0] - 1)
    excl = np.zeros(veg.shape, bool)
    excl[li] = valid_lidar[hr, hc]
    dist = np.hypot(e - OE, n - ON)
    # expected stems per 10 m cell: closed forest is roughly 1000 stems/ha at this altitude = 10 per cell;
    # only canopy-forming stems matter at these ranges, so use a third
    lam = 3.2 * np.clip((veg - 0.15) / 0.55, 0, 1) ** 1.2
    # inside the L1 area use the crisp cover map the terrain shader paints, so stems stop where the forest colour
    # stops: a hard edge at the foot of scree and cliff, not a thinning fringe
    cov = cover_at(e - OE, n - ON)
    lam = np.where(np.isfinite(cov), 3.2 * np.clip((cov - 0.4) / 0.2, 0, 1) * np.clip((veg - 0.1) / 0.3, 0.6, 1), lam)
    lam[excl] = 0
    lam[dist > RADIUS] = 0
    # thin with distance: beyond 6 km single stems are sub-pixel and clusters suffice
    lam *= np.clip(1.0 - (dist - 6000) / 9000, 0.35, 1)
    count = rng.poisson(lam)
    idx = np.nonzero(count)
    rep = count[idx]
    ee = np.repeat(e[idx], rep) + rng.uniform(-5, 5, rep.sum())
    nn = np.repeat(n[idx], rep) + rng.uniform(-5, 5, rep.sum())
    zz = dem(ee, nn)
    # height: tall in the valley, stunted toward the treeline (~2350 m) with stand-level variation
    base = np.interp(zz, [1300, 1700, 2000, 2200, 2350, 2450], [24, 23, 20, 13, 7, 3.5])
    hh = base * np.clip(1 + 0.22 * noise_field(ee, nn, 180.0, 4) + rng.normal(0, 0.2, len(ee)), 0.3, 1.5)
    print("scattered", len(ee), flush=True)
    return ee, nn, hh, zz


def build_under(te, tn, th):
    """Ground layer over the L0 area: grass, shrubs, logs, saplings, boulders. Driven by lidar canopy and Sentinel masks."""
    L = np.load(f"{OUT}/level_l0.npz")
    ext, res, zg = L["ext"], float(L["res"]), L["z"]
    x0, y0, x1, y1 = ext
    dtm = np.load(f"{RAW}/hrdem_site/dtm.npy", mmap_mode="r")
    dsm = np.load(f"{RAW}/hrdem_site/dsm.npy", mmap_mode="r")
    hw, hs, he, hn = C.HR_BOX
    r0, r1 = int((hn - (y1 + ON)) / 2), int((hn - (y0 + ON)) / 2)
    c0, c1 = int((x0 + OE - hw) / 2), int((x1 + OE - hw) / 2)
    chm = np.nan_to_num(np.array(dsm[r0:r1, c0:c1], np.float32) - np.array(dtm[r0:r1, c0:c1], np.float32))
    chm = np.clip(ndi.gaussian_filter(chm, 1.0), 0, 60)
    cover = np.clip(ndi.uniform_filter(chm, 7) / 12.0, 0, 1)
    gy, gx = np.gradient(zg[:-1, :-1] if zg.shape[0] > chm.shape[0] else zg, 2.0)
    slope = np.hypot(gx, gy)
    H, W = chm.shape
    slope = slope[:H, :W]
    ys, xs = np.mgrid[0:H, 0:W]
    x = x0 + (xs + 0.5) * 2.0
    y = y1 - (ys + 0.5) * 2.0
    m, b = sample_masks(x.ravel() + OE, y.ravel() + ON)
    veg = (m[..., 1] / 255.0).reshape(H, W)
    water = (m[..., 2] / 255.0).reshape(H, W)
    built = b.reshape(H, W)
    from PIL import Image
    wl = np.asarray(Image.open(f"{OUT}/water_l0.png"))[..., 0].astype(np.float32) / 255.0
    wl = wl[: water.shape[0], : water.shape[1]]
    wl = np.pad(wl, ((0, water.shape[0] - wl.shape[0]), (0, water.shape[1] - wl.shape[1])))
    water = np.maximum(water, wl)
    # cobble and driftwood run right to the 2 m lidar waterline
    c_dist = ndi.distance_transform_edt(wl < 0.4) * 2.0
    water = ndi.maximum_filter(water, size=3)  # keep a metre or two of clear bank
    w_dist = ndi.distance_transform_edt(water < 0.3) * 2.0
    land = (water < 0.3) & (slope < 1.1)
    open_ = 1 - cover
    stand = noise_field(x, y, 9.0, 41)
    patch = np.clip(0.55 + 0.45 * noise_field(x, y, 0.8, 42), 0, 1)
    gravel = (veg < 0.22) & (w_dist < 60) & land
    # expected per 2 m cell
    groups = []

    def emit(lam, kinds, zscale, yaw=True):
        k = rng.poisson(np.clip(lam, 0, 40))
        idx = np.nonzero(k)
        rep = k[idx]
        px = np.repeat(x[idx], rep) + rng.uniform(-1, 1, rep.sum())
        py = np.repeat(y[idx], rep) + rng.uniform(-1, 1, rep.sum())
        kind = rng.choice(kinds, rep.sum())
        sc = rng.uniform(*zscale, rep.sum()).astype(np.float32)
        groups.append((px, py, kind, sc))

    un = {v[0]: i for i, v in enumerate(ground.UNDER)}
    gi = [un["grass_a"], un["grass_b"], un["grass_c"]]
    si = [un["shrub_a"], un["shrub_b"], un["shrub_c"]]
    ri = [un["rock_a"], un["rock_b"], un["rock_c"]]
    mi = [un["meadow_a"], un["meadow_b"], un["meadow_c"]]
    ci = [un["cobble_a"], un["cobble_b"], un["cobble_c"]]
    di = [un["drift_a"], un["drift_b"], un["drift_c"]]
    meadow = np.clip(open_ * 1.4 - 0.25, 0, 1) * np.clip((veg - 0.12) / 0.2, 0, 1) * land * ~gravel * (slope < 0.35)
    # Cured late-September grass: overlapping carpet tiles on the open flats (one per ~2.6 m2 gives full cover),
    # single tufts in the forest openings and at the meadow edges
    emit(meadow * 4.0 / 2.6, mi, (0.8, 1.2))
    emit((3.0 * open_ ** 1.5 * np.clip(veg + 0.25, 0, 1) * (1 - meadow) + 0.8) * patch * land * ~gravel, gi, (0.6, 1.25))
    # river cobble over the bars and banks, densest near the water; bleached driftwood stranded near the waterline
    bar = (veg < 0.22) & (c_dist > 0) & (c_dist < 60) & (slope < 0.6)
    near_w = np.clip((45 - c_dist) / 30, 0, 1)
    emit((bar * 4.0 / 2.2 + (~bar & (c_dist > 0)) * 1.2 * np.clip((8 - c_dist) / 6, 0, 1)) * near_w * (slope < 0.6), ci, (0.7, 1.15))
    emit(bar * np.clip((18 - c_dist) / 14, 0, 1) * 0.014 * (slope < 0.3), di, (0.6, 1.15))
    emit((0.035 + 0.11 * cover * patch) * land, si, (0.6, 1.2))
    emit(0.006 * (0.1 + cover) * land, [un["log_a"], un["log_b"]], (0.7, 1.15))
    emit(0.02 * cover * land, [un["sap_a"], un["sap_b"]], (0.8, 1.8))
    emit(gravel * 0.9 + 0.01 * land, ri, (0.05, 0.28))
    # large angular boulders on steep ground
    emit(0.06 * np.clip(slope - 0.7, 0, 1) * (cover < 0.4), ri, (0.3, 1.3))
    px = np.concatenate([g[0] for g in groups])
    py = np.concatenate([g[1] for g in groups])
    kind = np.concatenate([g[2] for g in groups]).astype(np.int16)
    sc = np.concatenate([g[3] for g in groups])
    pz = ndi.map_coordinates(zg, [(y1 - py) / 2.0, (px - x0) / 2.0], order=1)
    np.savez(f"{OUT}/under.tmp.npz", x=px.astype(np.float32), y=py.astype(np.float32), z=pz.astype(np.float32), kind=kind, scale=sc,
             rot=rng.uniform(0, 2 * np.pi, len(px)).astype(np.float32))
    os.replace(f"{OUT}/under.tmp.npz", f"{OUT}/under.npz")
    print("understorey instances", len(px), {v[0]: int((kind == i).sum()) for i, v in enumerate(ground.UNDER)}, flush=True)


def main():
    e, n, h, z = lidar_tops()
    valid = lidar_valid_map()
    # filter lidar tops: must stand on vegetation; buildings have tall DSM but no chlorophyll
    m, b = sample_masks(e, n)
    veg = m[..., 1] / 255.0
    water = m[..., 2] > 128
    snow = m[..., 0] > 128
    keep = ~water & ~snow & ((veg > 0.12) | (h > 8))
    keep &= ~(b & (h > 2) & (rng.random(len(e)) < 0.0))
    e, n, h, z = e[keep], n[keep], h[keep], z[keep]
    print("lidar tops kept", len(e), flush=True)
    src = np.zeros(len(e), np.uint8)
    e2, n2, h2, z2 = scatter(valid)
    m2, _ = sample_masks(e2, n2)
    k2 = (m2[..., 2] < 128) & (m2[..., 0] < 128)
    e2, n2, h2, z2 = e2[k2], n2[k2], h2[k2], z2[k2]
    e = np.concatenate([e, e2])
    n = np.concatenate([n, n2])
    h = np.concatenate([h, h2])
    z = np.concatenate([z, z2])
    src = np.concatenate([src, np.ones(len(e2), np.uint8)])

    # aspect and moisture from the 30 m DEM
    col = (e - W0) / 30 - 0.5
    row = (N0 - n) / 30 - 0.5
    gy, gx = np.gradient(z30, 30.0)
    dzdx = ndi.map_coordinates(gx, [row, col], order=1, mode="nearest")
    dzdn = -ndi.map_coordinates(gy, [row, col], order=1, mode="nearest")
    # southness/westness of the slope: dry warm aspects carry Douglas fir at low elevation
    south = np.clip(dzdn * 1.4, -1, 1)
    west = np.clip(dzdx * 1.4, -1, 1)
    warm = 0.5 * south + 0.5 * west
    stand = noise_field(e, n, 70.0, 11)  # 200-400 m stands
    stand2 = noise_field(e, n, 25.0, 12)
    # species scores
    z_m = z
    s_pine = np.interp(z_m, [1280, 1700, 1850, 2050], [1.25, 1.35, 0.6, 0.0]) + 0.7 * stand
    s_spruce = np.interp(z_m, [1280, 1500, 1900, 2200, 2400], [0.55, 0.55, 1.0, 0.9, 0.5]) - 0.3 * stand + 0.25 * stand2
    s_fir = np.interp(z_m, [1400, 1800, 2100, 2350], [0.05, 0.5, 1.05, 1.1]) - 0.3 * stand - 0.25 * stand2
    s_dou = np.interp(z_m, [1280, 1500, 1680, 1800], [0.4, 0.45, 0.2, 0.0]) * (0.2 + 1.6 * np.clip(warm + 0.3, 0, 1))
    s_lar = np.interp(z_m, [1950, 2150, 2400], [0.0, 0.5, 0.75]) * np.clip(0.3 - warm, 0, 1) * 1.3
    scores = np.stack([s_spruce, s_fir, s_pine, s_dou, s_lar], 1).clip(0, None)
    # Trembling aspen in gold on the valley floor (Parks Canada montane list): clonal stands, high threshold on stand noise
    gum = -np.log(-np.log(rng.uniform(1e-6, 1 - 1e-6, scores.shape))) * 0.35
    sp_idx = np.argmax(scores + gum, axis=1)
    # Aspen grows in clones: whole patches of 50-150 m, decided by the patch, not tree by tree
    clone = noise_field(e, n, 22.0, 31) + 0.25 * noise_field(e, n, 4.0, 32)
    elev_ok = np.interp(z_m, [1280, 1550, 1700], [1.0, 0.7, 0.0])
    is_aspen = (clone * elev_ok > 1.55) & (rng.random(len(e)) < 0.85)
    sp_idx = np.where(is_aspen, 5, sp_idx)
    names = ["engelmann", "subalpine", "lodgepole", "douglas", "larch", "aspen"]
    variants = trees.VARIANTS
    v_by_sp = {nm: [(i, v) for i, v in enumerate(variants) if v[1] == nm] for nm in names + ["snag"]}
    # dead standing snags: 3-5 % of canopy trees, more on the warm mature stands, patchy
    p_snag = np.clip(0.022 + 0.02 * stand2, 0.004, 0.06) * (h > 7)
    is_snag = rng.random(len(e)) < p_snag
    var = np.zeros(len(e), np.int16)
    scale = np.ones(len(e), np.float32)
    for si, nm in enumerate(names):
        sel = np.nonzero((sp_idx == si) & ~is_snag)[0]
        vs = v_by_sp[nm]
        heights = np.array([v[2] for _, v in vs])
        for k in np.array_split(sel, max(1, len(sel) // 5000)):
            # pick the size class nearest the tree's real height, with some spread in the choice
            d = np.abs(h[k, None] - heights) * rng.uniform(0.75, 1.25, (len(k), len(heights)))
            c = np.argmin(d, axis=1)
            var[k] = [vs[j][0] for j in c]
            scale[k] = h[k] / heights[c]
    sel = np.nonzero(is_snag)[0]
    vs = v_by_sp["snag"]
    heights = np.array([v[2] for _, v in vs])
    c = np.argmin(np.abs(h[sel, None] - heights), axis=1)
    var[sel] = [vs[j][0] for j in c]
    scale[sel] = h[sel] / heights[c]
    scale = np.clip(scale, 0.45, 1.6)
    rot = rng.uniform(0, 2 * np.pi, len(e)).astype(np.float32)
    # exact base height on the render meshes, finest level that covers each stem
    zb = np.full(len(e), np.nan, np.float32)
    xr, yr = e - OE, n - ON
    for lev in ("l0", "l1", "l2", "l3"):
        L = np.load(f"{OUT}/level_{lev}.npz")
        ext, res, zz = L["ext"], float(L["res"]), L["z"]
        ins = np.isnan(zb) & (xr > ext[0] + res) & (xr < ext[2] - res) & (yr > ext[1] + res) & (yr < ext[3] - res)
        zb[ins] = ndi.map_coordinates(zz, [(ext[3] - yr[ins]) / res, (xr[ins] - ext[0]) / res], order=1)
    np.savez(f"{OUT}/forest.tmp.npz",
             x=(e - OE).astype(np.float32), y=(n - ON).astype(np.float32),
             h=h.astype(np.float32), var=var, scale=scale, rot=rot, src=src,
             z=zb)
    os.replace(f"{OUT}/forest.tmp.npz", f"{OUT}/forest.npz")
    build_under(e, n, h)
    counts = {nm: int(((sp_idx == si) & ~is_snag).sum()) for si, nm in enumerate(names)}
    counts["snag"] = int(is_snag.sum())
    print("total stems", len(e), counts, flush=True)


if __name__ == "__main__":
    main()
