#!/usr/bin/env python3
"""Light backdrop for the three.js page: out/env/web/. Run in out/env/.venv with -I. env/export_web_render.py makes the two images that need Cycles.

  terrain.glb         low-poly chunk, external texture terrain_albedo.jpg (Y up, world frame of SPEC)
  terrain_albedo.jpg  real Sentinel-2 colour, illumination removed
  trees.bin           impostor instances near the corridor: x, y, z, height f32, type u8 + 3 pad; little-endian
  manifest.json       file list, sizes, instance layout, sun, atlas layout
"""
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

import config as C
import trees

Image.MAX_IMAGE_PIXELS = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TER = os.path.join(ROOT, "out/env/terrain")
WEB = os.path.join(ROOT, "out/env/web")
os.makedirs(WEB, exist_ok=True)
OE, ON = C.ORIGIN_UTM
W0, S0, E0, N0 = C.AOI


def write_glb(path, positions, normals, uvs, indices, texture_uri):
    blobs = [positions.astype("<f4").tobytes(), normals.astype("<f4").tobytes(), uvs.astype("<f4").tobytes(), indices.astype("<u4").tobytes()]
    views, off = [], 0
    for i, b in enumerate(blobs):
        v = {"buffer": 0, "byteOffset": off, "byteLength": len(b), "target": 34963 if i == 3 else 34962}
        views.append(v)
        off += (len(b) + 3) & ~3
    gltf = {
        "asset": {"version": "2.0", "generator": "avis env/export_web.py"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0, "name": "terrain"}],
        "meshes": [{"name": "terrain", "primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1, "TEXCOORD_0": 2}, "indices": 3, "material": 0}]}],
        "materials": [{"name": "terrain", "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}, "metallicFactor": 0.0, "roughnessFactor": 0.92}}],
        "textures": [{"source": 0, "sampler": 0}],
        "samplers": [{"magFilter": 9729, "minFilter": 9987, "wrapS": 33071, "wrapT": 33071}],
        "images": [{"uri": texture_uri}],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": len(positions), "type": "VEC3", "min": positions.min(0).tolist(), "max": positions.max(0).tolist()},
            {"bufferView": 1, "componentType": 5126, "count": len(normals), "type": "VEC3"},
            {"bufferView": 2, "componentType": 5126, "count": len(uvs), "type": "VEC2"},
            {"bufferView": 3, "componentType": 5125, "count": len(indices), "type": "SCALAR"},
        ],
        "bufferViews": views,
        "buffers": [{"byteLength": off}],
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    bin_ = b"".join(b + b"\0" * (((len(b) + 3) & ~3) - len(b)) for b in blobs)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(bin_)))
        f.write(struct.pack("<II", len(js), 0x4E4F534A))
        f.write(js)
        f.write(struct.pack("<II", len(bin_), 0x004E4942))
        f.write(bin_)


def main():
    corr = json.load(open(os.path.join(ROOT, "data/env/corridor.json")))
    wp = np.array(corr["waypoints"])
    cx, cy = float(wp[:, 0].mean()), float(-wp[:, 2].mean())
    half = 7000.0
    n = 201
    z30 = np.load(f"{TER}/fused30.npy")
    oz = corr["elevation_m_at_origin"]
    xs = np.linspace(cx - half, cx + half, n)
    ys = np.linspace(cy + half, cy - half, n)  # north to south
    xx, yy = np.meshgrid(xs, ys)
    col = (xx + OE - W0) / 30 - 0.5
    row = (N0 - (yy + ON)) / 30 - 0.5
    z = ndi.map_coordinates(ndi.gaussian_filter(z30, 0.8), [row, col], order=1) - oz
    # world frame x east, y up, z south
    P = np.stack([xx, z, -yy], -1).reshape(-1, 3)
    dx = xs[1] - xs[0]
    gy, gx = np.gradient(z, dx)
    # gy is d/d(row) = d/d(south); gx = d/d(east). Normal in (x, y, z) = (-dz/dx, 1, -dz/dsouth).
    N = np.stack([-gx, np.ones_like(gx), -gy], -1)
    N /= np.linalg.norm(N, axis=2, keepdims=True)
    uv = np.stack([np.meshgrid(np.linspace(0, 1, n), np.linspace(0, 1, n))], 0)[0]
    UV = np.stack([uv[0], uv[1]], -1).reshape(-1, 2)  # glTF v runs top to bottom of the image, row 0 at north
    idx = np.arange(n * n).reshape(n, n)
    a, b, c, d = idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:], idx[1:, :-1]
    F = np.concatenate([np.stack([a, d, c], -1).reshape(-1, 3), np.stack([a, c, b], -1).reshape(-1, 3)]).ravel()
    write_glb(f"{WEB}/terrain.glb", P, N.reshape(-1, 3), UV, F, "terrain_albedo.jpg")
    # texture crop
    im = Image.open(f"{TER}/albedo.png")
    x0 = int((cx - half + OE - W0) / 10)
    x1 = int((cx + half + OE - W0) / 10)
    y0 = int((N0 - (cy + half + ON)) / 10)
    y1 = int((N0 - (cy - half + ON)) / 10)
    crop = im.crop((x0, y0, x1, y1)).resize((1536, 1536), Image.LANCZOS)
    crop.save(f"{WEB}/terrain_albedo.jpg", quality=82, optimize=True)
    # impostor instances within 900 m of the corridor
    f = np.load(f"{TER}/forest.npz")
    x, y, zt, h, var = f["x"], f["y"], f["z"], f["h"], f["var"]
    path = np.stack([wp[:, 0], -wp[:, 2]], 1)
    px = np.interp(np.arange(0, len(path) - 1, 0.25), np.arange(len(path)), path[:, 0])
    py = np.interp(np.arange(0, len(path) - 1, 0.25), np.arange(len(path)), path[:, 1])
    near = (x > px.min() - 900) & (x < px.max() + 900) & (y > py.min() - 900) & (y < py.max() + 900)
    x, y, zt, h, var = x[near], y[near], zt[near], h[near], var[near]
    d = np.full(len(x), 1e9)
    for a_, b_ in zip(px, py):
        d = np.minimum(d, np.hypot(x - a_, y - b_))
    keep = (d < 900) & (np.random.default_rng(1).random(len(x)) < np.clip(1.25 - d / 700, 0.18, 1))
    x, y, zt, h, var = x[keep], y[keep], zt[keep], h[keep], var[keep]
    if len(x) > 42000:
        s = np.random.default_rng(2).choice(len(x), 42000, replace=False)
        x, y, zt, h, var = x[s], y[s], zt[s], h[s], var[s]
    groups = {"engelmann": 0, "subalpine": 1, "lodgepole": 2, "douglas": 3, "larch": 4, "snag": 5, "aspen": 6}
    typ = np.array([groups[trees.VARIANTS[int(v)][1]] for v in var], np.uint8)
    rec = np.zeros(len(x), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("h", "<f4"), ("t", "u1"), ("p", "u1", 3)])
    rec["x"], rec["y"], rec["z"], rec["h"], rec["t"] = x, zt, -y, h, typ
    rec.tofile(f"{WEB}/trees.bin")
    files = {n_: os.path.getsize(f"{WEB}/{n_}") for n_ in sorted(os.listdir(WEB)) if n_ != "manifest.json"}
    atlas = None
    p_a = f"{WEB}/impostors.json"
    if os.path.exists(p_a):
        atlas = json.load(open(p_a))
    manifest = {
        "note": "Backdrop for the live page. World frame of SPEC: x east, y up, z south, metres from the corridor origin.",
        "place": corr["place"],
        "sun_dir": corr["sun_dir"],
        "terrain": {"file": "terrain.glb", "extent_m": half * 2, "grid": n, "cell_m": float(dx)},
        "trees": {"file": "trees.bin", "count": int(len(x)), "bytes_per_instance": 20, "layout": "x y z height f32le, type u8, 3 pad",
                  "types": groups, "atlas": atlas},
        "panorama": {"file": "sky_pano.jpg", "projection": "equirectangular", "taken_from": "corridor mid-point, world frame",
                     "note": "use as scene.background; horizon includes the far ranges beyond the terrain chunk"},
        "files_bytes": files,
        "total_bytes": sum(files.values()),
    }
    json.dump(manifest, open(f"{WEB}/manifest.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in manifest.items() if k in ("files_bytes", "total_bytes")}))


if __name__ == "__main__":
    main()
