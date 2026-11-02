#!/usr/bin/env python3
"""Structured C-grid for a 2D airfoil, written directly as an OpenFOAM polyMesh.

Built by the square-root map z = zeta^2 with the singularity inside the nose: the airfoil and wake become
a nearly flat line, and straight columns above it map back to near-orthogonal lines that cannot fold.
One cell thick in z with empty front and back.

    cgrid.py <selig.dat> <outDir> --level 1.0 [--radius 40]

Level scales every length: 1.0 is the medium grid, 1.5 fine, 0.667 coarse.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airfoil import Surface, clustered_stations, normalise, read_selig, thickness_camber

SPAN = 0.1
BASE = dict(n_surface=180, n_wake=50, n_normal=120, y1=7.0e-5, h_le=1.0e-3, h_te=2.0e-3, h_wake_far=0.02)


def geometric_heights(y1, n, total):
    """Cumulative heights 0..total over n cells with first cell y1 and constant ratio."""
    lo, hi = 1.0 + 1e-9, 2.0
    for _ in range(200):
        r = 0.5 * (lo + hi)
        if y1 * (r ** n - 1) / (r - 1) > total:
            hi = r
        else:
            lo = r
    r = 0.5 * (lo + hi)
    out, h, y = [0.0], y1, 0.0
    for _ in range(n):
        y += h
        out.append(y)
        h *= r
    out[-1] = total
    return out, r


def wall_curve(path, level, radius):
    raw = read_selig(path)
    pts, gap = normalise(raw)
    surf = Surface(pts)
    n_s = max(8, round(BASE["n_surface"] * level))
    h_le, h_te = BASE["h_le"] / level, BASE["h_te"] / level
    s_le = surf.s[surf.le_index]
    s_end = surf.s[-1]
    # Selig runs upper TE -> LE -> lower TE; the C-grid curve starts on the lower wake, so reverse it.
    lower = [surf.at(s_le + s) for s in clustered_stations(s_end - s_le, n_s, h_le, h_te)]  # LE -> lower TE
    upper = [surf.at(s_le - s) for s in clustered_stations(s_le, n_s, h_le, h_te)]  # LE -> upper TE
    wall = list(reversed(lower)) + upper[1:]
    wall[0] = wall[-1] = (1.0, 0.0)
    # Wake stations: geometric from the TE spacing out to the boundary.
    n_w = max(8, round(BASE["n_wake"] * level))
    wake_x, _ = geometric_heights(h_te, n_w, radius)
    wake = [(1.0 + x, 0.0) for x in wake_x[1:]]
    curve = list(reversed(wake)) + wall + wake
    meta = dict(te_gap_closed=gap, n_surface=n_s, n_wake=n_w, thickness=None, camber=None)
    meta["thickness"], meta["camber"] = thickness_camber(pts)
    return curve, n_w, n_w + len(wall) - 1, meta


def arc(xi, eta):
    """Physical arc length along a constant-xi line of z = zeta^2, up to a constant."""
    r = math.hypot(xi, eta)
    return eta * r + xi * xi * math.log(eta + r + 1e-300)


def map_origin(curve, i_tel, i_teu, thickness):
    """Map singularity inside the nose, half the nose radius behind it, so the nose maps nearly flat."""
    wall = curve[i_tel:i_teu + 1]
    i_le = min(range(len(wall)), key=lambda k: wall[k][0])
    x_le = wall[i_le][0]
    x0 = x_le + 0.55 * thickness ** 2
    side = lambda arr: min(arr, key=lambda p: abs(p[0] - x0))[1]
    return x0, 0.5 * (side(wall[:i_le + 1]) + side(wall[i_le:]))


def to_zeta(p, x0, y0, lower):
    """Square root with the branch cut along the wake: Im >= 0, Re < 0 on the lower side."""
    zr, zi = p[0] - x0, p[1] - y0
    r = math.hypot(zr, zi)
    theta = math.atan2(zi, zr)
    # Upper side and upper wake: theta near 0 up to pi. Lower side and lower wake: pi up to near 2 pi.
    if lower:
        if theta < 0.5 * math.pi:
            theta += 2 * math.pi
    elif theta < -0.5 * math.pi:
        theta += 2 * math.pi
    m = math.sqrt(r)
    return m * math.cos(0.5 * theta), m * math.sin(0.5 * theta)


def march(curve, i_tel, i_teu, level, radius, thickness):
    """Square-root C-grid: straight columns in the mapped plane, geometric in physical arc length."""
    y1 = BASE["y1"] / level
    nj = max(30, round(BASE["n_normal"] * level))
    ni = len(curve)
    hw = BASE["h_wake_far"] / level
    x0, y0 = map_origin(curve, i_tel, i_teu, thickness)
    i_le = i_tel + min(range(i_teu - i_tel + 1), key=lambda k: curve[i_tel + k][0])
    zetas = [to_zeta(p, x0, y0, i <= i_le) for i, p in enumerate(curve)]
    for (xa, _), (xb, _) in zip(zetas, zetas[1:]):
        if xb <= xa:
            raise RuntimeError("wall does not map monotonically; move the map origin")
    eta_max = math.sqrt(radius) + max(z[1] for z in zetas)
    grid = [[None] * ni for _ in range(nj + 1)]
    ratio_max = 0.0
    for i, ((xi, eta0), p) in enumerate(zip(zetas, curve)):
        h1 = y1 if i_tel <= i <= i_teu else y1 + (hw - y1) * max(p[0] - 1.0, 0.0) / radius
        base = arc(xi, eta0)
        heights, r = geometric_heights(h1, nj, arc(xi, eta_max) - base)
        ratio_max = max(ratio_max, r)
        grid[0][i] = p
        eta = eta0
        for j in range(1, nj + 1):
            target = base + heights[j]
            # Newton on the monotonic arc function, seeded from the previous layer.
            for _ in range(30):
                speed = 2.0 * math.hypot(xi, eta)
                step = (arc(xi, eta) - target) / speed
                eta -= step
                if abs(step) < 1e-13 * (1.0 + eta):
                    break
            grid[j][i] = ((xi * xi - eta * eta) + x0, 2.0 * xi * eta + y0)
    return grid, ni, nj + 1, y1, ratio_max


def foam_header(cls, obj, note=None):
    n = f'    note        "{note}";\n' if note else ""
    return (f'FoamFile\n{{\n    format      ascii;\n    class       {cls};\n{n}'
            f'    location    "constant/polyMesh";\n    object      {obj};\n}}\n')


def write_polymesh(grid, ni, nj, i_tel, i_teu, out):
    cols, rows = ni - 1, nj - 1
    ncell = cols * rows
    # 2D point ids, with the wake cut merged at j=0.
    pid = {}
    pts2 = []
    for j in range(nj):
        for i in range(ni):
            if j == 0 and i > i_teu:
                pid[(i, j)] = pid[(ni - 1 - i, 0)]
                continue
            pid[(i, j)] = len(pts2)
            pts2.append(grid[j][i])
    npts = len(pts2)

    def cell(i, j):
        return i + j * cols

    def centroid2(i, j):
        c = [grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i]]
        return sum(p[0] for p in c) / 4, sum(p[1] for p in c) / 4

    centres = [None] * ncell
    min_area = 1e300
    for j in range(rows):
        for i in range(cols):
            centres[cell(i, j)] = centroid2(i, j)
            a, b, c, d = grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i]
            area = 0.5 * ((c[0] - a[0]) * (d[1] - b[1]) - (d[0] - b[0]) * (c[1] - a[1]))
            min_area = min(min_area, abs(area))
            if area == 0:
                raise RuntimeError(f"zero-area cell at {i},{j}")

    def quad(p, q, own):
        """Side face from 2D edge p-q; orient the normal out of the owner cell."""
        (px, py), (qx, qy) = pts2[p], pts2[q]
        ox, oy = centres[own]
        mx, my = 0.5 * (px + qx) - ox, 0.5 * (py + qy) - oy
        # Face (p0, q0, q1, p1): normal = (q-p) x z = (dy, -dx).
        if (qy - py) * mx - (qx - px) * my > 0:
            return (2 * p, 2 * q, 2 * q + 1, 2 * p + 1)
        return (2 * q, 2 * p, 2 * p + 1, 2 * q + 1)

    internal = []
    for j in range(rows):
        for i in range(cols):
            c = cell(i, j)
            if i + 1 < cols:
                n = cell(i + 1, j)
                internal.append((c, n, pid[(i + 1, j)], pid[(i + 1, j + 1)]))
            if j + 1 < rows:
                n = cell(i, j + 1)
                internal.append((c, n, pid[(i, j + 1)], pid[(i + 1, j + 1)]))
    # Across the wake cut: lower-wake cell i meets upper-wake cell ni-2-i.
    for i in range(i_tel):
        internal.append((cell(i, 0), cell(ni - 2 - i, 0), pid[(i, 0)], pid[(i + 1, 0)]))
    internal.sort(key=lambda f: (f[0], f[1]))

    faces, owner, neigh = [], [], []
    for c, n, p, q in internal:
        faces.append(quad(p, q, c))
        owner.append(c)
        neigh.append(n)

    patches = []

    def add_patch(name, typ, items):
        start = len(faces)
        for f, o in items:
            faces.append(f)
            owner.append(o)
        patches.append((name, typ, len(items), start))

    wall_items = [(quad(pid[(i, 0)], pid[(i + 1, 0)], cell(i, 0)), cell(i, 0)) for i in range(i_tel, i_teu)]
    far = []
    for j in range(rows):
        far.append((quad(pid[(0, j)], pid[(0, j + 1)], cell(0, j)), cell(0, j)))
    for i in range(cols):
        far.append((quad(pid[(i, nj - 1)], pid[(i + 1, nj - 1)], cell(i, rows - 1)), cell(i, rows - 1)))
    for j in range(rows):
        far.append((quad(pid[(ni - 1, j)], pid[(ni - 1, j + 1)], cell(cols - 1, j)), cell(cols - 1, j)))
    add_patch("airfoil", "wall", wall_items)
    add_patch("farfield", "patch", far)
    sides = []
    for j in range(rows):
        for i in range(cols):
            a, b, c2, d = pid[(i, j)], pid[(i + 1, j)], pid[(i + 1, j + 1)], pid[(i, j + 1)]
            pa, pb, pd = pts2[a], pts2[b], pts2[d]
            nz = (pb[0] - pa[0]) * (pd[1] - pa[1]) - (pb[1] - pa[1]) * (pd[0] - pa[0])
            own = cell(i, j)
            # Front (z=0) normal must be -z, back (z=span) must be +z.
            sides.append((((2 * a, 2 * d, 2 * c2, 2 * b) if nz > 0 else (2 * a, 2 * b, 2 * c2, 2 * d)), own))
            sides.append((((2 * a + 1, 2 * b + 1, 2 * c2 + 1, 2 * d + 1) if nz > 0
                           else (2 * a + 1, 2 * d + 1, 2 * c2 + 1, 2 * b + 1)), own))
    add_patch("frontAndBack", "empty", sides)

    os.makedirs(out, exist_ok=True)
    note = f"nPoints:{2 * npts} nCells:{ncell} nFaces:{len(faces)} nInternalFaces:{len(internal)}"
    with open(os.path.join(out, "points"), "w") as f:
        f.write(foam_header("vectorField", "points"))
        f.write(f"{2 * npts}\n(\n")
        # Interleaved: point 2p is (x, y, 0) and 2p+1 is (x, y, span); the faces rely on this.
        f.writelines(f"({x:.12g} {y:.12g} 0)\n({x:.12g} {y:.12g} {SPAN:g})\n" for x, y in pts2)
        f.write(")\n")
    with open(os.path.join(out, "faces"), "w") as f:
        f.write(foam_header("faceList", "faces"))
        f.write(f"{len(faces)}\n(\n")
        f.writelines(f"4({a} {b} {c} {d})\n" for a, b, c, d in faces)
        f.write(")\n")
    with open(os.path.join(out, "owner"), "w") as f:
        f.write(foam_header("labelList", "owner", note))
        f.write(f"{len(owner)}\n(\n" + "\n".join(map(str, owner)) + "\n)\n")
    with open(os.path.join(out, "neighbour"), "w") as f:
        f.write(foam_header("labelList", "neighbour", note))
        f.write(f"{len(neigh)}\n(\n" + "\n".join(map(str, neigh)) + "\n)\n")
    with open(os.path.join(out, "boundary"), "w") as f:
        f.write(foam_header("polyBoundaryMesh", "boundary"))
        f.write(f"{len(patches)}\n(\n")
        for name, typ, n, start in patches:
            extra = "    inGroups    List<word> 1(wall);\n" if typ == "wall" else ""
            f.write(f"{name}\n{{\n    type        {typ};\n{extra}    nFaces      {n};\n    startFace   {start};\n}}\n")
        f.write(")\n")
    return ncell, min_area, [grid[0][i] for i in range(i_tel, i_teu + 1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("coords")
    ap.add_argument("out")
    ap.add_argument("--level", type=float, default=1.0)
    ap.add_argument("--radius", type=float, default=40.0)
    a = ap.parse_args()
    curve, i_tel, i_teu, meta = wall_curve(a.coords, a.level, a.radius)
    grid, ni, nj, y1, ratio = march(curve, i_tel, i_teu, a.level, a.radius, meta["thickness"])
    meta["ratio_max"] = ratio
    poly = os.path.join(a.out, "polyMesh")
    ncell, min_area, wall = write_polymesh(grid, ni, nj, i_tel, i_teu, poly)
    meta.update(level=a.level, radius=a.radius, cells=ncell, ni=ni, nj=nj, y1=y1, span=SPAN, min_cell_area=min_area,
                coords=os.path.abspath(a.coords))
    with open(os.path.join(a.out, "mesh.json"), "w") as f:
        json.dump(dict(meta, wall=wall), f)
    print(json.dumps(meta))


if __name__ == "__main__":
    main()
