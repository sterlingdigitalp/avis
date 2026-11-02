#!/usr/bin/env python3
"""Closed STL surfaces of a bird at its glide pose: wing and tail (right side) and body. Body frame, metres.

    stl_bird.py <pose.json> <wing.dat> <tail.dat> <outDir>

pose.json comes from cfd/pose.mjs (the physics kinematics). Assumptions that the design file leaves open, stated
here and copied into the case report:
  * wing section: the named section's coordinates laid on each station's chord line and up vector;
  * tail: all-moving V-tail panels, quarter-chord line straight and unswept, glide `tail` angle added to incidence;
  * body: design.body.loft through src/physics/geometry.js bodyMesh when the design has one (designs from version 7);
    otherwise an ellipsoid of the design's length, width and height, nose 0.45 L ahead of x = 0 (build.js noseLen),
    plus an 8 mm keel/boom tube (catalogue part tube-ec-woven-8x6) back to the tail trailing edge.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from airfoil import Surface, normalise, read_selig


def section(path, n=80):
    """Upper and lower y at shared cosine x stations, LE to TE."""
    pts, _ = normalise(read_selig(path))
    surf = Surface(pts)
    dense = surf.dense
    le = surf.le_index
    upper = sorted(dense[:le + 1])
    lower = sorted(dense[le:])

    def interp(arr, x):
        lo, hi = 0, len(arr) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if arr[mid][0] <= x:
                lo = mid
            else:
                hi = mid
        a, b = arr[lo], arr[hi]
        return a[1] if b[0] == a[0] else a[1] + (x - a[0]) / (b[0] - a[0]) * (b[1] - a[1])
    xs = [0.5 * (1 - math.cos(math.pi * k / n)) for k in range(n + 1)]
    x0 = upper[0][0]
    xs = [x0 + (1 - x0) * x for x in xs]
    yu = [interp(upper, x) for x in xs]
    yl = [interp(lower, x) for x in xs]
    yu[0] = yl[0] = 0.5 * (yu[0] + yl[0])
    yu[-1] = yl[-1] = 0.5 * (yu[-1] + yl[-1])
    return xs, yu, yl


def loop(sec, le, cvec, up, chord):
    """Closed section loop: upper TE->LE then lower LE->TE (shared ends removed from the lower run)."""
    xs, yu, yl = sec
    pt = lambda x, y: [le[k] + chord * (x * cvec[k] + y * up[k]) for k in range(3)]
    ring = [pt(x, y) for x, y in zip(reversed(xs), reversed(yu))]
    ring += [pt(x, y) for x, y in zip(xs[1:-1], yl[1:-1])]
    return ring


def loft(rings):
    """Triangles of a closed loft: side quads plus capped ends (strip caps, valid for any airfoil loop)."""
    tris = []
    n = len(rings[0])
    for a, b in zip(rings, rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            tris.append((a[i], b[i], b[j]))
            tris.append((a[i], b[j], a[j]))
    half = n // 2  # index of the LE in the ring
    for ring, flip in ((rings[0], False), (rings[-1], True)):
        # Upper point k (from the TE) pairs with the lower point at the same x, ring[n - k]; ring[0] is the TE.
        for k in range(half):
            u0, u1 = ring[k], ring[k + 1]
            l0 = ring[(n - k) % n]
            l1 = ring[n - k - 1]
            if k == 0:
                t = [(u0, u1, l1)]  # TE: upper and lower meet in one point
            elif k == half - 1:
                t = [(u0, u1, l0)]  # LE: u1 is the leading-edge point itself
            else:
                t = [(u0, u1, l1), (u0, l1, l0)]
            tris.extend([(c, b, a) if flip else (a, b, c) for a, b, c in t])
    return tris


def rodrigues(v, axis, ang):
    c, s = math.cos(ang), math.sin(ang)
    d = sum(a * b for a, b in zip(v, axis))
    cr = [axis[1] * v[2] - axis[2] * v[1], axis[2] * v[0] - axis[0] * v[2], axis[0] * v[1] - axis[1] * v[0]]
    return [v[k] * c + cr[k] * s + axis[k] * d * (1 - c) for k in range(3)]


def body_from_loft(mesh):
    """Closed triangles from geometry.js bodyMesh rings; a ring of zero size is a single apex point."""
    n = mesh["nAround"]
    rings = [mesh["points"][i * n:(i + 1) * n] for i in range(mesh["rings"])]
    size = lambda r: max(math.dist(p, r[0]) for p in r)
    tris = []
    for a, b in zip(rings, rings[1:]):
        for k in range(n):
            j = (k + 1) % n
            if size(a) < 1e-9 and size(b) < 1e-9:
                continue
            if size(a) < 1e-9:
                tris.append((a[0], b[j], b[k]))
            elif size(b) < 1e-9:
                tris.append((a[k], a[j], b[0]))
            else:
                tris += [(a[k], a[j], b[j]), (a[k], b[j], b[k])]
    for ring, first in ((rings[0], True), (rings[-1], False)):
        if size(ring) >= 1e-9:  # flat cap on an open end
            c = [sum(p[i] for p in ring) / n for i in range(3)]
            for k in range(n):
                j = (k + 1) % n
                tris.append((c, ring[j], ring[k]) if first else (c, ring[k], ring[j]))
    return tris


def tail_rings(tail, glide, sec, n_span=16, panel=None):
    dih = tail["dihedral_rad"]
    axis = [0.0, math.sin(dih), math.cos(dih)]
    inc = tail.get("incidence_rad", 0.0) + glide.get("tail", 0.0)
    cvec = rodrigues([-1.0, 0.0, 0.0], axis, inc)
    up = [axis[1] * cvec[2] - axis[2] * cvec[1], axis[2] * cvec[0] - axis[0] * cvec[2], axis[0] * cvec[1] - axis[1] * cvec[0]]
    up = [-u for u in up] if up[1] < 0 else up
    rings = []
    sweep = tail.get("sweep_m", 0.0) if panel is None else (panel["corners"][0][0] - 0.25 * panel["chord_root_m"]) - (
        panel["corners"][1][0] - 0.25 * panel["chord_tip_m"])
    for k in range(n_span + 1):
        f = k / n_span
        c = tail["chord_root_m"] * (1 - (1 - tail["taper"]) * f)
        q = [tail["x_m"] - sweep * f, tail["y_m"] + f * tail["span_each_m"] * axis[1],
             tail["z_m"] + f * tail["span_each_m"] * axis[2]]
        le = [q[i] - 0.25 * c * cvec[i] for i in range(3)]
        rings.append(loop(sec, le, cvec, up, c))
    return rings


def ellipsoid(cx, a, b, c, nu=48, nv=96):
    tris = []
    pts = [[[cx + a * math.cos(math.pi * i / nu), b * math.sin(math.pi * i / nu) * math.cos(2 * math.pi * j / nv),
             c * math.sin(math.pi * i / nu) * math.sin(2 * math.pi * j / nv)] for j in range(nv)] for i in range(nu + 1)]
    for i in range(nu):
        for j in range(nv):
            k = (j + 1) % nv
            p00, p01, p10, p11 = pts[i][j], pts[i][k], pts[i + 1][j], pts[i + 1][k]
            if i > 0:
                tris.append((p00, p10, p01))
            if i < nu - 1:
                tris.append((p01, p10, p11))
    return tris


def tube(x0, x1, y, r, n=24):
    tris = []
    ring = lambda x: [[x, y + r * math.cos(2 * math.pi * j / n), r * math.sin(2 * math.pi * j / n)] for j in range(n)]
    a, b = ring(x0), ring(x1)
    for j in range(n):
        k = (j + 1) % n
        tris += [(a[j], b[k], b[j]), (a[j], a[k], b[k])]
        tris += [([x0, y, 0.0], a[k], a[j]), ([x1, y, 0.0], b[j], b[k])]
    return tris


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            u = [b[k] - a[k] for k in range(3)]
            v = [c[k] - a[k] for k in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            m = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(f" facet normal {n[0] / m:.6e} {n[1] / m:.6e} {n[2] / m:.6e}\n  outer loop\n")
            for p in (a, b, c):
                f.write(f"   vertex {p[0]:.7e} {p[1]:.7e} {p[2]:.7e}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")


def signed_volume(tris):
    return sum((a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]))
               for a, b, c in tris) / 6.0


def main():
    pose = json.load(open(sys.argv[1]))
    wsec, tsec, out = section(sys.argv[2]), section(sys.argv[3]), sys.argv[4]
    os.makedirs(out, exist_ok=True)
    wing = loft([loop(wsec, s["le"], s["chord_vec"], s["up"], s["chord"]) for s in pose["stations"]])
    tail = loft(tail_rings(pose["tail"], pose["glide"], tsec, panel=pose.get("tail_panel")))
    if pose.get("body_mesh"):
        body_tris = body_from_loft(pose["body_mesh"])  # design.body.loft through src/physics/geometry.js
    else:
        body = pose["body"]
        L, W, H = body["length_m"], body["width_m"], body["height_m"]
        t = pose["tail"]
        body_tris = ellipsoid(0.45 * L - 0.5 * L, 0.5 * L, 0.5 * H, 0.5 * W) + tube(
            t["x_m"] - 0.75 * t["chord_root_m"] - 0.005, -0.3 * L, 0.0, 0.004)
    parts = dict(wing=wing, tail=tail, body=body_tris)
    report = {}
    for name, tris in parts.items():
        # Outward normals: flip the whole surface if its enclosed volume comes out negative.
        if signed_volume(tris) < 0:
            tris = [(a, c, b) for a, b, c in tris]
        write_stl(os.path.join(out, name + ".stl"), name, tris)
        report[name] = dict(triangles=len(tris), volume_m3=signed_volume(tris))
    allp = [p for tris in parts.values() for tri in tris for p in tri]
    bounds = dict(min=[min(p[k] for p in allp) for k in range(3)], max=[max(p[k] for p in allp) for k in range(3)])
    ws = pose["stations"]
    area_half = sum(0.5 * (a["chord"] + b["chord"]) * (b["s"] - a["s"]) for a, b in zip(ws, ws[1:]))
    json.dump(dict(bounds, parts=report, wing_area_half_m2=area_half, mac_root_chord=ws[0]["chord"]),
              open(os.path.join(out, "bounds.json"), "w"), indent=1)
    print(json.dumps(dict(parts=report, bounds=bounds, wing_area_half_m2=area_half)))


if __name__ == "__main__":
    main()
