#!/usr/bin/env python3
"""Polar tables from the validated V7 setup, resumable, at most two cases at a time.

    polars.py run <coords> [--name NAME] [--level L]    run every Re, alpha point
    polars.py write <name> [--level L]                  write data/calibration/polars/<name>.json

Every point appears in the output, including failures. Nothing is dropped, smoothed or filled.
"""
import argparse
import datetime
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pipeline

RES = [30000, 50000, 80000, 100000, 120000, 200000]  # 100k added: the V7 runs supply it
ALPHAS = list(range(-6, 17, 2))
TU = 0.10


def point_name(name, level, re_, a):
    return f"{name}-L{level:g}-Re{re_}-a{a:g}-Tu{TU:g}"


def run(coords, name, level):
    mesh = pipeline.build_mesh(coords, f"{name}-L{level:g}", level)
    pool = pipeline.Pool(2)
    # Highest Re first: the tunnel-validated range, and the most useful if a run is cut short.
    for re_ in reversed(RES):
        # No transient fallback here: on the shared host it costs hours per point. A point that does not settle keeps
        # its steady values, flagged unconverged-steady with the oscillation range, and is never dropped.
        pool.submit_sweep(f"polar-{name}", [(point_name(name, level, re_, a), re_, a) for a in ALPHAS], mesh, tu=TU,
                          allow_transient=False)
    pool.wait()


def solve(a, b):
    """Gaussian elimination with pivoting."""
    n = len(b)
    m = [list(r) + [v] for r, v in zip(a, b)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    return [m[k][n] / m[k][k] for k in range(n)]


def polyfit(x, y, degree):
    """Least-squares polynomial, lowest power first."""
    k = degree + 1
    a = [[sum(v ** (i + j) for v in x) for j in range(k)] for i in range(k)]
    b = [sum(w * v ** i for v, w in zip(x, y)) for i in range(k)]
    return solve(a, b)


def strip_params(points, camber):
    """The parameter block src/physics/polars.js uses, fitted to converged points only.

    Parameters that cannot be fitted are null, never a default. Attached range for the fit: lift within the linear
    run of alpha -2 to 6 deg. The raw tables stay the data; this block is a convenience for the fast model.
    """
    from math import log10, pi, radians
    good = [p for p in points if p["converged"] and p["cl"] is not None]
    per_re = {}
    for p in good:
        per_re.setdefault(p["re"], []).append(p)
    slopes, zeros, clmax, clmin, drag, cm = [], [], [], [], [], []
    for re_, pts in sorted(per_re.items()):
        att = [p for p in pts if -2 <= p["alpha_deg"] <= 6]
        if len(att) >= 3:
            c0, c1 = polyfit([radians(p["alpha_deg"]) for p in att], [p["cl"] for p in att], 1)
            slopes.append(c1)
            zeros.append(-c0 / c1)
            q = polyfit([p["cl"] for p in att], [p["cd"] for p in att], 2)
            if q[2] > 0:
                cl_min_drag = -q[1] / (2 * q[2])
                drag.append((re_, q[0] - q[1] ** 2 / (4 * q[2]), cl_min_drag, q[2]))
            cm.extend(p["cm"] for p in att)
        # Only where the data reach them: a peak before the last converged alpha, and negative stall below -4 deg.
        top = max(pts, key=lambda p: p["cl"])
        if top["alpha_deg"] < max(p["alpha_deg"] for p in pts):
            clmax.append((re_, top["cl"]))
        if min(p["alpha_deg"] for p in pts) <= -4:
            clmin.append((re_, min(p["cl"] for p in pts)))
    if not slopes:
        return None
    out = dict(a=sum(slopes) / len(slopes), alphaL0=sum(zeros) / len(zeros), cm0=sum(cm) / len(cm), camber=camber,
               reRef=1e5, clmax=None, clmaxPerDecade=None, clmin=None, cd0=None, reExp=None, clDrag=None, kDrag=None)
    if len(clmax) >= 2:
        b0, b1 = polyfit([log10(r / 1e5) for r, _ in clmax], [v for _, v in clmax], 1)
        out.update(clmax=b0, clmaxPerDecade=b1)
    elif len(clmax) == 1:
        out.update(clmax=clmax[0][1])
    if clmin:
        out["clmin"] = sum(v for _, v in clmin) / len(clmin)
    if len(drag) >= 3:  # two Re would be an exact fit, not a fit
        d0, d1 = polyfit([log10(r / 1e5) for r, *_ in drag], [log10(v[1]) for v in drag], 1)
        out.update(cd0=10 ** d0, reExp=-d1, clDrag=sum(v[2] for v in drag) / len(drag), kDrag=sum(v[3] for v in drag) / len(drag))
    out["fitted_from_points"] = len(good)
    out["status"] = "fitted to converged 2D RANS points in this file; Re below 100k is not tunnel-checked"
    return out


def write(name, level):
    base = os.path.join(pipeline.ROOT, "out/cfd/runs", f"polar-{name}")
    points = []
    for re_ in RES:
        for a in ALPHAS:
            d = os.path.join(base, point_name(name, level, re_, a))
            r = None
            # The V7 runs are the same setup (E387, level 1, Tu 0.1%), so they serve as e387 polar points.
            v7 = os.path.join(pipeline.ROOT, "out/cfd/runs/v7", f"e387-L{level:g}-Re{re_}-a{a:g}-Tu{TU:g}")
            for p in (os.path.join(d, "result.json"), os.path.join(d, "failure.json"),
                      os.path.join(v7, "result.json") if name == "e387" else ""):
                if p and os.path.exists(p):
                    r = json.load(open(p))
                    break
            r = r or dict(status="not-run", converged=False, cl=None, cd=None, cm=None)
            points.append(dict(
                re=re_, alpha_deg=a, cl=r.get("cl"), cd=r.get("cd"), cm=r.get("cm"), status=r.get("status"),
                converged=bool(r.get("converged")),
                unsteady_cl_range=([r["transient"]["cl_min"], r["transient"]["cl_max"]] if r.get("transient") else
                                   [r["steady"]["cl_min"], r["steady"]["cl_max"]] if r.get("steady") else None),
                upper_separated=r.get("upper_separated"), yplus_max=r.get("yplus_max"),
                validated_re=100000 <= re_ <= 200000))

    def table(key):
        return [[next(p[key] for p in points if p["re"] == re_ and p["alpha_deg"] == a) for a in ALPHAS] for re_ in RES]
    mesh_meta = json.load(open(os.path.join(pipeline.ROOT, "out/cfd/meshes", f"{name}-L{level:g}", "mesh.json")))
    out = dict(
        airfoil=name, re=RES, alpha_deg=ALPHAS, cl=table("cl"), cd=table("cd"), cm=table("cm"),
        converged=table("converged"), status=table("status"), points=points,
        strip_params=strip_params(points, mesh_meta["camber"]),
        coords=os.path.relpath(mesh_meta["coords"], pipeline.ROOT),
        thickness=mesh_meta["thickness"], camber=mesh_meta["camber"], te_gap_closed=mesh_meta["te_gap_closed"],
        method=dict(solver="OpenFOAM 13 foamRun incompressibleFluid", turbulence="kOmegaSSTLM", tu_percent=TU,
                    grid_level=level, cells=mesh_meta["cells"], y1_over_c=mesh_meta["y1"], cm_about="c/4, nose up positive",
                    notes="2D RANS. V7 validated range is Re 100k to 200k; Re below 100k is an extrapolation of the "
                          "transition correlation and has no tunnel check in this project."),
        generated=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        command="python3 cfd/polars.py write " + name)
    gate = os.path.join(pipeline.ROOT, "out/verification/V7.json")
    v7 = json.load(open(gate)) if os.path.exists(gate) else None
    out["validation"] = dict(
        gate="V7", passed=v7["passed"] if v7 else None,
        summary=({k: v7["measured"].get(k) for k in ("points", "points_passed", "points_unconverged", "cl_max_err",
                                                      "cd_max_err", "cl_mean_err", "cd_mean_err")} if v7 else None),
        note="Same solver, model, turbulence and grid settings as V7. V7 failed: treat these tables as unvalidated, "
             "and Cd especially so near the laminar-bubble drag rise and towards stall." if v7 and not v7["passed"]
        else None)
    path = os.path.join(pipeline.ROOT, "data/calibration/polars", f"{name}.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    bad = [p for p in points if not p["converged"]]
    print(f"{path}: {len(points)} points, {len(bad)} not converged")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("run", "write"))
    ap.add_argument("target")
    ap.add_argument("--name")
    ap.add_argument("--level", type=float, default=1.0)
    a = ap.parse_args()
    if a.what == "run":
        run(a.target, a.name or os.path.splitext(os.path.basename(a.target))[0], a.level)
    else:
        write(a.target, a.level)


if __name__ == "__main__":
    main()
