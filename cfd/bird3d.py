#!/usr/bin/env python3
"""3D finalist check: whole bird frozen at its cruise glide pose, half model, prism layers, kOmegaSSTLM.

    bird3d.py <design> [--cpus 8] [--memory 16g] [--surface-cell 0.0025] [--first-layer 5e-5] [--tag main]

Writes out/cfd/<design>/<tag>/ (case, logs) and out/cfd/<design>/<tag>/result.json, from which the report is
built. Lift and drag are whole-bird values (half model x 2), at the design air density and cruise speed.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import case3d
import pipeline

RESUME = pipeline.LOCAL_PROLOGUE + """
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} foamRun -parallel > log.foamRun.resume 2>&1 || finish 15
reconstructPar -latestTime -fields '(yPlus)' > log.reconstructPar 2>&1
finish 0
"""

SCRIPT = pipeline.LOCAL_PROLOGUE + """
blockMesh > log.blockMesh 2>&1 || finish 11
decomposePar -force > log.decomposePar 2>&1 || finish 12
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} snappyHexMesh -parallel -overwrite > log.snappyHexMesh 2>&1 || finish 13
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} checkMesh -parallel > log.checkMesh 2>&1
for d in processor*; do rm -rf $d/0; cp -r 0.orig $d/0; done
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} potentialFoam -parallel -initialiseUBCs -pName p > log.potentialFoam 2>&1
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} foamRun -parallel > log.foamRun 2>&1 || finish 15
reconstructPar -latestTime -fields '(yPlus)' > log.reconstructPar 2>&1
finish 0
"""


def read_forces(d):
    """Total (pressure + viscous) half-model force rows: (iter, fx, fy, fz)."""
    base = os.path.join(d, "postProcessing", "forces1")
    rows = []
    if not os.path.isdir(base):
        return rows
    for start in sorted(os.listdir(base), key=float):
        path = os.path.join(base, start, "forces.dat")
        if not os.path.exists(path):
            continue
        for line in open(path):
            if line.startswith("#"):
                continue
            # nan and inf must parse too, or a diverging run would never be caught
            nums = [float(v) for v in re.findall(r"[-+]?(?:nan|inf|\d*\.?\d+(?:[eE][-+]?\d+)?)",
                                                 line.replace("(", " ").replace(")", " "), flags=re.IGNORECASE)]
            if len(nums) >= 7:
                t, fp, fv = nums[0], nums[1:4], nums[4:7]
                rows.append((t, fp[0] + fv[0], fp[1] + fv[1], fp[2] + fv[2], fp, fv))
    return rows


def layer_coverage(d):
    """snappyHexMesh's final layer table: per patch, faces, mean layers, overall thickness [m] and [% of target]."""
    path = os.path.join(d, "log.snappyHexMesh")
    if not os.path.exists(path):
        return None
    lines = open(path).read().splitlines()
    heads = [k for k, line in enumerate(lines) if "overall thickness" in line]
    if not heads:
        return None
    out = {}
    for line in lines[heads[-1] + 1:heads[-1] + 12]:
        p = line.split()
        if len(p) >= 5 and p[0] in ("body", "wing", "tail"):
            try:
                out[p[0]] = dict(faces=int(p[1]), layers_mean=float(p[2]), thickness_m=float(p[3]),
                                 thickness_pct=float(p[4]))
            except ValueError:
                pass
    return out or None


def mesh_stats(d):
    path = os.path.join(d, "log.checkMesh")
    if not os.path.exists(path):
        return None
    text = open(path).read()
    g = lambda pat: (re.search(pat, text) or [None, None])[1]
    return dict(cells=g(r"cells:\s+(\d+)"), non_ortho_max=g(r"non-orthogonality Max: ([\d.e+-]+)"),
                skewness_max=g(r"Max skewness = ([\d.e+-]+)"), mesh_ok="Mesh OK" in text)


def yplus(d):
    base = os.path.join(d, "postProcessing", "yPlus1")
    if not os.path.isdir(base):
        return None
    out = {}
    for start in sorted(os.listdir(base), key=float):
        path = os.path.join(base, start, "yPlus.dat")
        for line in open(path):
            p = line.split()
            if line.startswith("#") or len(p) < 5:
                continue
            out[p[1]] = dict(min=float(p[2]), max=float(p[3]), mean=float(p[4]))
    return out or None


def summarise(rows, aoa, rho, V, S):
    """Whole-bird lift and drag from half-model force rows, averaged over the last 500 iterations (or last half)."""
    tail = [r for r in rows if r[0] > max(rows[-1][0] - 500, 0.5 * rows[-1][0])] or rows[-1:]
    lifts = [case3d.lift_drag(r[1], r[2], aoa) for r in tail]
    L = sum(x[0] for x in lifts) / len(lifts)
    D = sum(x[1] for x in lifts) / len(lifts)
    q = 0.5 * rho * V * V
    a = math.radians(aoa)
    # Viscous part of the drag: the viscous force resolved on the drag direction, doubled for the whole bird.
    visc = sum(-r[5][0] * math.cos(a) + r[5][1] * math.sin(a) for r in tail) / len(tail) * 2
    return dict(lift_n=L, drag_n=D, lift_range_n=[min(x[0] for x in lifts), max(x[0] for x in lifts)],
                drag_range_n=[min(x[1] for x in lifts), max(x[1] for x in lifts)], CL=L / (q * S), CD=D / (q * S),
                L_over_D=L / D if D else None, wing_area_m2=S, viscous_drag_share=visc / D if D else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("design")
    ap.add_argument("--cpus", type=int, default=8)
    ap.add_argument("--ranks", type=int, default=6)
    ap.add_argument("--memory", default="16g")
    ap.add_argument("--surface-cell", type=float, default=0.0025)
    ap.add_argument("--first-layer", type=float, default=5e-5)
    ap.add_argument("--iterations", type=int, default=3000)
    ap.add_argument("--tag", default="main")
    ap.add_argument("--max-seconds", type=int, default=6 * 3600)
    ap.add_argument("--resume", action="store_true", help="continue a stopped run from its last written time")
    a = ap.parse_args()
    design = json.load(open(os.path.join(ROOT, "data/designs", a.design + ".json")))
    top = os.path.join(ROOT, "out/cfd", a.design)
    stl = os.path.join(top, "stl")
    os.makedirs(stl, exist_ok=True)
    pose = os.path.join(top, "pose.json")
    with open(pose, "w") as f:
        subprocess.run(["node", os.path.join(HERE, "pose.mjs"), os.path.join(ROOT, "data/designs", a.design + ".json")],
                       stdout=f, check=True)
    p = json.load(open(pose))
    sections = {"e387-film": os.path.join(ROOT, "data/references/airfoils/e387.dat"),
                "thin-plate": os.path.join(ROOT, "out/cfd/ref/sections/thin-plate.dat")}
    subprocess.run([sys.executable, os.path.join(HERE, "stl_bird.py"), pose, sections[p["airfoil"]],
                    sections[p["tail"]["polar"]], stl], check=True, stdout=subprocess.DEVNULL)
    bounds = json.load(open(os.path.join(stl, "bounds.json")))
    d = os.path.join(top, a.tag)
    if a.resume:
        # Continue a stopped run from its last written time: same mesh, fields and settings, more iterations.
        spec = json.load(open(os.path.join(d, "spec.json")))
        last = max((float(t) for t in os.listdir(os.path.join(d, "processor0")) if t.replace(".", "").isdigit()), default=0)
        with open(os.path.join(d, "system/controlDict")) as f:
            text = f.read()
        text = re.sub(r"endTime \d+;", f"endTime {a.iterations};", text)
        text = re.sub(r"writeInterval \d+;", f"writeInterval {a.iterations};", text)
        with open(os.path.join(d, "system/controlDict"), "w") as f:
            f.write(text)
        with open(os.path.join(d, "run.sh"), "w") as f:
            f.write(RESUME.format(n=a.ranks))
        spec["resumed_from"] = last
    else:
        spec = make_new(d, a, design, p, stl, bounds)
    poll, state = poller(d, math.degrees(p["aoa_rad"]))
    t0 = time.time()
    rc, _ = pipeline.container(f"3d-{a.design}-{a.tag}", d, "run.sh", a.max_seconds, a.cpus, a.memory, poll)
    finish(d, a, design, p, spec, bounds, rc, state, poll, t0)


def make_new(d, a, design, p, stl, bounds):
    atm = design["atmosphere"]
    rho, mu = atm["rho_kg_m3"], atm["mu_Pa_s"]
    V, aoa = p["V"], math.degrees(p["aoa_rad"])
    spec = case3d.make_case(d, stl, speed=V, aoa_deg=aoa, rho=rho, nu=mu / rho, lref=design["geometry"]["mac_m"],
                            area_half=bounds["wing_area_half_m2"], first_layer=a.first_layer, cpus=a.ranks,
                            surface_cell=a.surface_cell, end_time=a.iterations)
    with open(os.path.join(d, "run.sh"), "w") as f:
        f.write(SCRIPT.format(n=a.ranks))
    return spec


def poller(d, aoa):
    state = dict(diverged=False)

    def poll():
        rows = read_forces(d)
        if rows and any(not math.isfinite(v) or abs(v) > 1e4 for v in rows[-1][1:4]):
            state["diverged"] = True
            return "kill"
        if rows and rows[-1][0] >= 1500:  # consecutive 500-iteration windows, from 1500 iterations on
            tail = [r for r in rows if r[0] > rows[-1][0] - 500]
            prev = [r for r in rows if rows[-1][0] - 1000 < r[0] <= rows[-1][0] - 500]
            if prev:
                lift = lambda rs: sum(case3d.lift_drag(r[1], r[2], aoa)[0] for r in rs) / len(rs)
                drag = lambda rs: sum(case3d.lift_drag(r[1], r[2], aoa)[1] for r in rs) / len(rs)
                if abs(lift(tail) - lift(prev)) < 0.005 * abs(lift(tail)) and abs(drag(tail) - drag(prev)) < 0.01 * abs(drag(tail)):
                    return "stop"
        return None
    return poll, state


def finish(d, a, design, p, spec, bounds, rc, state, poll, t0):
    atm = design["atmosphere"]
    rho, mu = atm["rho_kg_m3"], atm["mu_Pa_s"]
    V, aoa = p["V"], math.degrees(p["aoa_rad"])
    rows = read_forces(d)
    converged = poll() == "stop" if rows and not state["diverged"] else False
    res = dict(design=a.design, converged=converged, tag=a.tag, rc=rc, diverged=state["diverged"], wall_seconds=round(time.time() - t0),
               spec=spec, pose=dict(V=V, aoa_deg=aoa, phi_deg=math.degrees(p["phi"]), glide=p["glide"]),
               mesh=mesh_stats(d), layers=layer_coverage(d), yplus=yplus(d), iterations=rows[-1][0] if rows else 0,
               atmosphere=dict(rho=rho, mu=mu), weight_n=design["mass"]["total_kg"] * 9.80665)
    if rows and not state["diverged"]:
        res.update(summarise(rows, aoa, rho, V, 2 * bounds["wing_area_half_m2"]))
    json.dump(res, open(os.path.join(d, "result.json"), "w"), indent=1)
    print(json.dumps({k: res.get(k) for k in ("rc", "iterations", "lift_n", "drag_n", "CL", "CD", "L_over_D", "weight_n",
                                              "wall_seconds")}))


if __name__ == "__main__":
    main()
