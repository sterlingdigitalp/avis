#!/usr/bin/env python3
"""eta_s at the motion the design flies: one wing section with its real heave and pitch, from section_motion.mjs.

The airfoil (chord on x, LE at the origin) rotates about its spar axis and translates as a solid body in a fixed
freestream so that, at every instant, the relative wind at the spar axis in the section frame and the in-plane pitch
rate equal the physics kinematics. Construction: psi(t) = -(theta(t) - mean theta) (nose up is clockwise here),
relative wind in the inertial frame R(psi) q(t), freestream U = cycle mean of that, mesh velocity U - R(psi) q(t),
displacement its integral (periodic). Lengths in chords, speeds in |U|, so time is in c/|U|.

    suction_kin.py run <motion.json> <tag> [--level 0.667] [--cycles 3]
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bird3d
import case as casegen
import pipeline
import suction

ROOT = pipeline.ROOT
GROUP = "suction"


def motion_tables(m):
    """Nondimensional periodic tables (t, translation, rotation) and the freestream for one section's motion."""
    s = m["samples"]
    n = len(s)
    T = m["period_s"]
    th_mean = sum(p["theta"] for p in s) / n
    psi = [-(p["theta"] - th_mean) for p in s]
    rel = [(p["u"] * math.cos(a) - p["w"] * math.sin(a), p["u"] * math.sin(a) + p["w"] * math.cos(a))
           for p, a in zip(s, psi)]
    U = (sum(r[0] for r in rel) / n, sum(r[1] for r in rel) / n)
    Umag = math.hypot(*U)
    c = m["chord_m"]
    dt = T / n
    vel = [((U[0] - r[0]) / Umag, (U[1] - r[1]) / Umag) for r in rel]
    disp = [(0.0, 0.0)]
    for k in range(1, n + 1):
        a, b = vel[k - 1], vel[k % n]
        disp.append((disp[-1][0] + 0.5 * (a[0] + b[0]) * dt * Umag / c, disp[-1][1] + 0.5 * (a[1] + b[1]) * dt * Umag / c))
    drift = disp[-1]
    disp = [(d[0] - drift[0] * k / n, d[1] - drift[1] * k / n) for k, d in enumerate(disp)]  # close the loop exactly
    tstar = [k * dt * Umag / c for k in range(n + 1)]
    psi = psi + [psi[0]]
    speed = [math.hypot(p["u"], p["w"]) / Umag for p in s]
    return dict(t=tstar, disp=disp, psi=psi, U=U, Umag=Umag, period=T * Umag / c, closure_drift=drift,
                mean_speed=sum(speed) / n, alpha_inf=math.atan2(U[1], U[0]))


def make(d, mesh, m, tables, *, level, ranks, cycles):
    re_cfd = m["re_chord"] / tables["mean_speed"]  # nu* = mean|q*| / Re_local, with Re = 1 / nu in case.py
    y1 = json.load(open(os.path.join(mesh, "mesh.json")))["y1"]
    dt = 0.0025 * y1 / 7.0e-5  # wall Courant number as on the level-1 tests
    end = cycles * tables["period"]
    spec = casegen.make_case(d, mesh, re=re_cfd, alpha_deg=math.degrees(tables["alpha_inf"]), tu_pct=0.10,
                             mode="transient", end_time=end, dt=dt, ncpu=ranks)
    xp = m["pitch_axis_x_over_c"]
    tr = " ".join(f"({t:.6f} ({dx:.8f} {dy:.8f} 0))" for t, (dx, dy) in zip(tables["t"], tables["disp"]))
    # sixDoFMotion reads rotation angles in degrees (checked: a short run moved the LE point as degrees predict).
    ro = " ".join(f"({t:.6f} (0 0 {math.degrees(p):.8f}))" for t, p in zip(tables["t"], tables["psi"]))
    with open(os.path.join(d, "constant", "dynamicMeshDict"), "w") as f:
        f.write(casegen.header("dictionary", "dynamicMeshDict", "constant") + f"""\
mover
{{
    type            motionSolver;
    libs            ("libfvMotionSolvers.so");
    motionSolver    solidBody;
    cellZone        all;
    solidBodyMotionFunction sixDoFMotion;
    CofG            ({xp:.8f} 0 0);
    translation {{ type table; outOfBounds repeat; values ({tr}); }}
    rotation {{ type table; outOfBounds repeat; values ({ro}); }}
}}
""")
    with open(os.path.join(d, "system", "functions"), "a") as f:
        f.write("""
forces1
{
    type            forces;
    libs            ("libforces.so");
    writeControl    timeStep;
    writeInterval   1;
    log             no;
    patches         (airfoil);
    rho             rhoInf;
    rhoInf          1;
    CofR            (0.25 0 0);
}
""")
    with open(os.path.join(d, "run.sh"), "w") as f:
        f.write(pipeline.STEADY_SCRIPT.format(n=ranks).replace(
            "reconstructPar -latestTime > log.reconstructPar 2>&1 || finish 14\n", ""))
    spec.update(kind="design-kinematics", motion=dict({k: v for k, v in m.items() if k != "samples"}),
                period=tables["period"], cycles=cycles, dt=dt, U_inf=tables["U"], U_ref_m_s=tables["Umag"],
                alpha_inf_deg=math.degrees(tables["alpha_inf"]), closure_drift=tables["closure_drift"],
                re_local=m["re_chord"], level=level)
    with open(os.path.join(d, "spec.json"), "w") as f:
        json.dump(spec, f, indent=1)
    with open(os.path.join(d, "motion.json"), "w") as f:
        json.dump(m, f)
    return spec


def section_forces(raw, spec, m, span):
    """Per output step: chordwise and lift coefficients in the section frame, with the kinematic effective angle."""
    s = m["samples"]
    n = len(s)
    th_mean = sum(p["theta"] for p in s) / n
    out = []
    for t, fx, fy, *_ in raw:
        ph = (t / spec["period"]) % 1.0 * n
        k = int(ph) % n
        f = ph - int(ph)
        nxt = s[(k + 1) % n]
        lerp = lambda key: s[k][key] + f * (nxt[key] - s[k][key])
        psi = -(lerp("theta") - th_mean)
        a = math.atan2(lerp("w"), lerp("u"))
        cx, cy = fx / (0.5 * span), fy / (0.5 * span)
        # into the section frame: rotate by -psi
        ax = cx * math.cos(psi) + cy * math.sin(psi)
        ay = -cx * math.sin(psi) + cy * math.cos(psi)
        cl = ay * math.cos(a) - ax * math.sin(a)
        out.append((t, -ax, cl, a))
    return out


def evaluate(d, spec, m, cd_f):
    """eta_s (strip.js form) over the whole cycles after the first: mean(Cc + Cd_f cos a) / mean(CL sin a)."""
    raw = bird3d.read_forces(d)
    rows = section_forces(raw, spec, m, 0.1)
    if not rows:
        return dict(complete=False, cycles_simulated=0)
    period = spec["period"]
    n_avg = suction.cycles_after_startup(rows, period)
    res = dict(cycles_simulated=round(rows[-1][0] / period, 3), cycles_averaged=n_avg)
    integ = lambda xs: sum(0.5 * (p[1] + q[1]) * (q[0] - p[0]) for p, q in zip(xs, xs[1:]))
    if n_avg < 1:
        # partial: whatever follows the first cycle, flagged
        start, stop = period, rows[-1][0]
        partial = True
    else:
        # whole cycles aligned to the motion: from the end of the first to the end of the last complete one
        start, stop = period, (1 + n_avg) * period
        partial = False
    sel = [r for r in rows if start - 1e-9 <= r[0] <= stop + 1e-9]
    if len(sel) < 2:
        return dict(res, complete=False)
    per_cycle = []
    for c in range(1, n_avg + 1):
        cyc = [r for r in rows if c * period - 1e-9 <= r[0] <= (c + 1) * period + 1e-9]
        den_c = integ([(r[0], r[2] * math.sin(r[3])) for r in cyc])
        per_cycle.append(integ([(r[0], r[1] + cd_f * math.cos(r[3])) for r in cyc]) / den_c if den_c else None)
    res["eta_s_per_cycle"] = per_cycle
    span_t = sel[-1][0] - sel[0][0]
    num = integ([(r[0], r[1] + cd_f * math.cos(r[3])) for r in sel])
    den = integ([(r[0], r[2] * math.sin(r[3])) for r in sel])
    cc = integ([(r[0], r[1]) for r in sel]) / span_t
    cl = integ([(r[0], r[2]) for r in sel]) / span_t
    return dict(res, complete=not partial, partial=partial, averaged_from_t=sel[0][0], averaged_to_t=sel[-1][0],
                eta_s_strip=num / den if den else None, cc_mean=cc, cl_mean=cl, mean_cl_sin_a=den / span_t, cd_f=cd_f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("run", "evaluate"))
    ap.add_argument("motion")
    ap.add_argument("tag")
    ap.add_argument("--level", type=float, default=0.667)
    ap.add_argument("--cycles", type=float, default=3)
    a = ap.parse_args()
    m = json.load(open(a.motion))
    d = os.path.join(ROOT, "out/cfd/runs", GROUP, a.tag)
    if a.what == "run":
        mesh = pipeline.build_mesh(os.path.join(ROOT, "data/references/airfoils/e387.dat"), f"e387-L{a.level:g}", a.level)
        tables = motion_tables(m)
        spec = make(d, mesh, m, tables, level=a.level, ranks=pipeline.RANKS, cycles=a.cycles)
        state = dict(diverged=False)

        def poll():
            rows = pipeline.read_forces(d)
            if pipeline.bad(rows):
                state["diverged"] = True
                return "kill"
            return None
        rc, _ = pipeline.container(f"kin-{a.tag}"[:50], d, "run.sh", 8 * 3600, pipeline.CPUS, pipeline.MEMORY, poll)
        with open(os.path.join(d, "result.json"), "w") as f:
            json.dump(dict(spec, rc=rc, diverged=state["diverged"]), f, indent=1)
        pipeline.clean(d)
    spec = json.load(open(os.path.join(d, "spec.json")))
    cd_f, cd_from = suction.friction_drag_at(m["re_chord"])
    print(json.dumps(dict(evaluate(d, spec, m, cd_f), cd_f_from=cd_from)))


if __name__ == "__main__":
    main()
