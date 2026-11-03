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


def make(d, mesh, m, tables, *, level, ranks, cycles, write_fraction=1.0):
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
    checkpoint_every_cycle(d, tables["period"], write_fraction, end)
    spec.update(kind="design-kinematics", motion=dict({k: v for k, v in m.items() if k != "samples"}),
                period=tables["period"], cycles=cycles, dt=dt, U_inf=tables["U"], U_ref_m_s=tables["Umag"],
                alpha_inf_deg=math.degrees(tables["alpha_inf"]), closure_drift=tables["closure_drift"],
                re_local=m["re_chord"], level=level)
    with open(os.path.join(d, "spec.json"), "w") as f:
        json.dump(spec, f, indent=1)
    with open(os.path.join(d, "motion.json"), "w") as f:
        json.dump(m, f)
    return spec


RESUME_SCRIPT = pipeline.LOCAL_PROLOGUE + """
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} foamRun -parallel > log.foamRun.resume 2>&1 || finish 13
finish 0
"""


def set_control(d, **entries):
    """Set controlDict entries in place (endTime, writeInterval, purgeWrite, ...)."""
    import re as _re
    path = os.path.join(d, "system", "controlDict")
    text = open(path).read()
    for key, val in entries.items():
        text, n = _re.subn(rf"(^|\n){key}\s+[^;]*;", rf"\g<1>{key} {val};", text)
        if not n:
            text += f"{key} {val};\n"
    with open(path, "w") as f:
        f.write(text)


def checkpoint_every_cycle(d, period, fraction=1.0, end=None):
    """Write every `fraction` of a cycle and keep the two newest (the second-newest is complete for the host copy).
    The end time is rounded to whole write intervals: OpenFOAM writes no final time between intervals."""
    interval = fraction * period
    entries = dict(writeControl="runTime", writeInterval=f"{interval:.10g}", purgeWrite=2)
    if end is not None:
        entries["endTime"] = f"{max(1, round(end / interval)) * interval:.10g}"
    set_control(d, **entries)


def prepare_extend(d, cycles_more, ranks):
    """Set a finished or stopped case up to run `cycles_more` further cycles from its latest restart time."""
    spec = json.load(open(os.path.join(d, "spec.json")))
    t0 = pipeline.latest_processor_time(d)
    if t0 is None:
        raise RuntimeError(f"{d} has no restart time directory to extend from")
    frac = spec.get("write_fraction", 1.0)
    end = float(t0) + cycles_more * spec["period"]
    set_control(d, startFrom="latestTime")
    checkpoint_every_cycle(d, spec["period"], frac, end)
    end = max(1, round(end / (frac * spec["period"]))) * frac * spec["period"]
    if os.path.exists(os.path.join(d, "stop")):
        os.remove(os.path.join(d, "stop"))
    with open(os.path.join(d, "resume.sh"), "w") as f:
        f.write(RESUME_SCRIPT.format(n=ranks))
    spec.setdefault("extensions", []).append(dict(from_t=float(t0), to_t=end))
    with open(os.path.join(d, "spec.json"), "w") as f:
        json.dump(spec, f, indent=1)
    return float(t0), end


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


def evaluate_last(d, spec, m, cd_f, last=3, band=0.05):
    """eta_s over the last `last` whole cycles, aligned to the motion, with each cycle's value. Settled when every
    cycle's eta_s lies within +/- band of their mean (the coordinator's criterion, read as half-width band)."""
    rows = section_forces(bird3d.read_forces(d), spec, m, 0.1)
    period = spec["period"]
    if not rows:
        return dict(complete=False, cycles_simulated=0)
    n_whole = int(rows[-1][0] / period + 1e-3)
    integ = lambda xs: sum(0.5 * (p[1] + q[1]) * (q[0] - p[0]) for p, q in zip(xs, xs[1:]))
    per = []
    for c in range(max(1, n_whole - last), n_whole):  # never the first (start-up) cycle
        cyc = [r for r in rows if c * period - 1e-9 <= r[0] <= (c + 1) * period + 1e-9]
        num = integ([(r[0], r[1] + cd_f * math.cos(r[3])) for r in cyc])
        den = integ([(r[0], r[2] * math.sin(r[3])) for r in cyc])
        per.append(dict(cycle=c + 1, eta_s=num / den if den else None, cc=integ([(r[0], r[1]) for r in cyc]) / period,
                        cl=integ([(r[0], r[2]) for r in cyc]) / period, mean_cl_sin_a=den / period))
    etas = [p["eta_s"] for p in per if p["eta_s"] is not None]
    mean = sum(etas) / len(etas) if etas else None
    settled = bool(len(etas) == last and max(abs(e - mean) for e in etas) <= band)
    sel = [r for r in rows if per and per[0]["cycle"] - 1 <= r[0] / period <= per[-1]["cycle"] + 1e-9]
    num = integ([(r[0], r[1] + cd_f * math.cos(r[3])) for r in sel]) if sel else 0
    den = integ([(r[0], r[2] * math.sin(r[3])) for r in sel]) if sel else 0
    return dict(cycles_simulated=round(rows[-1][0] / period, 3), cycles_whole=n_whole, cycles_averaged=len(per),
                averaged_cycles=[p["cycle"] for p in per], per_cycle=per, eta_s_strip=num / den if den else None,
                eta_s_mean_of_cycles=mean, settled=settled, settle_band=band, cd_f=cd_f,
                complete=bool(n_whole >= 6 and len(per) == last))


def motion_check(d, m, tables, mesh):
    """Where the leading-edge point is at the last written time, against where the tables put it."""
    import bisect
    import re as _re
    meta = json.load(open(os.path.join(mesh, "mesh.json")))
    gid = 2 * (meta["n_wake"] + meta["n_surface"])
    le0 = meta["wall"][meta["n_surface"]]
    procs = [p for p in os.listdir(d) if p.startswith("processor")]
    if not procs:
        return None
    times = [t for t in os.listdir(os.path.join(d, procs[0])) if t.replace(".", "").replace("e-", "").isdigit()]
    times = [t for t in times if float(t) > 0]
    if not times:
        return None
    tt = max(times, key=float)
    T = tables["t"][-1]
    tau = float(tt) % T
    k = max(0, min(len(tables["t"]) - 2, bisect.bisect_right(tables["t"], tau) - 1))
    f = (tau - tables["t"][k]) / (tables["t"][k + 1] - tables["t"][k])
    psi = tables["psi"][k] + f * (tables["psi"][k + 1] - tables["psi"][k])
    dx = tables["disp"][k][0] + f * (tables["disp"][k + 1][0] - tables["disp"][k][0])
    dy = tables["disp"][k][1] + f * (tables["disp"][k + 1][1] - tables["disp"][k][1])
    xp = m["pitch_axis_x_over_c"]
    ex = xp + math.cos(psi) * (le0[0] - xp) - math.sin(psi) * le0[1] + dx
    ey = math.sin(psi) * (le0[0] - xp) + math.cos(psi) * le0[1] + dy
    for p in procs:
        addr = open(os.path.join(d, p, "constant/polyMesh/pointProcAddressing")).read()
        nums = [int(v) for v in addr[addr.index("(") + 1:addr.rindex(")")].split()]
        if gid in nums:
            txt = open(os.path.join(d, p, tt, "polyMesh/points")).read()
            pts = _re.findall(r"\(([^()]+)\)", txt[txt.index("(\n") + 2:])
            ax, ay = (float(v) for v in pts[nums.index(gid)].split()[:2])
            return dict(time=float(tt), cycles=float(tt) / T, expected=[ex, ey], actual=[ax, ay],
                        error_chords=math.hypot(ax - ex, ay - ey))
    return None


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


def run_container(d, spec, m, a, script):
    """Run until the motion's end time, or stop early (writeNow) once the last three whole cycles settle."""
    cd_f, _ = suction.friction_drag_at(m["re_chord"])
    state = dict(diverged=False, settled=False, seen=0)

    def poll():
        rows = pipeline.read_forces(d)
        if pipeline.bad(rows):
            state["diverged"] = True
            return "kill"
        whole = int(rows[-1][0] / spec["period"] + 1e-3) if rows else 0
        if whole >= max(6, a.min_cycles) and whole > state["seen"]:
            state["seen"] = whole
            ev = evaluate_last(d, spec, m, cd_f)
            if ev.get("settled") and ev.get("cycles_whole", 0) >= 6:
                state["settled"] = True
                return "stop"
        return None
    rc, _ = pipeline.container(f"kin-{a.tag}"[:50], d, script, 14 * 3600, a.cpus, pipeline.MEMORY, poll)
    return rc, state


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("run", "evaluate", "extend"))
    ap.add_argument("motion")
    ap.add_argument("tag")
    ap.add_argument("--level", type=float, default=0.667)
    ap.add_argument("--cycles", type=float, default=3)
    ap.add_argument("--cpus", type=int, default=pipeline.CPUS)
    ap.add_argument("--ranks", type=int, default=pipeline.RANKS)
    ap.add_argument("--min-cycles", type=int, default=6, help="no settled stop before this many whole cycles")
    ap.add_argument("--write-fraction", type=float, default=1.0, help="checkpoint every this fraction of a cycle")
    a = ap.parse_args()
    m = json.load(open(a.motion))
    d = os.path.join(ROOT, "out/cfd/runs", GROUP, a.tag)
    if a.what == "run":
        mesh = pipeline.build_mesh(os.path.join(ROOT, "data/references/airfoils/e387.dat"), f"e387-L{a.level:g}", a.level)
        tables = motion_tables(m)
        spec = make(d, mesh, m, tables, level=a.level, ranks=a.ranks, cycles=a.cycles, write_fraction=a.write_fraction)
        spec["write_fraction"] = a.write_fraction
        with open(os.path.join(d, "spec.json"), "w") as f:
            json.dump(spec, f, indent=1)
        rc, state = run_container(d, spec, m, a, "run.sh")
        try:
            check = motion_check(d, m, tables, mesh)
        except Exception as e:  # recorded, never fatal
            check = dict(error=repr(e))
        with open(os.path.join(d, "result.json"), "w") as f:
            json.dump(dict(spec, rc=rc, diverged=state["diverged"], settled_stop=state["settled"], motion_check=check), f, indent=1)
        pipeline.clean_keep_restart(d)
    elif a.what == "extend":
        mesh = os.path.join(ROOT, "out/cfd/meshes", f"e387-L{a.level:g}")
        tables = motion_tables(m)
        spec = json.load(open(os.path.join(d, "spec.json")))
        prepare_extend(d, a.cycles, a.ranks)
        rc, state = run_container(d, spec, m, a, "resume.sh")
        try:
            check = motion_check(d, m, tables, mesh)
        except Exception as e:
            check = dict(error=repr(e))
        res = json.load(open(os.path.join(d, "result.json"))) if os.path.exists(os.path.join(d, "result.json")) else dict(spec)
        res.update(rc_extend=rc, diverged=state["diverged"], settled_stop=state["settled"], motion_check=check)
        with open(os.path.join(d, "result.json"), "w") as f:
            json.dump(res, f, indent=1)
        pipeline.clean_keep_restart(d)
    spec = json.load(open(os.path.join(d, "spec.json")))
    cd_f, cd_from = suction.friction_drag_at(m["re_chord"])
    print(json.dumps(dict(evaluate_last(d, spec, m, cd_f), cd_f_from=cd_from)))


if __name__ == "__main__":
    main()
