#!/usr/bin/env python3
"""Leading-edge suction efficiency eta_s from 2D harmonic plunge (protocol: src/physics/strip.js SUCTION.to_measure).

The whole C-grid plunges as a solid body, h(t) = h0 sin(omega t), in a fixed freestream at mean incidence alpha0, so
the forces are inertial-frame forces with no frame correction. Non-dimensional: chord 1, U 1, b = 0.5, k = omega b / U.

Definitions (stated in data/calibration/suction.json):
  C_c(t)      chordwise force coefficient, positive towards the leading edge, = -F_x / (q c span); chord on x.
  C_S,ideal   Garrick's mean leading-edge suction of a flat plate in pure plunge, pi |C(k)|^2 (k h0 / b)^2, which for a
              flat plate in plunge is its whole mean thrust (Garrick 1936).
  eta_s       (mean C_c - C_c,steady(alpha0)) / C_S,ideal, primary: the plunge-induced chordwise force over the ideal
              plunge suction; C_c,steady = Cl sin alpha0 - Cd cos alpha0 from the steady run at the same Re, alpha0 and grid,
              so friction and steady suction cancel.
  eta_s,lit   (mean C_c + Cd_f) / C_S,ideal, the protocol's literal form with Cd_f the steady Cd at alpha0.

    suction.py run <coords> <name> --k K --h H --alpha A --level L --re RE
    suction.py write
"""
import argparse
import datetime
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bird3d
import case as casegen
import pipeline

ROOT = pipeline.ROOT
GROUP = "suction"
CYCLES, AVERAGE = 3, 2  # run three plunge cycles, average the last two


def bessel(x):
    """J0, J1, Y0, Y1 for 0 < x <= 3 (Abramowitz and Stegun 9.4.1-9.4.6)."""
    t = (x / 3.0) ** 2
    j0 = (1 - 2.2499997 * t + 1.2656208 * t ** 2 - 0.3163866 * t ** 3 + 0.0444479 * t ** 4 - 0.0039444 * t ** 5
          + 0.0002100 * t ** 6)
    j1 = x * (0.5 - 0.56249985 * t + 0.21093573 * t ** 2 - 0.03954289 * t ** 3 + 0.00443319 * t ** 4
              - 0.00031761 * t ** 5 + 0.00001109 * t ** 6)
    y0 = (2 / math.pi) * math.log(x / 2) * j0 + (0.36746691 + 0.60559366 * t - 0.74350384 * t ** 2
                                                + 0.25300117 * t ** 3 - 0.04261214 * t ** 4 + 0.00427916 * t ** 5
                                                - 0.00024846 * t ** 6)
    y1 = ((2 / math.pi) * x * math.log(x / 2) * j1 + (-0.6366198 + 0.2212091 * t + 2.1682709 * t ** 2
                                                       - 1.3164827 * t ** 3 + 0.3123951 * t ** 4 - 0.0400976 * t ** 5
                                                       + 0.0027873 * t ** 6)) / x
    return j0, j1, y0, y1


def theodorsen(k):
    """Theodorsen's function C(k) = F + iG, from Hankel functions of the second kind."""
    if not 0 < k <= 3:
        raise ValueError("k outside the Bessel approximation range")
    j0, j1, y0, y1 = bessel(k)
    den = (j1 + y0) ** 2 + (y1 - j0) ** 2
    return (j1 * (j1 + y0) + y1 * (y1 - j0)) / den, -(y1 * y0 + j1 * j0) / den


def ideal_suction(k, h0):
    """Garrick: mean leading-edge suction coefficient (chord, q based) of a flat plate plunging at amplitude h0 / c."""
    F, G = theodorsen(k)
    return math.pi * (F * F + G * G) * (k * h0 / 0.5) ** 2


def cycle_mean(rows, period, cycles):
    """Mean chordwise coefficient over the last whole `cycles` periods; None if the run is shorter."""
    if not rows:
        return None
    end = rows[-1][0]
    start = end - cycles * period
    if start < rows[0][0]:
        return None
    sel = [r for r in rows if r[0] >= start - 1e-9]
    # Trapezoid in time, so uneven output spacing does not bias the mean.
    area = sum(0.5 * (a[1] + b[1]) * (b[0] - a[0]) for a, b in zip(sel, sel[1:]))
    return area / (sel[-1][0] - sel[0][0])


def cycles_after_startup(rows, period):
    """Whole cycles available after the first (start-up) one; at least 1 when two cycles exist, else 0."""
    if not rows:
        return 0
    return max(0, int(rows[-1][0] / period + 1e-3) - 1)  # the plunge starts at t = 0


def strip_eta(raw, spec, span, cd_f, average=None):
    """eta_s as src/physics/strip.js applies it: realised chordwise force = eta L sin(a) - D_f cos(a), with a the
    instantaneous effective angle and L the measured lift normal to the relative wind. Over the last AVERAGE cycles:
    eta = mean(C_c + Cd_f cos a) / mean(C_L sin a). The mesh moves at +h0 omega cos(omega t) in y, so the relative
    wind is (cos a0, sin a0 - h0 omega cos(omega t))."""
    a0, w, h0 = math.radians(spec["alpha_deg"]), spec["omega"], spec["h_over_c"]
    end = raw[-1][0]
    sel = [r for r in raw if r[0] >= end - (average or AVERAGE) * spec["period"] - 1e-9]
    num, den = [], []
    for t, fx, fy, *_ in sel:
        a = math.atan2(math.sin(a0) - h0 * w * math.cos(w * t), math.cos(a0))
        cx, cy = fx / (0.5 * span), fy / (0.5 * span)
        cl = cy * math.cos(a) - cx * math.sin(a)
        num.append((t, -cx + cd_f * math.cos(a)))
        den.append((t, cl * math.sin(a)))
    integ = lambda xs: sum(0.5 * (p[1] + q[1]) * (q[0] - p[0]) for p, q in zip(xs, xs[1:]))
    d = integ(den)
    return dict(eta=integ(num) / d if d else None, mean_cl_sin_a=d / (sel[-1][0] - sel[0][0]), cd_f=cd_f)


def point_name(name, level, re_, k, h, alpha):
    return f"{name}-L{level:g}-Re{re_}-k{k:g}-h{h:g}-a{alpha:g}"


def make(d, mesh, *, re_, k, h, alpha, ranks):
    omega = 2.0 * k  # k = omega b / U with b = 0.5, U = 1
    period = 2 * math.pi / omega
    dt = 0.0025
    end = CYCLES * period
    spec = casegen.make_case(d, mesh, re=re_, alpha_deg=alpha, tu_pct=0.10, mode="transient", end_time=end, dt=dt,
                             ncpu=ranks)
    with open(os.path.join(d, "constant", "dynamicMeshDict"), "w") as f:
        f.write(casegen.header("dictionary", "dynamicMeshDict", "constant") + f"""\
mover
{{
    type            motionSolver;
    libs            ("libfvMotionSolvers.so");
    motionSolver    solidBody;
    cellZone        all;
    solidBodyMotionFunction oscillatingLinearMotion;
    amplitude       (0 {h:.10g} 0);
    omega           {omega:.10g};
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
        f.write(pipeline.STEADY_SCRIPT.format(n=ranks).replace("reconstructPar -latestTime > log.reconstructPar 2>&1 || finish 14\n", ""))
    spec.update(k=k, h_over_c=h, omega=omega, period=period, cycles=CYCLES, dt=dt)
    with open(os.path.join(d, "spec.json"), "w") as f:
        json.dump(spec, f, indent=1)
    return spec


def run(coords, name, *, k, h, alpha, level, re_, cpus=pipeline.CPUS, ranks=pipeline.RANKS):
    mesh = pipeline.build_mesh(coords, f"{name}-L{level:g}", level)
    pname = point_name(name, level, re_, k, h, alpha)
    d = os.path.join(ROOT, "out/cfd/runs", GROUP, pname)
    if os.path.exists(os.path.join(d, "result.json")):
        return json.load(open(os.path.join(d, "result.json")))
    spec = make(d, mesh, re_=re_, k=k, h=h, alpha=alpha, ranks=ranks)
    state = dict(diverged=False)

    def poll():
        rows = pipeline.read_forces(d)
        if pipeline.bad(rows):
            state["diverged"] = True
            return "kill"
        return None

    rc, _ = pipeline.container(f"suc-{pname}"[:50], d, "run.sh", 10 * 3600, cpus, pipeline.MEMORY, poll)
    span = json.load(open(os.path.join(mesh, "mesh.json")))["span"]
    rows = [(r[0], -r[1] / (0.5 * span), r[2] / (0.5 * span)) for r in bird3d.read_forces(d)]
    mean_c = cycle_mean(rows, spec["period"], AVERAGE)
    first = cycle_mean([r for r in rows if r[0] <= rows[-1][0] - spec["period"]], spec["period"], 1) if rows else None
    last = cycle_mean(rows, spec["period"], 1)
    res = dict(spec, name=pname, rc=rc, diverged=state["diverged"], cc_mean=mean_c, cc_cycle_n_minus_1=first,
               cc_cycle_n=last, simulated=rows[-1][0] if rows else 0, ideal_suction=ideal_suction(k, h),
               theodorsen=theodorsen(k), complete=bool(mean_c is not None and not state["diverged"] and rc == 0))
    with open(os.path.join(d, "result.json"), "w") as f:
        json.dump(res, f, indent=1)
    pipeline.clean(d)
    return res


def steady_reference(re_, alpha, level):
    """Steady Cl, Cd at the same Re, alpha and grid, from the V7 runs (E387)."""
    name = f"e387-L{level:g}-Re{re_}-a{alpha:g}-Tu0.1"
    p = next((q for q in (os.path.join(ROOT, "out/cfd/runs", g, name, "result.json") for g in ("v7", "v7-grid"))
              if os.path.exists(q)), None)
    if p is None:
        return None
    r = json.load(open(p))
    a = math.radians(alpha)
    return dict(status=r["status"], converged=r["converged"], cl=r["cl"], cd=r["cd"],
                cc=r["cl"] * math.sin(a) - r["cd"] * math.cos(a), source=os.path.relpath(p, ROOT))


def friction_drag(re_):
    """Polar minimum drag at this Re, as strip.js defines D_f: lowest Cd over converged V7 points (E387, level 1)."""
    vals = []
    base = os.path.join(ROOT, "out/cfd/runs/v7")
    for name in os.listdir(base) if os.path.isdir(base) else []:
        p = os.path.join(base, name, "result.json")
        if f"-Re{re_}-" in name and os.path.exists(p):
            r = json.load(open(p))
            if r.get("converged") and r.get("cd") is not None:
                vals.append((r["cd"], name))
    return min(vals) if vals else (None, None)


def decimated_history(rows, period, per_cycle=60):
    """Chordwise force coefficient against phase, about `per_cycle` samples per cycle, for the record."""
    if not rows:
        return None
    step = max(1, int(len(rows) / (rows[-1][0] / period * per_cycle)))
    return dict(t_over_period=[round(q[0] / period, 4) for q in rows[::step]], cc=[round(q[1], 5) for q in rows[::step]])


def kinematics_point(base, name, r):
    """A design-kinematics run (suction_kin.py): eta_s in the strip.js form, with the effective-angle history."""
    import suction_kin
    d = os.path.join(base, name)
    m = json.load(open(os.path.join(d, "motion.json")))
    cd_f, cd_from = friction_drag_at(m["re_chord"])
    ev = suction_kin.evaluate(d, r, m, cd_f)
    a = [p["alpha_eff"] for p in m["samples"]]
    rows = suction_kin.section_forces(bird3d.read_forces(d), r, m, 0.1)
    hist = decimated_history([(q[0], q[1]) for q in rows], r["period"]) if rows else None
    if hist:
        sel = rows[::max(1, int(len(rows) / len(hist["cc"])))][:len(hist["cc"])]
        hist["cl"] = [round(q[2], 4) for q in sel]
        hist["alpha_eff_deg"] = [round(math.degrees(q[3]), 3) for q in sel]
    mo = r["motion"]
    return dict(name=name, kind="design kinematics (cruise, flight log)", level=r["level"], design=mo["design"],
                design_version=mo["version"], flight_log=mo["log"], z_over_semispan=mo["z_over_semispan"],
                chord_m=mo["chord_m"], twist_weight=mo["twist_weight"], frequency_hz=mo["frequency_hz"], k=mo["k"],
                re=round(mo["re_chord"]), alpha_eff_amplitude_deg=round(math.degrees(0.5 * (max(a) - min(a))), 3),
                alpha_eff_mean_deg=round(math.degrees(sum(a) / len(a)), 3),
                alpha_eff_range_deg=[round(math.degrees(min(a)), 3), round(math.degrees(max(a)), 3)],
                alpha_eff_cycle_deg=[round(math.degrees(x), 3) for x in a[::4]],
                pitch_range_deg=[round(math.degrees(min(p["theta"] for p in m["samples"])), 3),
                                 round(math.degrees(max(p["theta"] for p in m["samples"])), 3)],
                complete=ev.get("complete", False), partial=ev.get("partial", True),
                eta_s_per_cycle=ev.get("eta_s_per_cycle"),
                cycle_to_cycle_spread=(max(ev["eta_s_per_cycle"]) - min(ev["eta_s_per_cycle"])
                                       if len(ev.get("eta_s_per_cycle") or []) >= 2 else None),
                periodic_converged=bool(len(ev.get("eta_s_per_cycle") or []) >= 2
                                        and max(ev["eta_s_per_cycle"]) - min(ev["eta_s_per_cycle"]) <= 0.1),
                cycles_simulated=ev.get("cycles_simulated"), cycles_averaged=ev.get("cycles_averaged"),
                cc_mean=ev.get("cc_mean"), cl_mean=ev.get("cl_mean"), eta_s=None, eta_s_literal=None,
                eta_s_strip=ev.get("eta_s_strip"), strip_detail=dict(cd_f=cd_f, cd_f_from=cd_from,
                                                                       mean_cl_sin_a=ev.get("mean_cl_sin_a")),
                chordwise_history=hist, evidence=os.path.relpath(d, ROOT),
                note="No Garrick reference exists for this motion, so only the strip.js form of eta_s is given.")


def grid_comparison(base, points):
    """Same plunge on two grids, compared over their common simulated window. When the finer run stopped inside its
    start-up cycle this compares start-up transients only, and says so."""
    out = []
    plunge = [p for p in points if p.get("kind", "").startswith("pure plunge")]
    for p in plunge:
        for q in plunge:
            if p["level"] < q["level"] and (p["k"], p["h_over_c"], p["alpha0_deg"]) == (q["k"], q["h_over_c"], q["alpha0_deg"]):
                rows = {}
                for x in (p, q):
                    raw = bird3d.read_forces(os.path.join(base, x["name"]))
                    rows[x["name"]] = [(r[0], -r[1] / 0.05) for r in raw]
                end = min(r[-1][0] for r in rows.values())
                means = {}
                for nm, rr in rows.items():
                    sel = [r for r in rr if r[0] <= end + 1e-9]
                    means[nm] = sum(0.5 * (a[1] + b[1]) * (b[0] - a[0]) for a, b in zip(sel, sel[1:])) / (sel[-1][0] - sel[0][0])
                period = math.pi / p["k"]
                out.append(dict(coarse=p["name"], fine=q["name"], window_c_over_U=[0, round(end, 3)],
                                window_cycles=round(end / period, 3), cc_mean=means,
                                difference_rel=(means[q["name"]] - means[p["name"]]) / abs(means[p["name"]]),
                                note="window inside the start-up cycle: compares the start-up transient on both grids, "
                                     "not the cycle-averaged eta_s" if end < period else "common averaged window"))
    return out


def friction_drag_at(re_):
    """D_f at any Re between the V7 Re numbers: lowest converged Cd at 100k and 200k, interpolated in log Re."""
    lo, lo_from = friction_drag(100000)
    hi, hi_from = friction_drag(200000)
    if lo is None or hi is None:
        return lo if hi is None else hi, lo_from or hi_from
    f = min(1.0, max(0.0, math.log(re_ / 1e5) / math.log(2.0)))
    return lo * (hi / lo) ** f, f"log-Re interpolation of {lo_from} and {hi_from}"


def write():
    base = os.path.join(ROOT, "out/cfd/runs", GROUP)
    points = []
    for name in sorted(os.listdir(base)) if os.path.isdir(base) else []:
        p = os.path.join(base, name, "result.json")
        if os.path.exists(p):
            r = json.load(open(p))
        elif os.path.exists(os.path.join(base, name, "spec.json")):
            # Stopped before it wrote a result (on request): kept as a partial point, never dropped.
            r = json.load(open(os.path.join(base, name, "spec.json")))
            if r.get("kind") != "design-kinematics":
                r.update(diverged=False, stopped_early=True, ideal_suction=ideal_suction(r["k"], r["h_over_c"]),
                         theodorsen=theodorsen(r["k"]))
        else:
            continue
        if r.get("kind") == "design-kinematics":
            points.append(kinematics_point(base, name, r))
            continue
        level = float(name.split("-L")[1].split("-")[0])
        ref = steady_reference(r["re"], r["alpha_deg"], level)
        # Recomputed from the stored force history: every whole cycle after the first (start-up) one is averaged.
        span = 0.1
        raw = bird3d.read_forces(os.path.join(base, name))
        rows = [(q[0], -q[1] / (0.5 * span), q[2] / (0.5 * span)) for q in raw]
        n_avg = cycles_after_startup(rows, r["period"])
        usable = bool(n_avg >= 1 and not r.get("diverged"))
        cc_mean = cycle_mean(rows, r["period"], n_avg) if usable else None
        per_cycle = [cycle_mean([q for q in rows if q[0] <= rows[-1][0] - j * r["period"] + 1e-9], r["period"], 1)
                     for j in range(n_avg)][::-1] if usable else []
        eta = eta_lit = strip = None
        if usable and ref:
            eta = (cc_mean - ref["cc"]) / r["ideal_suction"]
            eta_lit = (cc_mean + ref["cd"]) / r["ideal_suction"]
        cd_f, cd_f_from = friction_drag(r["re"])
        if usable and cd_f is not None:
            strip = dict(strip_eta(raw, r, span, cd_f, average=n_avg), cd_f_from=cd_f_from)
        amp = math.degrees(math.atan(2 * r["k"] * r["h_over_c"]))
        points.append(dict(name=name, kind="pure plunge (protocol)", stopped_early=r.get("stopped_early", False), level=level, re=r["re"], k=r["k"], h_over_c=r["h_over_c"],
                           alpha0_deg=r["alpha_deg"], alpha_eff_amplitude_deg=amp,
                           alpha_eff_range_deg=[r["alpha_deg"] - amp, r["alpha_deg"] + amp],
                           chordwise_history=decimated_history(rows, r["period"]) if rows else None,
                           complete=usable, cycles_simulated=round(rows[-1][0] / r["period"], 2) if rows else 0,
                           cycles_averaged=n_avg, cc_mean=cc_mean, cc_per_averaged_cycle=per_cycle,
                           ideal_suction=r["ideal_suction"], theodorsen_F_G=r["theodorsen"], steady_reference=ref,
                           eta_s=eta, eta_s_literal=eta_lit,
                           eta_s_strip=strip and strip["eta"], strip_detail=strip, evidence=os.path.relpath(os.path.join(base, name), ROOT)))
    out_grid = grid_comparison(base, points)
    out = dict(
        grid_sensitivity=out_grid,
        quantity="leading-edge suction efficiency eta_s",
        summary="Every point run is listed; none is fitted or selected. Compare eta_s_strip against "
                "alpha_eff_amplitude_deg. Points with complete=false are partial and say how many cycles they hold. "
                "For the design-kinematics points eta_s_per_cycle gives each averaged cycle; periodic_converged is "
                "false when two cycles differ by more than 0.1 in eta_s (threshold set when these results were read, "
                "stated here so it can be judged), and then eta_s_strip is a two-cycle mean, not a settled value.",
        protocol="src/physics/strip.js SUCTION.to_measure",
        section="E387 coordinates (the design's 'e387-film' wing: film on both faces over E387 ribs). The 2 mm rod "
                "leading edge is not modelled separately: E387's own nose is used.",
        method="OpenFOAM 13, kOmegaSSTLM, Tu 0.1%, 2D C-grid (V7 settings), whole mesh in solid-body harmonic plunge "
               "h = h0 sin(omega t), consistent PIMPLE dt 0.0025 c/U scaled with the first-cell height, the first "
               "cycle discarded as start-up and every later whole cycle averaged (cycles_averaged per point).",
        definitions=dict(
            cc="chordwise force coefficient towards the leading edge, -Fx/(q c span), pressure plus viscous",
            ideal_suction="Garrick mean LE suction of a flat plate in pure plunge: pi |C(k)|^2 (k h0/b)^2, k = omega b/U",
            eta_s="(mean cc - cc_steady(alpha0)) / ideal_suction; cc_steady = Cl sin a0 - Cd cos a0 at the same Re, a0, grid",
            eta_s_literal="(mean cc + Cd_steady(alpha0)) / ideal_suction, the protocol's wording",
            eta_s_strip="mean(cc + Cd_f cos a) / mean(CL sin a) over the last two cycles: the eta that makes strip.js's "
                        "realised chordwise force, eta L sin a - D_f cos a, match the measured cycle mean; a is the "
                        "instantaneous effective angle from the plunge velocity, CL the measured lift normal to the "
                        "relative wind, Cd_f the lowest converged V7 Cd at that Re (strip.js's polar minimum drag). "
                        "This is the form to use in strip.js."),
        points=points,
        generated=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        command="python3 cfd/suction.py write")
    path = os.path.join(ROOT, "data/calibration/suction.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    print(path, len(points), "points")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("run", "write", "campaign"))
    ap.add_argument("coords", nargs="?")
    ap.add_argument("name", nargs="?")
    ap.add_argument("--k", type=float)
    ap.add_argument("--h", type=float)
    ap.add_argument("--alpha", type=float, default=4.0)
    ap.add_argument("--level", type=float, default=1.0)
    ap.add_argument("--re", type=int, default=100000)
    a = ap.parse_args()
    if a.what == "write":
        return write()
    if a.what == "run":
        print(json.dumps(run(a.coords, a.name, k=a.k, h=a.h, alpha=a.alpha, level=a.level, re_=a.re)))
        return None
    # Campaign: the Larus-like point on two grids, then a mild and a strong point. Two at a time.
    coords = os.path.join(ROOT, "data/references/airfoils/e387.dat")
    import threading
    # Mean incidence 4 deg (protocol: 0-6) so every case has a steady reference on its own grid: V7 main (level 1)
    # and the V7 grid study (level 0.667) both run alpha 4 at Re 100k.
    jobs = [dict(k=0.35, h=0.75, level=0.667), dict(k=0.35, h=0.75, level=1.0), dict(k=0.2, h=0.3, level=0.667),
            dict(k=0.5, h=1.0, level=0.667)]
    pool = pipeline.Pool(int(os.environ.get("AVIS_SUCTION_SLOTS", "1")))

    def job(j):
        pool._acquire()
        try:
            r = run(coords, "e387", k=j["k"], h=j["h"], alpha=4.0, level=j["level"], re_=100000)
            print(r["name"], r["complete"], r["cc_mean"], flush=True)
        except Exception as e:
            print("failed", j, repr(e), flush=True)
        finally:
            pool._release()
    threads = [threading.Thread(target=job, args=(j,)) for j in jobs]
    for t in threads:
        t.start()
        __import__("time").sleep(1)
    for t in threads:
        t.join()
    write()
    return None


if __name__ == "__main__":
    main()
