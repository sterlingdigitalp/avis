#!/usr/bin/env python3
"""Gate V7: E387 CFD vs UIUC. Runs nothing itself: scores finished points and writes out/verification/V7.json.

Run it with the grid and sensitivity groups already finished (cfd/v7_campaign.py does both). It writes the gate
only when every declared point has a result, and an unconverged point counts as a failed point.

    python3 cfd/gates/v7.py [--level L]
"""
import argparse
import datetime
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, "out/cfd/runs")
ALPHAS = list(range(-2, 11))
RES = (100000, 200000)
CL_LIMIT, CD_LIMIT = 0.10, 0.25


def load(group):
    out = {}
    base = os.path.join(RUNS, group)
    if not os.path.isdir(base):
        return out
    for name in os.listdir(base):
        for fname in ("result.json", "failure.json"):
            path = os.path.join(base, name, fname)
            if os.path.exists(path):
                out[name] = json.load(open(path))
                break
    return out


def reference():
    """UIUC tables from research's reference file, checked identical to an independent PDF extraction."""
    path = os.path.join(ROOT, "data/references/airfoil-e387.json")
    d = json.load(open(path))
    u = d["uiuc_lsat_vol5"]
    own = json.load(open(os.path.join(ROOT, "out/cfd/ref/uiuc-e387e.json")))
    polars = {}
    for re_ in RES:
        rows = [dict(alpha_deg=r[0], cl=r[1], cd=r[2]) for r in u[f"re_{re_}"]["rows"]]
        if rows != own["polars"][str(re_)]["rows"]:
            raise SystemExit(f"reference file and PDF extraction disagree at Re {re_}")
        polars[str(re_)] = dict(rows=rows)
    nasa = {}
    for re_ in RES:
        n = d["nasa_tm_4062"][f"re_{re_}"]
        cols = n["columns"]
        ia, il, id_ = cols.index("alpha_deg"), cols.index("cl"), cols.index("cd")
        best = {}
        for r in n["rows"]:
            best.setdefault(r[ia], (r[il], r[id_]))
        nasa[str(re_)] = sorted((a, v[0], v[1]) for a, v in best.items())
    return dict(polars=polars, nasa=nasa, source=u["citation"], freestream_turbulence=own["freestream_turbulence"]), path


def nasa_at(table, alpha):
    """LTPT Cl, Cd at alpha, for context only. Cd may be null where the report prints none."""
    for a, b in zip(table, table[1:]):
        if a[0] <= alpha <= b[0] and b[0] > a[0]:
            f = (alpha - a[0]) / (b[0] - a[0])
            cl = a[1] + f * (b[1] - a[1])
            cd = None if a[2] is None or b[2] is None else a[2] + f * (b[2] - a[2])
            return cl, cd
    return None, None


def interp(rows, alpha, key):
    for a, b in zip(rows, rows[1:]):
        if a["alpha_deg"] <= alpha <= b["alpha_deg"]:
            f = (alpha - a["alpha_deg"]) / (b["alpha_deg"] - a["alpha_deg"])
            return a[key] + f * (b[key] - a[key])
    raise ValueError(f"alpha {alpha} outside reference")


def score(results, level, ref):
    points, missing = [], []
    for re_ in RES:
        rows = ref["polars"][str(re_)]["rows"]
        for alpha in ALPHAS:
            name = f"e387-L{level:g}-Re{re_}-a{alpha:g}-Tu0.1"
            r = results.get(name)
            if r is None:
                missing.append(name)
                continue
            cl_ref, cd_ref = interp(rows, alpha, "cl"), interp(rows, alpha, "cd")
            p = dict(re=re_, alpha_deg=alpha, status=r.get("status"), converged=bool(r.get("converged")),
                     cl_ref=round(cl_ref, 4), cd_ref=round(cd_ref, 5), cl=r.get("cl"), cd=r.get("cd"), cm=r.get("cm"),
                     yplus_max=r.get("yplus_max"), tu_measured_pct=r.get("tu_measured_pct"),
                     upper_separated=r.get("upper_separated"))
            if r.get("cl") is not None:
                p["cl_err"] = abs(r["cl"] - cl_ref) / abs(cl_ref)
                p["cd_err"] = abs(r["cd"] - cd_ref) / abs(cd_ref)
                p["cl_signed_err"] = (r["cl"] - cl_ref) / abs(cl_ref)
                p["cd_signed_err"] = (r["cd"] - cd_ref) / abs(cd_ref)
                p["cl_abs_err"] = r["cl"] - cl_ref
                p["passed"] = bool(p["converged"] and p["cl_err"] <= CL_LIMIT and p["cd_err"] <= CD_LIMIT)
                n_cl, n_cd = nasa_at(ref["nasa"][str(re_)], alpha)
                p["ltpt_cl"], p["ltpt_cd"] = n_cl, n_cd
                if n_cl:
                    p["cl_err_vs_ltpt"] = abs(r["cl"] - n_cl) / abs(n_cl)
                    p["uiuc_vs_ltpt_cl"] = abs(cl_ref - n_cl) / abs(n_cl)
                if n_cd:
                    p["cd_err_vs_ltpt"] = abs(r["cd"] - n_cd) / abs(n_cd)
                    p["uiuc_vs_ltpt_cd"] = abs(cd_ref - n_cd) / abs(n_cd)
            else:
                p["passed"] = False
            points.append(p)
    return points, missing


def tunnels(points):
    """Context required by the SPEC: CFD vs NASA LTPT, and UIUC vs LTPT. It never changes pass or fail."""
    out = {}
    for key in ("cl_err_vs_ltpt", "cd_err_vs_ltpt", "uiuc_vs_ltpt_cl", "uiuc_vs_ltpt_cd"):
        vals = [p for p in points if p.get(key) is not None]
        if vals:
            top = max(vals, key=lambda p: p[key])
            out[key] = dict(max=round(top[key], 4), re=top["re"], alpha_deg=top["alpha_deg"],
                            mean=round(sum(p[key] for p in vals) / len(vals), 4))
    return out


def summary(points):
    done = [p for p in points if "cl_err" in p]
    if not done:
        return {}

    def worst(key):
        p = max(done, key=lambda q: q[key])
        return dict(value=round(p[key], 4), re=p["re"], alpha_deg=p["alpha_deg"])

    def mean(key):
        return round(sum(p[key] for p in done) / len(done), 4)
    return dict(points=len(points), points_passed=sum(p["passed"] for p in points),
                points_unconverged=sum(not p["converged"] for p in points),
                cl_max_err=worst("cl_err"), cd_max_err=worst("cd_err"), cl_mean_err=mean("cl_err"),
                cd_mean_err=mean("cd_err"),
                yplus_max=max(p["yplus_max"] or 0 for p in done),
                tu_measured_pct_range=[min(p["tu_measured_pct"] or 0 for p in done),
                                       max(p["tu_measured_pct"] or 0 for p in done)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", type=float, default=1.0, help="grid level the gate is scored on")
    a = ap.parse_args()
    ref, ref_path = reference()
    results = load("v7")
    points, missing = score(results, a.level, ref)
    if missing:
        print(f"NOT WRITING V7: {len(missing)} declared points have no result, e.g. {missing[:3]}")
        return 2
    grid = load("v7-grid")
    sens = load("v7-tu")
    passed = all(p["passed"] for p in points)
    gate = dict(gate="V7", design=None, passed=passed,
                measured=dict(summary(points), level=a.level, context_not_scored=tunnels(points), failed_points=[
                    {k: (round(p[k], 4) if isinstance(p.get(k), float) else p.get(k))
                     for k in ("re", "alpha_deg", "status", "cl", "cl_err", "cd", "cd_err")}
                    for p in points if not p["passed"]]),
                limit=dict(cl_relative=CL_LIMIT, cd_relative=CD_LIMIT, applied="every point, Re 100k and 200k, alpha -2 to 10"),
                evidence="out/cfd/v7/report.json",
                ran_at=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
                command="python3 cfd/gates/v7.py " + " ".join(sys.argv[1:]))
    # UIUC and NASA values appear only in this local report under out/, never in cfd/ or the gate summary: see SPEC.
    report = dict(points=points, reference=ref["source"], reference_file=os.path.relpath(ref_path, ROOT), grid_study=grid, tu_sensitivity=sens,
                  turbulence=ref["freestream_turbulence"])
    os.makedirs(os.path.join(ROOT, "out/cfd/v7"), exist_ok=True)
    json.dump(report, open(os.path.join(ROOT, "out/cfd/v7/report.json"), "w"), indent=1)
    os.makedirs(os.path.join(ROOT, "out/verification"), exist_ok=True)
    json.dump(gate, open(os.path.join(ROOT, "out/verification/V7.json"), "w"), indent=1)
    print(json.dumps({k: gate["measured"][k] for k in gate["measured"] if k != "failed_points"}))
    print("V7", "PASS" if passed else "FAIL")
    return 0


if __name__ == "__main__":
    sys.exit(main())
