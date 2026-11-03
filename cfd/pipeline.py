#!/usr/bin/env python3
"""Run 2D airfoil cases: steady first, transient fallback when steady will not settle.

Never hides a bad point: every case ends in a result.json with an explicit status.
  status: steady | steady-mean | transient-mean | unconverged-steady | unconverged-transient | diverged | failed
`converged` is true only for steady, steady-mean and transient-mean. steady-mean is a small limit cycle about a stable
mean; its amplitude is always reported, and cfd/README.md records how it was checked against time-accurate runs.
"""
import json
import math
import os
import re
import shutil
import subprocess
import threading
import time

import case as casegen

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

STEADY = dict(window=2000, min_iter=4000, max_iter=12000, cl_tol=0.005, cd_tol=0.01,
              mean_after=8000, mean_cl_tol=0.005, mean_cd_tol=0.01, amp_cl=0.04, amp_cd=0.15)
# Windows of 5 c/U hold many shedding cycles: lift crossings at Re 100k, alpha -2 came every 0.1 to 0.3 c/U.
# dt 0.005 c/U (Courant about 9) diverged after 4.4 c/U at Re 100k alpha -2; 0.0025 (Courant about 4) was stable as far as
# it was run (1.3 c/U). A transient that still diverges is recorded as diverged with its steady values flagged.
# max_time is a budget, not a criterion: on the shared host a transient point costs hours, so the run stops after two
# windows plus 2 c/U of start-up; a point whose two window means disagree is flagged unconverged-transient.
TRANSIENT = dict(window=5.0, min_time=10.0, max_time=12.0, cl_tol=0.01, cd_tol=0.02, dt=0.0025)
# Docker quota stays at the 8 CPU limit with few ranks inside it. Measured: 8 ranks in an 8 CPU quota are throttled in
# a third of periods, and two 6-rank cases ran under 1 it/s (from 23) when the shared machine's load reached 180.
# Fewer ranks lose far less to a pre-empted rank stalling every collective.
CPUS, RANKS, MEMORY = 8, 3, "16g"


def force_files(d):
    base = os.path.join(d, "postProcessing", "forceCoeffs1")
    if not os.path.isdir(base):
        return []
    starts = sorted(os.listdir(base), key=float)
    return [os.path.join(base, s, "forceCoeffs.dat") for s in starts]


def read_forces(d):
    """Rows (t, Cm, Cd, Cl), oldest first. A NaN token is kept as float nan."""
    rows = []
    for path in force_files(d):
        if not os.path.exists(path):
            continue
        with open(path) as f:
            for line in f:
                if line.startswith("#"):
                    continue
                p = line.split()
                if len(p) >= 4:
                    try:
                        rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
                    except ValueError:
                        pass
    return rows


def stats(rows):
    n = len(rows)
    cm, cd, cl = [r[1] for r in rows], [r[2] for r in rows], [r[3] for r in rows]
    return dict(cl=sum(cl) / n, cd=sum(cd) / n, cm=sum(cm) / n, cl_min=min(cl), cl_max=max(cl), cd_min=min(cd),
                cd_max=max(cd), cm_min=min(cm), cm_max=max(cm), samples=n)


def bad(rows):
    return any(not math.isfinite(v) or abs(v) > 1e3 for v in rows[-1][1:]) if rows else False


def steady_settled(rows, cfg):
    if not rows or rows[-1][0] < cfg["min_iter"]:
        return False
    tail = [r for r in rows if r[0] > rows[-1][0] - cfg["window"]]
    s = stats(tail)
    return ((s["cl_max"] - s["cl_min"]) / max(abs(s["cl"]), 0.1) < cfg["cl_tol"]
            and (s["cd_max"] - s["cd_min"]) / max(abs(s["cd"]), 1e-3) < cfg["cd_tol"])


def steady_mean_settled(rows, cfg):
    """Small limit cycle: consecutive window means agree and the oscillation amplitude is bounded."""
    if not rows or rows[-1][0] < cfg["mean_after"]:
        return False
    end, w = rows[-1][0], cfg["window"]
    a = [r for r in rows if end - 2 * w < r[0] <= end - w]
    b = [r for r in rows if r[0] > end - w]
    if not a or not b:
        return False
    sa, sb = stats(a), stats(b)
    cl, cd = max(abs(sb["cl"]), 0.1), max(abs(sb["cd"]), 1e-3)
    return (abs(sa["cl"] - sb["cl"]) / cl < cfg["mean_cl_tol"] and abs(sa["cd"] - sb["cd"]) / cd < cfg["mean_cd_tol"]
            and (sb["cl_max"] - sb["cl_min"]) / cl < cfg["amp_cl"] and (sb["cd_max"] - sb["cd_min"]) / cd < cfg["amp_cd"])


def transient_settled(rows, t0, cfg):
    if not rows or rows[-1][0] - t0 < cfg["min_time"]:
        return False
    end, w = rows[-1][0], cfg["window"]
    a = [r for r in rows if end - 2 * w < r[0] <= end - w]
    b = [r for r in rows if r[0] > end - w]
    if not a or not b:
        return False
    sa, sb = stats(a), stats(b)
    return (abs(sa["cl"] - sb["cl"]) / max(abs(sb["cl"]), 0.1) < cfg["cl_tol"]
            and abs(sa["cd"] - sb["cd"]) / max(abs(sb["cd"]), 1e-3) < cfg["cd_tol"])


def container(name, d, script, max_seconds, cpus, memory, on_poll):
    """Run the script; on_poll() returns 'kill', 'stop' or None every few seconds."""
    log = open(os.path.join(d, f"log.{script}"), "w")
    proc = subprocess.Popen(
        [os.path.join(HERE, "run-case.sh"), d, name, str(cpus), memory, str(max_seconds), script],
        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
    killed = stopped = False
    while proc.poll() is None:
        time.sleep(10)
        verdict = on_poll()
        if verdict == "stop" and not stopped:
            open(os.path.join(d, "stop"), "w").close()
            stopped = True
        elif verdict == "kill" and not killed:
            subprocess.run(["docker", "rm", "-f", f"avis-cfd-{name}"], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
            killed = True
    log.close()
    return proc.returncode, killed


# Cases run on the container's own disk. Measured: an append to the bind-mounted case took 61 ms under host load
# (0.08 ms on container disk), and the solver writes a dozen log lines and checks the stop file every iteration, so
# rank 0 blocked on host I/O while the others spun. Forces sync back every 15 s for the host's convergence poll; the
# host's stop file is relayed in; everything is copied back when the run ends.
LOCAL_PROLOGUE = """set -u
rm -f /work/stop; R=/tmp/case; rm -rf $R; mkdir -p $R; cp -a /work/. $R/; cd $R
sync_back() {{ mkdir -p /work/postProcessing; cp -a postProcessing/. /work/postProcessing/ 2>/dev/null || true; ckpt; }}
# Checkpoint: the second-newest processor time directory is complete (purgeWrite 2), so copy it to the host and keep
# only that one there, so a killed run can still be resumed from its last whole checkpoint.
ckpt() {{ for p in processor*; do [ -d "$p" ] || continue; ts=$(ls -1 "$p" | grep -E '^[0-9.e+-]+$' | grep -vx 0 | sort -g); [ $(echo "$ts" | grep -c .) -ge 2 ] || continue; t=$(echo "$ts" | tail -2 | head -1); [ -d "/work/$p/$t" ] && continue; mkdir -p "/work/$p"; [ -d "/work/$p/constant" ] || cp -a "$p/constant" "/work/$p/"; cp -a "$p/$t" "/work/$p/"; for o in $(ls -1 "/work/$p" | grep -E '^[0-9.e+-]+$' | grep -vx 0 | grep -vx "$t"); do rm -rf "/work/$p/$o"; done; done; }}
( while true; do sleep 15; [ -f /work/stop ] && touch $R/stop; sync_back; done ) &
SYNC=$!
finish() {{ rc=$1; kill $SYNC 2>/dev/null; sync_back; cp -a $R/. /work/ 2>/dev/null; exit $rc; }}
"""  # a str.format template: braces doubled

STEADY_SCRIPT = LOCAL_PROLOGUE + """
decomposePar -force > log.decomposePar 2>&1 || finish 11
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} potentialFoam -parallel -initialiseUBCs -pName p > log.potentialFoam 2>&1 || finish 12
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} foamRun -parallel > log.foamRun 2>&1 || finish 13
reconstructPar -latestTime > log.reconstructPar 2>&1 || finish 14
finish 0
"""

TRANSIENT_SCRIPT = LOCAL_PROLOGUE + """
mpirun --allow-run-as-root --bind-to none --mca mpi_yield_when_idle 1 -np {n} foamRun -parallel > log.foamRun.transient 2>&1 || finish 13
reconstructPar -latestTime > log.reconstructPar 2>&1 || finish 14
finish 0
"""


def latest_time_dir(d):
    best = None
    for name in os.listdir(d):
        try:
            v = float(name)
        except ValueError:
            continue
        if v > 0 and (best is None or v > best[0]):
            best = (v, name)
    return best[1] if best else None


def boundary_values(path, patch):
    """Scalars or vectors of one patch's value entry from an ascii field file."""
    with open(path) as f:
        text = f.read()
    i = text.find("boundaryField")
    m = re.compile(r"\b" + patch + r"\s*\{").search(text, i)
    if not m:
        return None
    end = text.find("}", m.end())
    block = text[m.end():end]
    v = re.search(r"value\s+nonuniform\s+List<(\w+)>\s*(\d+)\s*\(", block)
    if not v:
        return None
    start = v.end()
    body = block[start:block.rfind(")")]
    if v.group(1) == "vector":
        return [tuple(float(a) for a in t.split()) for t in re.findall(r"\(([^()]+)\)", body)]
    return [float(t) for t in body.split()]


def internal_values(path):
    """Scalar internalField of an ascii field file, or None when it is uniform."""
    with open(path) as f:
        text = f.read()
    m = re.search(r"internalField\s+nonuniform\s+List<scalar>\s*(\d+)\s*\(", text)
    if not m:
        return None
    n = int(m.group(1))
    return [float(v) for v in text[m.end():text.index(")", m.end())].split()[:n]]


def wall_pressure(path, mesh):
    """Wall pressure per airfoil face. A zeroGradient wall writes no values, so take the wall-adjacent cell: in this
    C-grid the cell on wall face k is cell n_wake + k (cgrid.py numbers cells i + j * (ni - 1))."""
    p = boundary_values(path, "airfoil")
    if p:
        return p
    cells = internal_values(path)
    if cells is None:
        return None
    first, n = mesh["n_wake"], len(mesh["wall"]) - 1
    return cells[first:first + n]


def extract_wall(d, mesh_dir):
    """Cp and Cf along the airfoil; separated regions on each surface. x/c in the airfoil frame."""
    t = latest_time_dir(d)
    if t is None:
        return None
    mesh = json.load(open(os.path.join(mesh_dir, "mesh.json")))
    wall = mesh["wall"]
    paths = [os.path.join(d, t, f) for f in ("p", "wallShearStress", "yPlus")]
    if not all(os.path.exists(f) for f in paths):
        return None
    p = wall_pressure(paths[0], mesh)
    tau, yp = (boundary_values(f, "airfoil") for f in paths[1:])
    if not p or not tau or len(p) != len(wall) - 1 or len(tau) != len(p):
        return None
    faces = []
    for k in range(len(p)):
        a, b = wall[k], wall[k + 1]
        ds = math.hypot(b[0] - a[0], b[1] - a[1])
        tx, ty = (b[0] - a[0]) / ds, (b[1] - a[1]) / ds  # lower TE -> LE -> upper TE
        faces.append(dict(x=0.5 * (a[0] + b[0]), y=0.5 * (a[1] + b[1]), cp=2.0 * p[k],
                          cf=2.0 * (tau[k][0] * tx + tau[k][1] * ty)))
    # Split at the leading-edge vertex: faces before it are the lower surface, faces after it the upper.
    i_le = min(range(len(wall)), key=lambda k: wall[k][0])
    upper = faces[i_le:]
    lower = list(reversed(faces[:i_le]))
    # Along the curve the upper surface runs LE to TE. The solver's sign is fixed from data below.
    return dict(time=float(t), upper=upper, lower=lower, yplus=yp)


def separated_regions(side, sign):
    """Runs of reversed flow in x/c on one surface given in LE-to-TE order."""
    out, start = [], None
    prev = None
    for f in side:
        rev = sign * f["cf"] < 0
        if rev and start is None:
            start = f["x"]
        if not rev and start is not None:
            out.append([round(start, 4), round(prev, 4)])
            start = None
        prev = f["x"]
    if start is not None:
        out.append([round(start, 4), round(prev, 4), "open"])
    return out


def wall_summary(w):
    """Bubble positions and y+. Sign: upper-surface shear in flow direction is LE to TE along the curve."""
    if not w:
        return dict(wall_data=False)
    # The solver reports wall shear as force on fluid or on wall by version, so pin the sign from physics:
    # the first 2% of the upper surface past the suction peak is attached for attached-flow cases, and the
    # chord-integrated upper surface is forward-dominated at the start of the run of faces.
    head = [f for f in w["upper"] if 0.005 < f["x"] < 0.05]
    sign = 1.0 if sum(f["cf"] for f in head) >= 0 else -1.0
    yp = w["yplus"] or [0.0]
    return dict(wall_data=True, upper_separated=separated_regions(w["upper"], sign),
                lower_separated=separated_regions(w["lower"], -sign),
                yplus_max=max(yp), yplus_mean=sum(yp) / len(yp), yplus_over_1_frac=sum(v > 1 for v in yp) / len(yp),
                cf_sign_applied=sign)


def probe_tu(d):
    base = os.path.join(d, "postProcessing", "upstream")
    try:
        dirs = sorted(os.listdir(base), key=float)
    except OSError:
        return None
    for s in reversed(dirs):
        path = os.path.join(base, s, "k")
        if os.path.exists(path):
            rows = [ln.split() for ln in open(path) if not ln.startswith("#") and ln.strip()]
            if rows:
                return 100.0 * math.sqrt(2.0 * float(rows[-1][1]) / 3.0)
    return None


FIELDS = ("U", "p", "k", "omega", "nut", "gammaInt", "ReThetat")


def internal_block(text):
    """The `internalField ...;` statement of an ascii field file."""
    i = text.index("internalField")
    j = text.index("boundaryField", i)
    return text[i:text.rindex(";", i, j) + 1]


def warm_start(d, src):
    """Replace the uniform initial internal fields with the converged fields saved by a neighbouring point."""
    for name in FIELDS:
        path = os.path.join(d, "0", name)
        saved = os.path.join(src, "final", name)
        text = open(path).read()
        block = internal_block(open(saved).read())
        text = text.replace(internal_block(text), block, 1)
        with open(path, "w") as f:
            f.write(text)


def save_final(d):
    """Keep the converged internal fields for the next point of a sweep (removed when that point is done)."""
    t = latest_time_dir(d)
    if t is None or not all(os.path.exists(os.path.join(d, t, name)) for name in FIELDS):
        return False
    os.makedirs(os.path.join(d, "final"), exist_ok=True)
    for name in FIELDS:
        shutil.copy(os.path.join(d, t, name), os.path.join(d, "final", name))
    return True


def set_transient(d, spec, dt, end_time):
    a = math.radians(spec["alpha_deg"])
    meta_span = json.load(open(os.path.join(ROOT, "out/cfd/meshes", spec["mesh"], "mesh.json")))["span"]
    casegen.write_system(d, mode="transient", end_time=end_time, dt=dt, span=meta_span, a=a, ncpu=spec["ncpu"],
                         write_interval=end_time)


def clean_keep_restart(d):
    """Like clean, but keeps what a restart needs: system, constant, the processor meshes and each processor's latest
    time directory, so the case can be extended later (suction_kin.py extend)."""
    for name in os.listdir(d):
        path = os.path.join(d, name)
        if name.startswith("processor"):
            times = sorted((t for t in os.listdir(path) if re.fullmatch(r"[0-9.e+-]+", t) and t != "0"), key=float)
            for t in times[:-1]:
                shutil.rmtree(os.path.join(path, t), ignore_errors=True)
        elif name == "dynamicCode":
            shutil.rmtree(path, ignore_errors=True)
        elif name not in ("0",) and re.fullmatch(r"[0-9.e+-]+", name):
            shutil.rmtree(path, ignore_errors=True)


def latest_processor_time(d):
    p = os.path.join(d, "processor0")
    if not os.path.isdir(p):
        return None
    times = [t for t in os.listdir(p) if re.fullmatch(r"[0-9.e+-]+", t) and t != "0"]
    return max(times, key=float) if times else None


def clean(d):
    for name in os.listdir(d):
        path = os.path.join(d, name)
        if name.startswith("processor") or name in ("constant", "dynamicCode"):
            shutil.rmtree(path, ignore_errors=True)
        elif name not in ("0",) and re.fullmatch(r"[0-9.e+-]+", name):
            shutil.rmtree(path, ignore_errors=True)
    shutil.rmtree(os.path.join(d, "0"), ignore_errors=True)


def classify_steady(rows, diverged, cfg):
    """Status fields for the steady stage, from its force history. Pure."""
    out = dict(cl=None, cd=None, cm=None, converged=False, status="diverged" if diverged else "failed")
    if diverged or not rows:
        return out
    settled = steady_settled(rows, cfg)
    mean = not settled and steady_mean_settled(rows, cfg)
    tail = [r for r in rows if r[0] > rows[-1][0] - cfg["window"]]
    s = stats(tail)
    status = "steady" if settled else "steady-mean" if mean else "unconverged-steady"
    out.update(cl=s["cl"], cd=s["cd"], cm=s["cm"], converged=settled or mean, status=status,
               steady=dict(s, iterations=rows[-1][0], settled=settled, limit_cycle=mean))
    return out


def classify_transient(rows, t0, diverged, cfg):
    """Status fields for the transient stage, or a failure with no values of its own. Pure."""
    if diverged or not rows:
        return dict(status="diverged" if diverged else "failed", converged=False)
    settled = transient_settled(rows, t0, cfg)
    s = stats([r for r in rows if r[0] > rows[-1][0] - cfg["window"]])
    return dict(cl=s["cl"], cd=s["cd"], cm=s["cm"], converged=settled,
                status="transient-mean" if settled else "unconverged-transient",
                transient=dict(s, elapsed=rows[-1][0] - t0, settled=settled, dt=cfg["dt"]))


def steady_poller(d, cfg, state):
    def poll():
        rows = read_forces(d)
        if bad(rows):
            state["diverged"] = True
            return "kill"
        if steady_settled(rows, cfg) or steady_mean_settled(rows, cfg):
            return "stop"
        return None
    return poll


def transient_poller(d, t0, cfg, state):
    def poll():
        rows = [r for r in read_forces(d) if r[0] > t0]
        if bad(rows):
            state["diverged"] = True
            return "kill"
        return "stop" if transient_settled(rows, t0, cfg) else None
    return poll


def run_transient_stage(d, tag, spec, cold, mesh_dir, ranks, cpus, memory, cfg, kw):
    """Time-accurate stage. A blown-up or failed steady run has no usable fields, so that case restarts cold."""
    if cold:
        casegen.make_case(d, mesh_dir, mode="transient", end_time=1, dt=cfg["dt"], ncpu=ranks, **kw)
        with open(os.path.join(d, "transient.sh"), "w") as f:
            f.write(STEADY_SCRIPT.format(n=ranks).replace("log.foamRun", "log.foamRun.transient"))
        t0 = 0.0
    else:
        t0 = float(latest_time_dir(d))
        if os.path.exists(os.path.join(d, "stop")):
            os.remove(os.path.join(d, "stop"))
        with open(os.path.join(d, "transient.sh"), "w") as f:  # current script, also for a resumed point
            f.write(TRANSIENT_SCRIPT.format(n=ranks))
    set_transient(d, spec, cfg["dt"], t0 + cfg["max_time"])
    state = dict(diverged=False)
    # A distinct name: the steady stage's sidecar removes its container by name shortly after that stage ends.
    rc, _ = container(tag + "-t", d, "transient.sh", 6 * 3600, cpus, memory, transient_poller(d, t0, cfg, state))
    rows = [r for r in read_forces(d) if r[0] > t0]
    return dict(classify_transient(rows, t0, state["diverged"], cfg), transient_rc=rc, transient_cold_start=cold)


def run_point(group, name, mesh_dir, *, re_, alpha, tu=0.10, cpus=CPUS, ranks=RANKS, memory=MEMORY, allow_transient=True,
              keep=False, steady_cfg=None, transient_cfg=None, force_transient=False, init_from=None):
    """Run one polar point. Resumable: a finished result.json is returned untouched."""
    d = os.path.join(ROOT, "out/cfd/runs", group, name)
    done = os.path.join(d, "result.json")
    if os.path.exists(done):
        return json.load(open(done))
    sc = dict(STEADY, **(steady_cfg or {}))
    tc = dict(TRANSIENT, **(transient_cfg or {}))
    # dt was found stable on the level-1 grid (first cell 7e-5 c); finer grids get proportionally smaller steps so the
    # wall Courant number stays where it was tested.
    y1 = json.load(open(os.path.join(mesh_dir, "mesh.json")))["y1"]
    tc["dt"] = tc["dt"] * min(1.0, y1 / 7.0e-5)
    t_start = time.time()
    kw = dict(re=re_, alpha_deg=alpha, tu_pct=tu)
    tag = re.sub(r"[^a-zA-Z0-9_.-]", "-", f"{group}-{name}")[:50]
    resumed = resume_steady(d, re_, alpha, tu)
    if resumed:
        spec, done_ = resumed
        return finish_point(d, group, name, mesh_dir, spec, tag, done_["rc"], done_["diverged"], sc, tc, kw, ranks, cpus,
                            memory, allow_transient, force_transient, keep, t_start)
    spec = casegen.make_case(d, mesh_dir, mode="steady", end_time=sc["max_iter"], ncpu=ranks, **kw)
    if init_from and os.path.isdir(os.path.join(init_from, "final")):
        warm_start(d, init_from)
        # The start is a converged neighbour, not potential flow, so the minimum run is shorter. Settling tests unchanged.
        sc = dict(sc, min_iter=2500, mean_after=5000)
        spec["warm_start_from"] = os.path.basename(init_from)
        with open(os.path.join(d, "spec.json"), "w") as f:
            json.dump(spec, f, indent=1)
    else:
        init_from = None
    with open(os.path.join(d, "steady.sh"), "w") as f:
        script = STEADY_SCRIPT.format(n=ranks)
        if init_from:  # potentialFoam would overwrite the warm-start velocity
            script = "\n".join(l for l in script.splitlines() if "potentialFoam" not in l) + "\n"
        f.write(script)
    with open(os.path.join(d, "transient.sh"), "w") as f:
        f.write(TRANSIENT_SCRIPT.format(n=ranks))
    tag = re.sub(r"[^a-zA-Z0-9_.-]", "-", f"{group}-{name}")[:50]
    state = dict(diverged=False)
    rc, _ = container(tag + "-s", d, "steady.sh", 4 * 3600, cpus, memory, steady_poller(d, sc, state))
    with open(os.path.join(d, "steady.done"), "w") as f:
        json.dump(dict(rc=rc, diverged=state["diverged"]), f)
    return finish_point(d, group, name, mesh_dir, spec, tag, rc, state["diverged"], sc, tc, kw, ranks, cpus, memory,
                        allow_transient, force_transient, keep, t_start)


def resume_steady(d, re_, alpha, tu):
    """A finished steady stage from an interrupted run: its spec, rc and divergence flag, with any partial
    transient output removed. None when there is nothing safe to resume."""
    try:
        spec = json.load(open(os.path.join(d, "spec.json")))
        done = json.load(open(os.path.join(d, "steady.done")))
    except (OSError, ValueError):
        return None
    if (spec["re"], spec["alpha_deg"], spec["tu_pct"], spec["mode"]) != (re_, alpha, tu, "steady"):
        return None
    t0 = latest_time_dir(d)
    if done["diverged"] or done["rc"] != 0 or t0 is None:
        return None
    for name in os.listdir(d):
        if name.startswith("processor"):
            for t in os.listdir(os.path.join(d, name)):
                try:
                    if float(t) > float(t0):
                        shutil.rmtree(os.path.join(d, name, t))
                except ValueError:
                    pass
    for fo in os.listdir(os.path.join(d, "postProcessing")):
        for t in os.listdir(os.path.join(d, "postProcessing", fo)):
            if float(t) >= float(t0) and float(t) > 0:
                shutil.rmtree(os.path.join(d, "postProcessing", fo, t))
    for f in ("stop", "log.foamRun.transient"):
        if os.path.exists(os.path.join(d, f)):
            os.remove(os.path.join(d, f))
    return spec, done


def finish_point(d, group, name, mesh_dir, spec, tag, rc, diverged, sc, tc, kw, ranks, cpus, memory, allow_transient,
                 force_transient, keep, t_start):
    done = os.path.join(d, "result.json")
    steady = classify_steady(read_forces(d), diverged, sc)  # only steady output exists at this point
    result = dict(spec, name=name, group=group, steady_rc=rc, **steady)
    if (not steady["converged"] and allow_transient) or force_transient:
        cold = diverged or rc != 0 or latest_time_dir(d) is None
        result.update(run_transient_stage(d, tag, spec, cold, mesh_dir, ranks, cpus, memory, tc, kw))
    wall = extract_wall(d, mesh_dir)
    result.update(wall_summary(wall))
    result["tu_measured_pct"] = probe_tu(d)
    result["wall_seconds"] = round(time.time() - t_start)
    if wall:
        with open(os.path.join(d, "wall.json"), "w") as f:
            json.dump(wall, f)
    # Only a converged steady field seeds the next point; anything else would carry its problem forward.
    result["final_saved"] = bool(result.get("status") in ("steady", "steady-mean") and save_final(d))
    with open(done + ".tmp", "w") as f:
        json.dump(result, f, indent=1)
    os.replace(done + ".tmp", done)
    if not keep:
        clean(d)
    return result


SLOTS_FILE = os.path.join(ROOT, "out/cfd/slots")


def allowed_slots(default):
    """Concurrent 2D cases allowed now. out/cfd/slots lets a 3D case take a slot without stopping the programme."""
    try:
        return max(0, min(default, int(open(SLOTS_FILE).read().strip())))
    except (OSError, ValueError):
        return default


class Pool:
    """At most `size` cases at a time, fewer while out/cfd/slots says so. Checked before each case starts."""

    def __init__(self, size=2):
        self.size = size
        self.running = 0
        self.threads = []
        self.results = []
        self.lock = threading.Lock()

    def _acquire(self):
        while True:
            with self.lock:
                if self.running < allowed_slots(self.size):
                    self.running += 1
                    return
            time.sleep(10)

    def _release(self):
        with self.lock:
            self.running -= 1

    def submit(self, *args, **kw):
        def work():
            self._acquire()
            try:
                try:
                    r = run_point(*args, **kw)
                except Exception as e:  # recorded, never dropped
                    r = dict(group=args[0], name=args[1], status="failed", converged=False, error=repr(e),
                             re=kw.get("re_"), alpha_deg=kw.get("alpha"), cl=None, cd=None, cm=None)
                    d = os.path.join(ROOT, "out/cfd/runs", args[0], args[1])
                    os.makedirs(d, exist_ok=True)
                    with open(os.path.join(d, "failure.json"), "w") as f:
                        json.dump(r, f, indent=1)
            finally:
                self._release()
            with self.lock:
                self.results.append(r)
                print(f"{time.strftime('%H:%M:%S')} {r.get('name')} {r.get('status')} "
                      f"Cl={r.get('cl')} Cd={r.get('cd')} {r.get('wall_seconds')}s", flush=True)
        t = threading.Thread(target=work, daemon=True)
        t.start()
        self.threads.append(t)
        time.sleep(0.05)  # keeps submission order as start order

    def submit_sweep(self, group, points, mesh_dir, **kw):
        """points: [(name, re, alpha)] in sweep order. One slot per running point; each point warm-starts from the
        last converged point before it in the same sweep, and that point's saved field is then dropped."""
        def work():
            seed = None
            for name, re_, alpha in points:
                self._acquire()
                try:
                    try:
                        r = run_point(group, name, mesh_dir, re_=re_, alpha=alpha, init_from=seed, **kw)
                    except Exception as e:
                        r = dict(group=group, name=name, status="failed", converged=False, error=repr(e), re=re_,
                                 alpha_deg=alpha, cl=None, cd=None, cm=None)
                        d = os.path.join(ROOT, "out/cfd/runs", group, name)
                        os.makedirs(d, exist_ok=True)
                        with open(os.path.join(d, "failure.json"), "w") as f:
                            json.dump(r, f, indent=1)
                finally:
                    self._release()
                d = os.path.join(ROOT, "out/cfd/runs", group, name)
                if os.path.isdir(os.path.join(d, "final")):
                    if seed and os.path.isdir(os.path.join(seed, "final")):
                        shutil.rmtree(os.path.join(seed, "final"))
                    seed = d
                with self.lock:
                    self.results.append(r)
                    print(f"{time.strftime('%H:%M:%S')} {r.get('name')} {r.get('status')} "
                          f"Cl={r.get('cl')} Cd={r.get('cd')} {r.get('wall_seconds')}s", flush=True)
            if seed and os.path.isdir(os.path.join(seed, "final")):
                shutil.rmtree(os.path.join(seed, "final"))
        t = threading.Thread(target=work, daemon=True)
        t.start()
        self.threads.append(t)
        time.sleep(0.05)

    def wait(self):
        for t in self.threads:
            t.join()
        return self.results


def build_mesh(coords, name, level, radius=40.0):
    out = os.path.join(ROOT, "out/cfd/meshes", name)
    if not os.path.exists(os.path.join(out, "mesh.json")):
        subprocess.run(["python3", os.path.join(HERE, "cgrid.py"), coords, out, "--level", str(level),
                        "--radius", str(radius)], check=True, stdout=subprocess.DEVNULL)
    return out
