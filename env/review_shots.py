#!/usr/bin/env python3
"""Review stills of the film's flight shots, rendered through the film's own Blender path.

    python3 env/review_shots.py tag=before [shots=wide,track,...|all] [count=1] [samples=48] [res=1920x1080]
                                [out=out/env/review/pass3] [override=id:shot=track_side_long,lens=85]

Reads film/edit.json (never writes it) for each flight segment's scenario, shot, start, duration, slow motion
and lens, bakes the bird's pose with film/bake_pose.mjs and renders film/blender_shot.py into our own folder,
so a review still is exactly a film frame. Only `count` frames from the middle of the shot are rendered:
the rest of the shot window is filled with empty placeholders while Blender runs, and the rig is solved over the
whole shot (span=), as the film's holds are; then the placeholders are removed.

Output: <out>/<tag>/<segment id>_<shot>_<frame>.png and <out>/<tag>/timing.json (seconds per frame, first
frame of a hold excluded from the steady figure when count > 1). Every Blender call goes through
tools/run-guarded.py --gpu with a timeout under 600 s.
"""
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLENDER = "/Applications/Blender.app/Contents/MacOS/Blender"


def parse(argv):
    kw = dict(tag="review", shots="all", count="1", samples="", res="", out="out/env/review/pass3", timeout="590",
              edit="film/edit.json", override="")
    for p in argv:
        k, v = p.split("=", 1)
        if k == "override":
            kw["override"] = (kw["override"] + ";" + v).strip(";")
        else:
            kw[k] = v
    return kw


def overrides(text):
    """'track:shot=track_side_long,lens=85;under:lens=20' -> {'track': {'shot': 'track_side_long', 'lens': 85.0}}."""
    out = {}
    for item in filter(None, text.split(";")):
        sid, rest = item.split(":", 1)
        d = out.setdefault(sid, {})
        for kv in rest.split(","):
            k, v = kv.split("=", 1)
            d[k] = v if k in ("shot", "scenario") else float(v)
    return out


def plate(seg, fps):
    """Same arithmetic as film/render.mjs: frames are indices into the log resampled at fps * slow."""
    slow = seg.get("slow", 1)
    return round(seg["start_s"] * fps * slow), round(seg["dur"] * fps), slow


def picks(i0, n, count):
    mid = i0 + int(n * 0.55)
    return list(range(mid, min(mid + count, i0 + n)))


def run(cmd, timeout, log, gpu=False):
    g = ["python3", "tools/run-guarded.py", "--timeout", str(timeout), "--log", log] + (["--gpu"] if gpu else []) + ["--"]
    return subprocess.run(g + cmd, cwd=ROOT).returncode


def ensure_glb(design, log):
    """The bird GLB and every bake must come from the same design file: physics may move the design mid-batch, and
    film/blender_shot.py refuses a GLB/bake mismatch. Re-export the GLB whenever its design_sha256 is stale."""
    import hashlib
    sha = hashlib.sha256(open(os.path.join(ROOT, "data/designs/%s.json" % design), "rb").read()).hexdigest()
    meta = os.path.join(ROOT, "out/film/bird/%s.glb.json" % design)
    have = json.load(open(meta)).get("design_sha256") if os.path.exists(meta) else None
    if have != sha:
        print("[review] design changed, re-exporting the bird GLB", flush=True)
        if run(["node", "--import", "./film/lib/three-hooks.mjs", "film/export_glb.mjs", design], 300, log):
            raise SystemExit("GLB export failed, see %s" % log)


def bake_pose(seg, kw, fps, slow, i0, n, log):
    bake = os.path.join(ROOT, kw["out"], "bake", seg["id"])
    os.makedirs(os.path.dirname(bake), exist_ok=True)
    rc = run(["node", "--import", "./film/lib/three-hooks.mjs", "film/bake_pose.mjs", "out/flight/%s.json" % seg["scenario"],
              "--fps", str(fps * slow), "--from", str(i0), "--to", str(i0 + n), "--out", bake], 120, log)
    if rc:
        raise SystemExit("bake failed for %s (%d), see %s" % (seg["id"], rc, log))
    return bake


def placeholders(work, i0, n, want):
    """Empty files for every frame of the shot window we do not want, so Blender renders only `want`."""
    holders = []
    for i in range(i0, i0 + n):
        f = os.path.join(work, "f_%05d.png" % i)
        if i in want:
            if os.path.exists(f):
                os.remove(f)
        elif not os.path.exists(f):
            open(f, "wb").close()
            holders.append(f)
    return holders


def blender_args(seg, kw, fps, slow, bl, bake, work, i0, n):
    args = ["scenario=" + seg["scenario"], "shot=" + seg["shot"], "glb=out/film/bird/%s.glb" % kw["design"], "bake=" + bake,
            "frames=%d:%d" % (i0, i0 + n), "span=%d:%d" % (i0, i0 + n), "out=" + work, "fps=%g" % fps, "slow=%g" % slow, "res=" + (kw["res"] or bl["res"]),
            "samples=" + (kw["samples"] or str(bl["samples"])), "sun=%g" % bl.get("sun", 22), "world=%g" % bl.get("world", 0.4),
            "haze=%g" % bl.get("haze", 1.6e-5), "look=" + bl.get("look", "AgX - Medium High Contrast"),
            "exposure=%g" % bl.get("exposure", 0)]
    if seg.get("lens"):
        args.append("lens=%g" % seg["lens"])
    return args


def render_segment(seg, kw, fps, bl, dest):
    ensure_glb(kw["design"], os.path.join(ROOT, kw["out"], "logs", "export_glb.log"))
    i0, n, slow = plate(seg, fps)
    want = picks(i0, n, int(kw["count"]))
    work = os.path.join(ROOT, kw["out"], "work", seg["id"])
    log = os.path.join(ROOT, kw["out"], "logs", "%s-%s.log" % (kw["tag"], seg["id"]))
    os.makedirs(work, exist_ok=True)
    os.makedirs(os.path.dirname(log), exist_ok=True)
    args = blender_args(seg, kw, fps, slow, bl, bake_pose(seg, kw, fps, slow, i0, n, log), work, i0, n)
    tf = os.path.join(work, "timing.json")
    if os.path.exists(tf):
        os.remove(tf)
    holders = placeholders(work, i0, n, want)
    t0 = time.time()
    try:
        for attempt in range(3):  # Metal sometimes aborts while loading kernels; frames on disk are kept
            rc = run([BLENDER, "--background", "--factory-startup", "--python-exit-code", "7", "--python", "film/blender_shot.py",
                      "--"] + args, int(kw["timeout"]), log, gpu=True)
            if all(os.path.exists(os.path.join(work, "f_%05d.png" % i)) for i in want):
                break
            print("[review] %s: Blender exit %d, attempt %d" % (seg["id"], rc, attempt + 1), flush=True)
    finally:
        for f in holders:
            os.remove(f)
    times = json.load(open(tf)) if os.path.exists(tf) else {}
    os.makedirs(dest, exist_ok=True)
    for i in want:
        src = os.path.join(work, "f_%05d.png" % i)
        if os.path.exists(src):
            os.replace(src, os.path.join(dest, "%s_%s_%05d.png" % (seg["id"], seg["shot"], i)))
    vals = [times[str(i)] for i in want if str(i) in times]
    return dict(shot=seg["shot"], frames=want, lens=seg.get("lens"), seconds=vals, wall_s=round(time.time() - t0, 1),
                steady_s=round(sum(vals[1:]) / len(vals[1:]), 2) if len(vals) > 1 else None)


def main():
    kw = parse(sys.argv[1:])
    edit = json.load(open(os.path.join(ROOT, kw["edit"])))
    kw["design"] = edit["design"]
    fps = edit["fps"]
    bl = edit["blender"]
    ov = overrides(kw["override"])
    segs = [dict(s, **ov.get(s["id"], {})) for s in edit["segments"] if s.get("kind") == "bl"]
    if kw["shots"] != "all":
        keep = kw["shots"].split(",")
        segs = [s for s in segs if s["id"] in keep or s["shot"] in keep]
    dest = os.path.join(ROOT, kw["out"], kw["tag"])
    tfile = os.path.join(dest, "timing.json")
    os.makedirs(dest, exist_ok=True)
    allt = json.load(open(tfile)) if os.path.exists(tfile) else {}
    for seg in segs:
        r = render_segment(seg, kw, fps, bl, dest)
        allt[seg["id"]] = r
        json.dump(allt, open(tfile, "w"), indent=1)
        print("[review]", seg["id"], json.dumps(r), flush=True)


if __name__ == "__main__":
    main()
