#!/usr/bin/env python3
"""Build tree meshes into out/env/cache/trees/ on the CPU, in parallel, so a render never generates geometry while it
holds the machine-wide GPU lock. Same file names and content keys as blib.cached_arrays. Pure numpy.

    prebuild_trees.py [VARIANT[:LODS] ...] [--jobs N]     e.g.  eng_b:01 fir_b:0 pin_b    (default: every variant, LOD 0-3)
"""
import glob
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import ground
import trees

CDIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out/env/cache/trees")


def path_for(stem, key):
    return os.path.join(CDIR, f"{stem}_{key}.npz")


def build_one(job):
    kind, name, lod = job
    t0 = time.time()
    if kind == "tree":
        _, sp, H, cr, seed = next(v for v in trees.VARIANTS if v[0] == name)
        stem, key = f"{name}_l{lod}", trees.cache_key(sp, lod)
        build = lambda: trees.build_tree(sp, H, cr, seed, lod)  # noqa: E731
    else:
        _, uk, seed = next(v for v in ground.UNDER if v[0] == name)
        stem, key = name, ground.cache_key(uk)
        build = lambda: ground.build_under(uk, seed)  # noqa: E731
    p = path_for(stem, key)
    if os.path.exists(p):
        return f"{stem} cached"
    arr = build()
    np.savez(p + ".tmp.npz", **arr)
    os.replace(p + ".tmp.npz", p)
    for old in glob.glob(os.path.join(CDIR, f"{stem}.npz")) + glob.glob(os.path.join(CDIR, f"{stem}_??????????.npz")):
        if old != p:
            os.remove(old)
    return f"{stem} {len(arr['faces']) / 1e6:.2f}M tris {time.time() - t0:.0f}s"


def jobs_from(argv):
    jobs = []
    names = [a for a in argv if not a.startswith("--")]
    if not names:
        names = [v[0] for v in trees.VARIANTS] + [u[0] for u in ground.UNDER]
    under = {u[0] for u in ground.UNDER}
    for a in names:
        name, _, lods = a.partition(":")
        if name in under:
            jobs.append(("under", name, 0))
        else:
            jobs.extend(("tree", name, int(c)) for c in (lods or "0123"))
    return jobs


def main():
    os.makedirs(CDIR, exist_ok=True)
    n = int(sys.argv[sys.argv.index("--jobs") + 1]) if "--jobs" in sys.argv else 6
    argv = [a for i, a in enumerate(sys.argv[1:]) if a != "--jobs" and (i == 0 or sys.argv[i] != "--jobs")]
    jobs = jobs_from(argv)
    # biggest first, so the long hero builds start at once
    jobs.sort(key=lambda j: (j[2] != 0, j[0] != "tree"))
    with ProcessPoolExecutor(n) as ex:
        for msg in ex.map(build_one, jobs):
            print(msg, flush=True)


if __name__ == "__main__":
    main()
