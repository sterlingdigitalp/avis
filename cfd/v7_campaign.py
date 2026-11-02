#!/usr/bin/env python3
"""V7 runs, resumable. Finished points are never rerun.

    v7_campaign.py main [level]     all 13 alphas at both Re, on the gate grid
    v7_campaign.py grid             grid study: levels 0.667 and 1.5 at alpha 0, 4, 8, 10, both Re; with level 1 from main, three grids
    v7_campaign.py tu [level]       Tu sensitivity 0.05% and 0.15% at alpha 0, 4, 8, both Re
    v7_campaign.py transient [lvl]  time-accurate check of steady means at alpha 4 and 8, both Re
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pipeline

COORDS = os.path.join(pipeline.ROOT, "out/cfd/ref/e387.dat")
RES = (100000, 200000)


def points(pool, group, level, alphas, tu=0.10, res=RES, allow_transient=True):
    """One up-sweep in alpha per Re, warm-started point to point, as the tunnel runs were taken."""
    mesh = pipeline.build_mesh(COORDS, f"e387-L{level:g}", level)
    for r in res:
        pool.submit_sweep(group, [(f"e387-L{level:g}-Re{r}-a{a:g}-Tu{tu:g}", r, a) for a in sorted(alphas)], mesh, tu=tu,
                          allow_transient=allow_transient)


def main_phase(pool, level):
    points(pool, "v7", level, list(range(-2, 11)))


def grid_phase(pool, _level):
    points(pool, "v7-grid", 0.667, [0, 4, 8, 10])  # level 1 points come from the main group, same setup
    # Fine grid without the transient fallback: measured at 1.6 h per transient point on the shared host. Its points
    # report steady values, flagged unconverged-steady with the oscillation range when they do not settle.
    points(pool, "v7-grid", 1.5, [0, 4, 8, 10], allow_transient=False)


def tu_phase(pool, level):
    for tu in (0.05, 0.15):
        points(pool, "v7-tu", level, [0, 4, 8], tu=tu)


def transient_phase(pool, level):
    mesh = pipeline.build_mesh(COORDS, f"e387-L{level:g}", level)
    for r in RES:
        for a in (4, 8):
            pool.submit("v7-transient", f"e387-L{level:g}-Re{r}-a{a:g}-Tu0.1", mesh, re_=r, alpha=a, tu=0.10,
                        force_transient=True)


PHASES = dict(main=main_phase, grid=grid_phase, tu=tu_phase, transient=transient_phase)


def main(argv=None, pool=None):
    argv = sys.argv[1:] if argv is None else argv
    pool = pool or pipeline.Pool(2)
    PHASES[argv[0]](pool, float(argv[1]) if len(argv) > 1 else 1.0)
    pool.wait()


if __name__ == "__main__":
    main()
