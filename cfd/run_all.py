#!/usr/bin/env python3
"""The whole 2D programme in priority order, never more than two cases at once. Resumable at every point.

    run_all.py
Order: V7 main and grid study, the V7 gate, polars for the sections the physics code uses (e387 for 'e387-film'
wings, thin-plate for the tail), V7 Tu sensitivity and transient cross-check, the gate again, then the other sections. The V7 gate is scored after the
V7 phases; polar files are rewritten after each airfoil.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF = os.path.join(ROOT, "data/references/airfoils")
SECTIONS = os.path.join(ROOT, "out/cfd/ref/sections")
ORDER = [("e387", REF), ("thin-plate", SECTIONS), ("sd7037", REF), ("film-rib-4", SECTIONS), ("sd7003", REF),
         ("ag24", REF), ("mh32", REF), ("s5010", REF)]


def run(*args):
    print("::", " ".join(args), flush=True)
    subprocess.run([sys.executable, *args], check=False)


def polar(name, base):
    coords = os.path.join(base, name + ".dat")
    if not os.path.exists(coords):
        print(f":: skip {name}: no coordinates at {coords}", flush=True)
        return
    run(os.path.join(HERE, "polars.py"), "run", coords, "--name", name)
    run(os.path.join(HERE, "polars.py"), "write", name)


def main():
    # Order by what other workstreams wait on: the gate, its grid study, the two sections the physics code uses,
    # then the sensitivity and cross-check runs, then the remaining candidate sections.
    for phase in ("main", "grid"):
        run(os.path.join(HERE, "v7_campaign.py"), phase)
    run(os.path.join(HERE, "gates", "v7.py"))
    if not os.path.isdir(SECTIONS):
        run(os.path.join(HERE, "sections.py"), SECTIONS)
    for name, base in ORDER[:2]:
        polar(name, base)
    for phase in ("tu", "transient"):
        run(os.path.join(HERE, "v7_campaign.py"), phase)
    run(os.path.join(HERE, "gates", "v7.py"))
    for name, base in ORDER[2:]:
        polar(name, base)


if __name__ == "__main__":
    main()
